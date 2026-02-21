using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using KSP.Localization;

#if KERBALISM
using static KERBALISM.API;
#endif

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    [KSPAddon(KSPAddon.Startup.EveryScene, false)]
    public partial class OrbitalDecay : MonoBehaviour
    {
        private double lastUT;

        // Configuration Variables
        private bool debugMode = false;
        private double stormDecayRate = 1.5e-7;
        private bool stormDistanceScaling = true;
        private bool applyStormDecayToNoAtmosphereBody = false;

        private bool naturalDecayEnabled = true;
        private double naturalDecayMultiplier = 1.0;
        private double naturalDecayAltitudeCutoff = 10.0;

        private bool warningEnabled = true;
        private double warningThreshold = 0.2;
        private double reentryDestroySeconds = 60.0;

        // Constants: 1 Astronomical Unit in meters (Kerbin SMA)
        private const double AU = 13599840256; 

        // State Variables
        private HashSet<Guid> lowOrbitWarned = new HashSet<Guid>();
        private HashSet<Guid> lowPeriapsisWarned = new HashSet<Guid>();
        private Dictionary<Guid, double> pendingDestroyTimers = new Dictionary<Guid, double>();
        private Dictionary<Guid, double> pendingDestroyNextMessageTimes = new Dictionary<Guid, double>();

        // UI Variables
        private Rect windowRect;
        private bool isWindowInitialized = false;
        private Vector2 scrollPosition;
        private bool showGui = false;
        private bool debugForceStorm = false;
        
        // UI Settings
        private float uiScale = 1.0f;
        private int fontSize = 13;
        private const float baseWindowWidth = 500f;
        private const int baseFontSize = 13;
        private const float windowRightMargin = 20f;
        private bool showSettings = false;
        private KeyCode toggleKey = KeyCode.Q;
        private bool isRebinding = false;

        // Cached GUIStyles for DrawWindow (rebuilt when fontSize changes)
        private GUIStyle _cachedBold;
        private GUIStyle _cachedNormal;
        private GUIStyle _cachedRed;
        private GUIStyle _cachedGreen;
        private GUIStyle _cachedYellow;
        private GUIStyle _cachedSubHeader;
        private GUIStyle _cachedVesselNameStyle;
        private GUIStyle _cachedButton;
        private int _cachedFontSizeForStyles = -1;
        
        // UI Filter
        private enum FilterMode { All, Stable, Natural, Storm }
        private FilterMode currentFilter = FilterMode.All;
        private enum DebrisFilterMode { All, OnlyDebris, ExcludeDebris }
        private DebrisFilterMode currentDebrisFilter = DebrisFilterMode.All;
        private int currentBodyFilterIndex = 0;
        private List<CelestialBody> bodyFilterBodies = new List<CelestialBody>();
        private string[] bodyFilterNames = Array.Empty<string>();
        private int bodyFilterSignature = 0;
        private bool showBodyFilterPopup = false;
        private Rect bodyFilterPopupRect;
        private Vector2 bodyFilterPopupScrollPosition;
        private const float BODY_BUTTON_PADDING = 16f;
        private const float BODY_POPUP_MAX_WIDTH = 350f;
        private const float BODY_POPUP_DEFAULT_WIDTH = 220f;
        private const float BODY_POPUP_LIST_HEIGHT = 340f;

        // Caching for Performance
        private Dictionary<Guid, double> cachedDecayTimesPe = new Dictionary<Guid, double>();
        private Dictionary<Guid, float> lastDecayCalcTimePe = new Dictionary<Guid, float>();
        private Dictionary<Guid, double> cachedDecayTimesAp = new Dictionary<Guid, double>();
        private Dictionary<Guid, float> lastDecayCalcTimeAp = new Dictionary<Guid, float>();
        private const float CACHE_INTERVAL = 1.0f;
        private List<Vessel> cachedVesselOrder = new List<Vessel>();
        private int cachedValidVesselCount = 0;
        private int cachedVesselSignature = 0;
        private Guid cachedActiveVesselId = Guid.Empty;
        private List<VesselDisplayState> cachedVisibleVessels = new List<VesselDisplayState>();
        private bool uiCacheDirty = true;
        private float uiLastRefreshTime = -1f;
        private const float UI_CACHE_INTERVAL = 0.25f;

        void Start()
        {
            lastUT = Planetarium.GetUniversalTime();
            LoadSettings();
            LoadUISettings();
        }

        void LoadSettings()
        {
            // Look for the configuration node
            ConfigNode[] nodes = GameDatabase.Instance.GetConfigNodes("ORBITAL_DECAY");
            if (nodes != null && nodes.Length > 0)
            {
                ConfigNode cfg = nodes[0];

                // Storm
                cfg.TryGetValue("stormDecayRate", ref stormDecayRate);
                cfg.TryGetValue("stormDistanceScaling", ref stormDistanceScaling);
                cfg.TryGetValue("applyStormDecayToNoAtmosphereBody", ref applyStormDecayToNoAtmosphereBody);

                // Natural
                cfg.TryGetValue("naturalDecayEnabled", ref naturalDecayEnabled);
                cfg.TryGetValue("naturalDecayMultiplier", ref naturalDecayMultiplier);
                cfg.TryGetValue("naturalDecayAltitudeCutoff", ref naturalDecayAltitudeCutoff);

                // Warnings
                cfg.TryGetValue("warningEnabled", ref warningEnabled);
                cfg.TryGetValue("warningThreshold", ref warningThreshold);
                cfg.TryGetValue("reentryDestroySeconds", ref reentryDestroySeconds);
                if (reentryDestroySeconds <= 0) reentryDestroySeconds = 60.0;

                Debug.Log($"[KerbalismOrbitalDecay] Settings Loaded: StormRate={stormDecayRate}, NatEnabled={naturalDecayEnabled}, Warn={warningEnabled}");
            }
            else
            {
                Debug.Log("[KerbalismOrbitalDecay] No settings file found, using defaults.");
            }
        }

        void LoadUISettings()
        {
            string path = KSPUtil.ApplicationRootPath + "GameData/SpaceWeatherAndAtmosphericOrbitalDecay/PluginData/UISettings.cfg";
            if (File.Exists(path))
            {
                ConfigNode node = ConfigNode.Load(path);
                if (node != null)
                {
                    if (node.HasValue("uiScale")) 
                    {
                        float s;
                        if (float.TryParse(node.GetValue("uiScale"), out s)) uiScale = Mathf.Clamp(s, 0.5f, 3.0f);
                    }
                    if (node.HasValue("fontSize")) 
                    {
                        int f;
                        if (int.TryParse(node.GetValue("fontSize"), out f)) fontSize = Mathf.Clamp(f, 8, 40);
                    }
                    
                    if (node.HasValue("toggleKey"))
                    {
                        try { toggleKey = (KeyCode)Enum.Parse(typeof(KeyCode), node.GetValue("toggleKey")); }
                        catch { toggleKey = KeyCode.Q; }
                    }

                    float x = 0, y = 0;
                    bool hasPos = false;
                    if (node.HasValue("windowX")) { float.TryParse(node.GetValue("windowX"), out x); hasPos = true; }
                    if (node.HasValue("windowY")) { float.TryParse(node.GetValue("windowY"), out y); hasPos = true; }
                    
                    if (hasPos)
                    {
                        windowRect = new Rect(x, y, GetWindowWidthForFontSize(fontSize), 0);
                        isWindowInitialized = true;
                    }
                }
            }
        }

        void SaveUISettings()
        {
            ConfigNode node = new ConfigNode("UI_SETTINGS");
            node.AddValue("uiScale", uiScale);
            node.AddValue("fontSize", fontSize);
            node.AddValue("toggleKey", toggleKey.ToString());
            node.AddValue("windowX", windowRect.x);
            node.AddValue("windowY", windowRect.y);
            
            string dir = KSPUtil.ApplicationRootPath + "GameData/SpaceWeatherAndAtmosphericOrbitalDecay/PluginData/";
            if (!Directory.Exists(dir)) Directory.CreateDirectory(dir);
            
            node.Save(dir + "UISettings.cfg");
            ScreenMessages.PostScreenMessage(Localizer.Format("#SWAOD_Msg_UISaved"), 3.0f, ScreenMessageStyle.UPPER_CENTER);
        }

        void Update()
        {
            // Only run in Flight, TrackingStation, or SpaceCentre
            if (HighLogic.LoadedScene != GameScenes.FLIGHT &&
                HighLogic.LoadedScene != GameScenes.TRACKSTATION &&
                HighLogic.LoadedScene != GameScenes.SPACECENTER)
            {
                return;
            }

            // Initialize Window Position (Right side of screen)
            if (!isWindowInitialized)
            {
                float initialWidth = GetWindowWidthForFontSize(fontSize);
                windowRect = new Rect(Screen.width - initialWidth - windowRightMargin, 100, initialWidth, 0);
                isWindowInitialized = true;
            }

            // Toggle UI with Alt + UserKey
            if ((Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt)) && Input.GetKeyDown(toggleKey))
            {
                showGui = !showGui;
                uiCacheDirty = true;
                uiLastRefreshTime = -1f;
            }

            double currentUT = Planetarium.GetUniversalTime();
            double dt = currentUT - lastUT;
            lastUT = currentUT;

            // If time is paused or moving backwards, do nothing
            if (dt <= 0) return;

            // Iterate over all vessels in the game
            for (int i = FlightGlobals.Vessels.Count - 1; i >= 0; i--)
            {
                Vessel v = FlightGlobals.Vessels[i];
                if (!IsValidVessel(v)) continue;

                // Solar Storm Decay Logic
                bool stormActive = false;
#if KERBALISM
                stormActive = StormInProgress(v);
#endif
                stormActive = stormActive || debugForceStorm;

                if (stormActive)
                {
                    ApplyStormDecay(v, dt, currentUT);
                }

                // Natural Atmospheric Decay Logic
                if (naturalDecayEnabled)
                {
                    ApplyNaturalDecay(v, dt, currentUT);
                }
                
            }

            if (showGui)
            {
                RefreshUiCache(false);
            }
        }

        private string FormatTime(double seconds)
        {
            double dayLen = GameSettings.KERBIN_TIME ? 21600.0 : 86400.0;
            double yearLen = dayLen * (GameSettings.KERBIN_TIME ? 426.0 : 365.0);

            if (seconds > yearLen * 100) return Localizer.Format("#SWAOD_Time_GT100y");

            if (seconds > yearLen)
            {
                int years = (int)(seconds / yearLen);
                int days = (int)((seconds % yearLen) / dayLen);
                return Localizer.Format("#SWAOD_Time_YearsDays", years, days);
            }
            
            if (seconds > dayLen)
            {
                int days = (int)(seconds / dayLen);
                int hours = (int)((seconds % dayLen) / 3600.0);
                return Localizer.Format("#SWAOD_Time_DaysHours", days, hours);
            }
            
            if (seconds > 3600)
            {
                int hours = (int)(seconds / 3600.0);
                int mins = (int)((seconds % 3600.0) / 60.0);
                return Localizer.Format("#SWAOD_Time_HoursMins", hours, mins);
            }

            int m = (int)(seconds / 60.0);
            int s = (int)(seconds % 60.0);
            return Localizer.Format("#SWAOD_Time_MinsSecs", m, s);
        }

        private string FormatAltitude(double meters)
        {
            double absMeters = Math.Abs(meters);
            if (absMeters >= 1000000.0)
            {
                return (meters / 1000000.0).ToString("F3") + " Mm";
            }
            if (absMeters >= 1000.0)
            {
                return (meters / 1000.0).ToString("F3") + " km";
            }
            return meters.ToString("F1") + " m";
        }

        private void DestroyLoadedVessel(Vessel v)
        {
            if (v == null || !v.loaded) return;
            List<Part> parts = v.Parts;
            if (parts == null || parts.Count == 0)
            {
                v.Die();
                return;
            }
            for (int i = parts.Count - 1; i >= 0; i--)
            {
                Part p = parts[i];
                if (p != null) p.explode();
            }
            v.Die();
        }

        private bool IsValidVessel(Vessel v)
        {
            if (v == null || v.state == Vessel.State.DEAD) return false;

            // Filter out flags, space objects (asteroids/comets), and unknown types
            if (v.vesselType == VesselType.Flag ||
                v.vesselType == VesselType.SpaceObject ||
                v.vesselType == VesselType.Unknown) return false;

            if (v.loaded && v == FlightGlobals.ActiveVessel) return true;

            if (v.situation != Vessel.Situations.ORBITING && v.situation != Vessel.Situations.SUB_ORBITAL) return false;

            return true;
        }

        private double GetDistanceToSun(Vessel v)
        {
            CelestialBody sun = FlightGlobals.Bodies[0];
            if (v.mainBody == sun) return v.orbit.radius;

            Vector3d sunPos = sun.position;
            Vector3d bodyPos = v.mainBody.position;
            Vector3d relPos = v.orbit.pos;
            
            return Vector3d.Distance(bodyPos + relPos, sunPos);
        }
    }
}
