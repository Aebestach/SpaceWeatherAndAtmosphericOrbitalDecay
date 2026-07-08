using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.Localization;

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    [KSPAddon(KSPAddon.Startup.EveryScene, false)]
    public partial class OrbitalDecay : MonoBehaviour
    {
        private double lastUT;

        // Configuration Variables
        private double stormDecayRate = 1.5e-7;
        private bool stormDistanceScaling = true;

        private bool naturalDecayEnabled = true;
        private double naturalDecayMultiplier = 1.0;
        private double naturalDecayAltitudeCutoff = 10.0;
        private double exosphereFitStart = 0.80;
        private double exosphereFitEnd = 0.90;
        private double exosphereScaleHeightMin = 0.03;
        private double exosphereScaleHeightMax = 0.30;
        private int exosphereFitSamples = 8;
        private int orbitAverageSamples = 24;

        private bool warningEnabled = true;
        private double warningThreshold = 0.2;
        private double reentryDestroySeconds = 60.0;

        // Constants: 1 Astronomical Unit in meters (Kerbin SMA)
        private const double AU = 13599840256; 

        // State Variables
        private HashSet<Guid> vesselDecayDisabled = new HashSet<Guid>();
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
        private float _lastUiScaleFactor = -1f;
        private const int FontSize = 18;
        private const float BaseWindowWidth = 600f;
        private const float WindowRightMargin = 20f;
        private bool showSettings = false;

        // Cached GUIStyles for DrawWindow (rebuilt when fontSize changes)
        private GUIStyle _cachedBold;
        private GUIStyle _cachedNormal;
        private GUIStyle _cachedRow;
        private GUIStyle _cachedRed;
        private GUIStyle _cachedGreen;
        private GUIStyle _cachedYellow;
        private GUIStyle _cachedSubHeader;
        private GUIStyle _cachedVesselNameStyle;
        private GUIStyle _cachedButton;
        private GUIStyle _cachedBoxStyle;
        private int _cachedFontSizeForStyles = -1;
        
        // UI Filter
        private enum FilterMode { All, Stable, Natural, Storm }
        private FilterMode currentFilter = FilterMode.All;
        private enum DebrisFilterMode { All, OnlyDebris, ExcludeDebris }
        private DebrisFilterMode currentDebrisFilter = DebrisFilterMode.All;
        private int currentBodyFilterIndex = 0;
        private List<CelestialBody> bodyFilterBodies = new List<CelestialBody>();
        private string[] bodyFilterNames = Array.Empty<string>();
        private bool bodyFilterCacheInitialized = false;
        private bool showBodyFilterPopup = false;
        private Rect bodyFilterPopupRect;
        private Vector2 bodyFilterPopupScrollPosition;
        private const float BODY_BUTTON_PADDING = 16f;
        private const float BODY_POPUP_MAX_WIDTH = 350f;
        private const float BODY_POPUP_DEFAULT_WIDTH = 220f;
        private const float BODY_POPUP_LIST_HEIGHT = 340f;
        private const float VESSEL_LIST_HEIGHT = 600f;
        private const float VESSEL_ENTRY_SPACING = 4f;

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
            SwaodParameters.Instance?.ApplyAutoUiScale();
            if (TryGetUniversalTime(out double startUt))
                lastUT = startUt;
            KerbalismIntegration.Initialize();
            SyncSettingsFromParameters();
        }

        void SyncSettingsFromParameters()
        {
            SwaodGameplayParameters parameters = SwaodGameplayParameters.Instance;
            if (parameters == null)
                return;

            parameters.ApplyTo(
                ref stormDecayRate,
                ref stormDistanceScaling,
                ref naturalDecayEnabled,
                ref naturalDecayMultiplier,
                ref naturalDecayAltitudeCutoff,
                ref exosphereFitStart,
                ref exosphereFitEnd,
                ref exosphereScaleHeightMin,
                ref exosphereScaleHeightMax,
                ref exosphereFitSamples,
                ref orbitAverageSamples,
                ref warningEnabled,
                ref warningThreshold,
                ref reentryDestroySeconds);

            Debug.Log($"[KerbalismOrbitalDecay] Settings Loaded: StormRate={stormDecayRate}, NatEnabled={naturalDecayEnabled}, Warn={warningEnabled}");
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
                float initialWidth = GetWindowWidthForFontSize(FontSize);
                Vector2 screen = UIScale.GuiScreenSize();
                windowRect = new Rect(screen.x - initialWidth - WindowRightMargin, 100, initialWidth, 0);
                isWindowInitialized = true;
            }

            var parameters = SwaodParameters.Instance;
            if (parameters != null && parameters.IsHotkeyPressed())
            {
                showGui = !showGui;
                uiCacheDirty = true;
                uiLastRefreshTime = -1f;
            }

            if (!TryGetUniversalTime(out double currentUT))
                return;
            if (lastUT <= 0d)
            {
                lastUT = currentUT;
                return;
            }
            double dt = currentUT - lastUT;
            lastUT = currentUT;

            // If time is paused or moving backwards, do nothing
            if (dt <= 0) return;

            // Iterate over all vessels in the game
            for (int i = FlightGlobals.Vessels.Count - 1; i >= 0; i--)
            {
                Vessel v = FlightGlobals.Vessels[i];
                if (!IsValidVessel(v)) continue;

                bool stormActive = KerbalismIntegration.IsStormInProgress(v) ||
                    (SwaodParameters.IsDebugModeEnabled && debugForceStorm);
                if ((naturalDecayEnabled || stormActive) && !vesselDecayDisabled.Contains(v.id))
                {
                    ApplyNaturalDecay(v, dt, currentUT, stormActive);
                }
                
            }

            if (showGui)
            {
                RefreshUiCache(false);
            }
        }

        private static bool TryGetUniversalTime(out double universalTime)
        {
            universalTime = 0d;
            try
            {
                universalTime = Planetarium.GetUniversalTime();
                return true;
            }
            catch (NullReferenceException)
            {
                return false;
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

        private AtmosphericDecayModel.DecaySettings GetDecaySettings()
        {
            AtmosphericDecayModel.DecaySettings settings = AtmosphericDecayModel.GetDefaultSettings();
            settings.NaturalDecayMultiplier = naturalDecayMultiplier;
            settings.NaturalDecayAltitudeCutoff = naturalDecayAltitudeCutoff;
            settings.ExosphereFitStart = exosphereFitStart;
            settings.ExosphereFitEnd = exosphereFitEnd;
            settings.ExosphereScaleHeightMin = exosphereScaleHeightMin;
            settings.ExosphereScaleHeightMax = exosphereScaleHeightMax;
            settings.ExosphereFitSamples = exosphereFitSamples;
            settings.OrbitAverageSamples = orbitAverageSamples;
            return settings;
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
