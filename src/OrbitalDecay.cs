using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using KSP.Localization;
using ClickThroughFix;

#if KERBALISM
using static KERBALISM.API;
#endif

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    [KSPAddon(KSPAddon.Startup.EveryScene, false)]
    public class OrbitalDecay : MonoBehaviour
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
        private const float BODY_POPUP_DEFAULT_HEIGHT = 480f;
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

        private struct VesselDisplayState
        {
            public Vessel Vessel;
            public bool IsStorming;
            public bool IsForced;
            public bool IsNatural;
            public bool StormInRange;
            public double EffectiveStormRate;
            public double CurrentStormRate;
            public string VesselNameRich;
            public string BodyName;
            public string PeAltText;
            public string ApAltText;
            public string PeTimeText;
            public string ApTimeText;
            public string IncText;
            public string EccText;
            public string StatusText;
            public string StormRateText;
            public bool ShowDestroyButton;
        }

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

        // --- DECAY LOGIC ------------------------------------------------------------
        private void ApplyStormDecay(Vessel v, double dt, double currentUT)
        {
            // Check if we should apply decay to bodies without atmosphere
            if (!applyStormDecayToNoAtmosphereBody && !v.mainBody.atmosphere)
            {
                return;
            }

            if (v.mainBody.atmosphere)
            {
                double maxAlt = v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff;
                if (v.altitude > maxAlt) return;
            }
            else
            {
                double maxAlt = v.mainBody.sphereOfInfluence - v.mainBody.Radius;
                if (v.altitude > maxAlt) return;
            }

            if (v.loaded && v.mainBody.atmosphere && v.altitude < v.mainBody.atmosphereDepth * 0.85)
            {
                return;
            }

            if (v.loaded && v.situation != Vessel.Situations.ORBITING) return;

            Orbit o = v.orbit;

            // Distance Scaling: Inverse Square Law relative to Kerbin (1 AU)
            double distanceFactor = 1.0;
            if (stormDistanceScaling)
            {
                double dist = GetDistanceToSun(v);
                dist = Math.Max(dist, 1000.0);
                distanceFactor = Math.Pow(AU / dist, 2);
            }

            // Calculate decay factor, Rate is modified by distance
            double effectiveRate = stormDecayRate * distanceFactor;
            double decayFactor = Math.Exp(-effectiveRate * dt);
            double deltaSMA = o.semiMajorAxis * (decayFactor - 1.0);
            ApplyDecayToVessel(v, o, deltaSMA, decayFactor, currentUT);
        }

        private void ApplyNaturalDecay(Vessel v, double dt, double currentUT)
        {
            // Only applies if the body has an atmosphere
            if (!v.mainBody.atmosphere) return;

            Orbit o = v.orbit;
            double atmDepth = v.mainBody.atmosphereDepth;
            double altitude = v.altitude;

            // Handle Atmospheric Entry (Loaded & Unloaded)
            if (altitude < atmDepth)
            {
                if (v.loaded)
                {
                    if (!lowOrbitWarned.Contains(v.id))
                    {
                        if (v.vesselType != VesselType.Debris)
                        {
                            ScreenMessages.PostScreenMessage(Localizer.Format("#SWAOD_Warning_EnteredAtm", v.vesselName), 5.0f, ScreenMessageStyle.UPPER_CENTER);
                        }
                        lowOrbitWarned.Add(v.id);
                    }
                    if (pendingDestroyTimers.Remove(v.id))
                        pendingDestroyNextMessageTimes.Remove(v.id);
                    return;
                }
                
                if (!pendingDestroyTimers.TryGetValue(v.id, out double timeLeft))
                {
                    timeLeft = reentryDestroySeconds;
                    pendingDestroyNextMessageTimes[v.id] = currentUT;
                }
                timeLeft -= dt;
                pendingDestroyTimers[v.id] = timeLeft;

                if (pendingDestroyNextMessageTimes.TryGetValue(v.id, out double nextMsgTime) && currentUT >= nextMsgTime)
                {
                    if (v.vesselType != VesselType.Debris)
                    {
                        string msg = Localizer.Format("#SWAOD_Msg_ReEntry_Body", v.vesselName, FormatTime(reentryDestroySeconds)) + "  T-" + FormatTime(Math.Max(0.0, timeLeft)) + "/" + FormatTime(reentryDestroySeconds);
                        ScreenMessages.PostScreenMessage(msg, 1.0f, ScreenMessageStyle.UPPER_CENTER);
                    }
                    pendingDestroyNextMessageTimes[v.id] = currentUT + 1.0;
                }
                
                if (timeLeft <= 0)
                {
                    if (v.vesselType != VesselType.Debris)
                        ScreenMessages.PostScreenMessage(Localizer.Format("#SWAOD_Msg_Destroyed_Body", v.vesselName), 10.0f, ScreenMessageStyle.UPPER_CENTER);
                    v.Die();
                    pendingDestroyTimers.Remove(v.id);
                    pendingDestroyNextMessageTimes.Remove(v.id);
                    return;
                }
            }
            else
            {
                if (pendingDestroyTimers.Remove(v.id))
                    pendingDestroyNextMessageTimes.Remove(v.id);
            }

            if (v.loaded && altitude < atmDepth * 0.85) return;

            if (v.loaded && v.situation != Vessel.Situations.ORBITING) return;

            double maxAlt = atmDepth * naturalDecayAltitudeCutoff;
            if (altitude > maxAlt) return;

            double density = GetExosphericDensity(v.mainBody, altitude);

            if (density <= 1e-22) density = 1e-22; 

            // Physics-based Drag Decay
            // Fd = 0.5 * rho * v^2 * Cd * A

            // Velocity calculation (Use Vis-viva equation for accurate orbital speed)
            // v = sqrt(mu * (2/r - 1/a))
            double mu = v.mainBody.gravParameter;
            double a = v.orbit.semiMajorAxis;

            double r = v.altitude + v.mainBody.Radius;

            // Vis-viva equation
            double vSq = mu * (2.0 / r - 1.0 / a);
            double vel = Math.Sqrt(Math.Max(0, vSq));

            // Estimate Area/Mass ratio (Ballistic Coefficient)
            double mass = v.GetTotalMass();
            if (mass <= 0.001) mass = 0.1;

            double m_kg = mass * 1000.0;
            double area = Math.Pow(mass, 0.666) * 4.0; // Rough estimation
            double Cd = 2.0;

            // Fd (Newtons) = 0.5 * rho * v^2 * Cd * A
            double dragForceNewtons = 0.5 * density * (vel * vel) * Cd * area;

            // Energy Decay Rate: dE/dt = -Fd * v
            // da/dt = - (2 * a^2 * v * Fd) / (mu * m)

            // Instantaneous change in SMA per second (Meters per Second)
            double da_dt = -(2.0 * a * a * vel * dragForceNewtons) / (mu * m_kg);

            // Apply Multiplier
            da_dt *= naturalDecayMultiplier;

            if (v.loaded && altitude < atmDepth && altitude > atmDepth - 2000.0)
            {
                da_dt *= 10.0;
            }


            double deltaSMA = 0;
            double remainingDt = dt;
            
            
            while (remainingDt > 0)
            {
                double stepDt = remainingDt;
                
                if (stepDt > 3600.0) stepDt = 3600.0;
                
                double current_da_dt = da_dt; 

                double estimatedDrop = Math.Abs(da_dt * stepDt);
                
                if (estimatedDrop > 100.0)
                {
                    double scaleHeight = 7000.0;
                    double boostFactor = 1.0 + (estimatedDrop / (2.0 * scaleHeight));
                    
                    current_da_dt *= boostFactor;
                }
                
                deltaSMA += current_da_dt * stepDt;
                remainingDt -= stepDt;
            }

            double criticalAlt = v.mainBody.atmosphereDepth * (1.0 + warningThreshold); // e.g. 1.2 * 70km = 84km
            if (warningEnabled && v.orbit.PeA < criticalAlt && !lowPeriapsisWarned.Contains(v.id))
            {
                if (v.vesselType != VesselType.Debris)
                {
                    ScreenMessages.PostScreenMessage(Localizer.Format("#SWAOD_Warning_LowOrbit", v.vesselName, FormatAltitude(criticalAlt)), 5.0f, ScreenMessageStyle.UPPER_CENTER);
                }
                lowPeriapsisWarned.Add(v.id);
            }


            double currentBuffer = altitude - atmDepth;
            
            if (currentBuffer > 0)
            {
                 // If we are very high, limit the drop to avoid skipping atmosphere entirely in one frame
                 double maxSafeDrop = Math.Max(currentBuffer * 0.5, 10000.0);
                 
                 if (-deltaSMA > maxSafeDrop)
                 {
                     deltaSMA = -maxSafeDrop;
                 }
            }

            double newSMA = a + deltaSMA;
            double minSMA = v.mainBody.Radius + 100.0;

            if (newSMA < minSMA)
                newSMA = minSMA;

            deltaSMA = newSMA - a;
            double decayRatio = newSMA / a;
            ApplyDecayToVessel(v, o, deltaSMA, decayRatio, currentUT);
        }

        private void ApplyDecayToVessel(Vessel v, Orbit o, double deltaSMA, double eccFactor, double currentUT)
        {
            if (Math.Abs(deltaSMA) < 1e-10) return;

            if (v.loaded && !v.packed)
            {
                double a = o.semiMajorAxis;
                double r = v.altitude + v.mainBody.Radius;
                double mu = v.mainBody.gravParameter;
                double vSq = mu * (2.0 / r - 1.0 / a);
                double speed = Math.Sqrt(Math.Max(0, vSq));

                if (speed > 1e-6)
                {
                    Vector3d velVec = v.obt_velocity;
                    if (velVec.sqrMagnitude > 1e-10)
                    {
                        double dv = (mu / (2.0 * a * a * speed)) * deltaSMA;
                        Vector3d dvVec = velVec.normalized * dv;
                        v.ChangeWorldVelocity(dvVec);
                    }
                }
                return;
            }

            double smaFactor = 1.0 + (deltaSMA / o.semiMajorAxis);
            ModifyOrbit(o, smaFactor, eccFactor, currentUT);
        }

        private double GetExosphericDensity(CelestialBody body, double altitude)
        {
            double atmDepth = body.atmosphereDepth;
            double maxCutoffAlt = atmDepth * naturalDecayAltitudeCutoff;

            if (altitude > maxCutoffAlt) return 0.0;

            double h_base = atmDepth * 0.95;
            double p_base = body.GetPressure(h_base);
            double t_base = body.GetTemperature(h_base);
            double rho_base = FlightGlobals.getAtmDensity(p_base, t_base, body);
            
            if (rho_base < 1e-15) rho_base = 1e-15;
            double curveRefScale = 10.0; 
            double curveMaxAlt = atmDepth * curveRefScale;
            double rho_target = 1e-14;   
            
            if (altitude <= h_base)
            {
                 double p = body.GetPressure(altitude);
                 double t = body.GetTemperature(altitude);
                 return FlightGlobals.getAtmDensity(p, t, body);
            }
            
            double t_factor = (altitude - h_base) / (curveMaxAlt - h_base);
            double t_curved = Math.Pow(t_factor, 0.5);
            double logRho = Math.Log(rho_base) * (1.0 - t_curved) + Math.Log(rho_target) * t_curved;
            
            return Math.Exp(logRho);
        }

        private void ModifyOrbit(Orbit o, double smaFactor, double eccFactor, double currentUT)
        {
            double currentMeanAnomaly = o.meanAnomaly;

            double oldA = o.semiMajorAxis;
            double oldE = o.eccentricity;
            double oldRp = oldA * (1.0 - oldE);

            double newA = oldA * smaFactor;
            double newE = oldE * eccFactor;

            if (oldE >= 0.0 && oldE < 1.0 && newA > 0.0)
            {
                double minE = 1.0 - (oldRp / newA);
                if (minE > newE) newE = minE;
            }

            if (newE < 0.0) newE = 0.0;
            if (oldE < 1.0 && newE >= 1.0) newE = 0.999999;

            o.semiMajorAxis = newA;
            o.eccentricity = newE;

            o.epoch = currentUT;
            o.meanAnomalyAtEpoch = currentMeanAnomaly;

            o.Init();
            o.UpdateFromUT(currentUT);
        }

        private double EstimateDecayTime(Vessel v, double startAlt, double effectiveStormRate)
        {
            // Reentry time calculated based on Periapsis, ignoring Apoapsis.
            CelestialBody body = v.mainBody;
            double atmDepth = body.atmosphereDepth;
            double targetAlt = atmDepth;
            
            if (startAlt <= atmDepth) return 0;

            if (!naturalDecayEnabled && effectiveStormRate > 0)
            {
                double startR = body.Radius + startAlt;
                double endR = body.Radius + atmDepth;
                double time = Math.Log(startR / endR) / effectiveStormRate;
                if (time < 0) time = 0;
                return time;
            }
            
            double simAlt = startAlt;
            double totalTime = 0;
            double range = startAlt - atmDepth;
            double stepAlt = range / 10.0;
            
            // Limit max iterations for performance
            int maxSteps = 100; 
            double stepScale = atmDepth * 0.2;
            if (stepScale > 0)
            {
                double estimatedSteps = range / stepScale;
                if (estimatedSteps < 20.0) maxSteps = 20;
                else if (estimatedSteps < 100.0) maxSteps = (int)Math.Ceiling(estimatedSteps);
            }
            stepAlt = range / (double)maxSteps;
            
            double mu = body.gravParameter;
            double R = body.Radius;
            double mass = v.GetTotalMass();
            if (mass <= 0.001) mass = 0.1;
            double m_kg = mass * 1000.0;
            if (m_kg < 1.0) m_kg = 100.0; // Safety
            double area = Math.Pow(mass, 0.666) * 4.0;
            double Cd = 2.0;

            for (int i = 0; i < maxSteps; i++)
            {
                double midAlt = simAlt - (stepAlt * 0.5);
                double simR = R + midAlt;
                double simSMA = simR;
                double simVel = Math.Sqrt(mu / simR);
                
                double simRho = GetExosphericDensity(body, midAlt);

                if (simRho < 1e-22) simRho = 1e-22;

                double simDrag = 0.5 * simRho * simVel * simVel * Cd * area;
                double sim_da_dt_natural = -(2.0 * simSMA * simSMA * simVel * simDrag) / (mu * m_kg);
                sim_da_dt_natural *= naturalDecayMultiplier;

                double sim_da_dt_storm = -simSMA * effectiveStormRate;
                
                double sim_da_dt = sim_da_dt_natural + sim_da_dt_storm;
                
                if (Math.Abs(sim_da_dt) < 1e-20) 
                {
                    totalTime += 1e9;
                    break; 
                }

                double dt = stepAlt / Math.Abs(sim_da_dt);
                
                totalTime += dt;
                simAlt -= stepAlt;
                
                if (simAlt <= atmDepth) break;
            }
            
            return totalTime;
        }

        private double GetCachedDecayTime(Vessel v, double startAlt, double effectiveStormRate, Dictionary<Guid, double> cache, Dictionary<Guid, float> timeCache)
        {
            float currentTime = Time.realtimeSinceStartup;

            // Check cache validity
            if (cache.TryGetValue(v.id, out double cachedTime))
            {
                if (timeCache.TryGetValue(v.id, out float lastTime))
                {
                    if (currentTime - lastTime < CACHE_INTERVAL)
                    {
                        return cachedTime;
                    }
                }
            }

            double newTime = EstimateDecayTime(v, startAlt, effectiveStormRate);

            cache[v.id] = newTime;
            timeCache[v.id] = currentTime;

            return newTime;
        }

        private string GetDecayTimeDisplay(Vessel v, double altitude, bool isStorming, bool isForced, double effectiveStormRate, Dictionary<Guid, double> cache, Dictionary<Guid, float> timeCache)
        {
            if (!v.mainBody.atmosphere) return Localizer.Format("#SWAOD_NotAvailable");

            double atmDepth = v.mainBody.atmosphereDepth;
            if (altitude <= atmDepth) return Localizer.Format("#SWAOD_ReEntry");

            double maxDecayAlt = atmDepth * naturalDecayAltitudeCutoff;
            if (altitude > maxDecayAlt) return Localizer.Format("#SWAOD_NotAvailable");

            bool canDecay = isStorming || isForced;
            if (!canDecay && naturalDecayEnabled)
            {
                if (altitude < maxDecayAlt) canDecay = true;
            }

            if (!canDecay) return Localizer.Format("#SWAOD_NotAvailable");

            double timeSeconds = GetCachedDecayTime(v, altitude, effectiveStormRate, cache, timeCache);
            return FormatTime(timeSeconds);
        }

        // --- UI LOGIC ---------------------------------------------------------------
        private void DumpAtmosphereLogs()
        {
            Vessel v = FlightGlobals.ActiveVessel;
            if (v == null || !v.mainBody.atmosphere) return;
            
            CelestialBody b = v.mainBody;
            double atmDepth = b.atmosphereDepth;
            
            Debug.Log($"[OrbitalDecay] Atmosphere Dump for {b.name}");
            Debug.Log($"[OrbitalDecay] AtmDepth: {atmDepth}");
            
            double maxAlt = atmDepth * naturalDecayAltitudeCutoff;
            
            double range = maxAlt - (atmDepth * 0.8);
            double step = range > 0 ? Math.Max(atmDepth * 0.05, range / 40.0) : atmDepth * 0.05;

            for (double alt = atmDepth * 0.8; alt <= maxAlt; alt += step)
            {
                double p = b.GetPressure(alt);
                double t = b.GetTemperature(alt);
                double rho = FlightGlobals.getAtmDensity(p, t, b);
                double calcRho = GetExosphericDensity(b, alt);
            
                
                double R = b.Radius;
                double r = R + alt;
                double a = r;
                double mu = b.gravParameter;
                double orbVel = Math.Sqrt(mu / r);
                double m = 1000.0;
                double A = 2.0;
                double Cd = 2.0;
                
                double Fd = 0.5 * calcRho * orbVel * orbVel * Cd * A;
                double da_dt = -(2.0 * a * a * orbVel * Fd) / (mu * m);
                da_dt *= naturalDecayMultiplier;
                
                string timeStr = "N/A";
                if (Math.Abs(da_dt) > 1e-10)
                {     
                    double timeSec = 10000.0 / Math.Abs(da_dt); 
                    timeStr = FormatTime(timeSec);
                }

                Debug.Log($"[OrbitalDecay] Alt: {alt/1000:F1}km | P: {p:E2} | Rho: {calcRho:E2} | da/dt: {da_dt:E2} m/s | 10km Drop Time (Instant): {timeStr}");
            }
            
            double vSq = v.mainBody.gravParameter * (2.0 / (v.altitude + v.mainBody.Radius) - 1.0 / v.orbit.semiMajorAxis);
            double vel = Math.Sqrt(Math.Max(0, vSq));
            double area = Math.Pow(v.GetTotalMass(), 0.666) * 4.0;
            double dens = GetExosphericDensity(v.mainBody, v.altitude);
            double drag = 0.5 * dens * vSq * 2.0 * area;
            double natural_da_dt = -(2.0 * v.orbit.semiMajorAxis * v.orbit.semiMajorAxis * vel * drag) / (v.mainBody.gravParameter * v.GetTotalMass() * 1000.0);
            natural_da_dt *= naturalDecayMultiplier;
            
            double predictedTime = EstimateDecayTime(v, v.altitude, 0);
            Debug.Log($"[OrbitalDecay] FULL PREDICTION from {v.altitude/1000:F1}km: {FormatTime(predictedTime)}");
        }

        void OnGUI()
        {
            if (showGui)
            {
                GUI.skin = HighLogic.Skin;
                Matrix4x4 oldMatrix = GUI.matrix;
                GUIUtility.ScaleAroundPivot(new Vector2(uiScale, uiScale), Vector2.zero);
                GUIStyle windowTitleStyle = new GUIStyle(GUI.skin.window) { fontSize = fontSize + 1 };
                windowRect = ClickThruBlocker.GUILayoutWindow(884422, windowRect, DrawWindow, Localizer.Format("#SWAOD_Title"), windowTitleStyle);
                if (showBodyFilterPopup && bodyFilterNames.Length > 0)
                {
                    GUIStyle popupTitleStyle = new GUIStyle(GUI.skin.window) { fontSize = fontSize + 1, fontStyle = FontStyle.Bold };
                    bodyFilterPopupRect = ClickThruBlocker.GUILayoutWindow(884423, bodyFilterPopupRect, DrawBodyFilterPopup, Localizer.Format("#SWAOD_SelectBody"), popupTitleStyle);
                }
                GUI.matrix = oldMatrix;
            }
        }

        private void DrawWindow(int windowID)
        {
            GUILayout.BeginVertical();

            if (_cachedFontSizeForStyles != fontSize)
            {
                _cachedFontSizeForStyles = fontSize;
                _cachedBold = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = fontSize + 1 };
                _cachedNormal = new GUIStyle(GUI.skin.label) { fontSize = fontSize };
                _cachedRed = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 0.4f, 0.4f) }, fontSize = fontSize, fontStyle = FontStyle.Bold };
                _cachedGreen = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(0.4f, 1f, 0.4f) }, fontSize = fontSize, fontStyle = FontStyle.Bold };
                _cachedYellow = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 1f, 0.4f) }, fontSize = fontSize };
                _cachedSubHeader = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = fontSize - 1, alignment = TextAnchor.MiddleLeft };
                _cachedVesselNameStyle = new GUIStyle(GUI.skin.label) { richText = true, fontSize = fontSize, fontStyle = FontStyle.Bold };
            }

            GUIStyle bold = _cachedBold;
            GUIStyle normal = _cachedNormal;
            GUIStyle red = _cachedRed;
            GUIStyle green = _cachedGreen;
            GUIStyle yellow = _cachedYellow;
            GUIStyle subHeader = _cachedSubHeader;
            GUIStyle vesselNameStyle = _cachedVesselNameStyle;
            GUIStyle destroyButtonStyle = GUI.skin.button;
            string destroyLabel = Localizer.Format("#SWAOD_DestroyNow");
            float destroyButtonWidth = destroyButtonStyle.CalcSize(new GUIContent(destroyLabel)).x + 12f;

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("#SWAOD_Config"), bold);
            GUILayout.FlexibleSpace();
            if (GUILayout.Button(showSettings ? Localizer.Format("#SWAOD_HideConfig") : Localizer.Format("#SWAOD_ShowConfig"), GUI.skin.button, GUILayout.Width(100)))
            {
                showSettings = !showSettings;
                if (!showSettings)
                {
                    windowRect.height = 0;
                }
            }
            GUILayout.EndHorizontal();

            if (showSettings)
            {
                GUILayout.BeginVertical("box");
                {
                    // UI Settings
                    GUILayout.Label(Localizer.Format("#SWAOD_UIScale", uiScale.ToString("F1")), subHeader);
                    
                    float oldScale = uiScale;
                    uiScale = GUILayout.HorizontalSlider(uiScale, 1.0f, 2.0f);
                    
                    if (Math.Abs(oldScale - uiScale) > 0.001f)
                    {
                        // Keep window position stable relative to screen center or top-left
                        windowRect.x *= (oldScale / uiScale);
                        windowRect.y *= (oldScale / uiScale);
                        
                        windowRect.width = GetWindowWidthForFontSize(fontSize);
                        windowRect.height = 0;
                    }
                    
                    GUILayout.Label(Localizer.Format("#SWAOD_FontSize", fontSize), subHeader);
                    int oldFontSize = fontSize;
                    fontSize = (int)GUILayout.HorizontalSlider((float)fontSize, 10f, 20f);
                    if (oldFontSize != fontSize)
                    {
                        ApplyWindowWidth(GetWindowWidthForFontSize(fontSize));
                        windowRect.height = 0;
                    }

                    GUILayout.Space(5);

                    // Key Binding UI
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(Localizer.Format("#SWAOD_Shortcut", toggleKey.ToString()), subHeader);
                    if (GUILayout.Button(isRebinding ? Localizer.Format("#SWAOD_PressKey") : Localizer.Format("#SWAOD_ChangeKey"), GUI.skin.button, GUILayout.Width(100)))
                    {
                        isRebinding = !isRebinding;
                    }
                    GUILayout.EndHorizontal();

                    if (isRebinding)
                    {
                        Event e = Event.current;
                        if (e.isKey && e.type == EventType.KeyDown && e.keyCode != KeyCode.None)
                        {
                             // Ignore modifier keys themselves
                             if (e.keyCode != KeyCode.LeftAlt && e.keyCode != KeyCode.RightAlt && e.keyCode != KeyCode.LeftControl && e.keyCode != KeyCode.RightControl)
                             {
                                 toggleKey = e.keyCode;
                                 isRebinding = false;
                                 e.Use();
                             }
                        }
                    }

                    GUILayout.Space(5);
                    if (GUILayout.Button(Localizer.Format("#SWAOD_Reset"), GUI.skin.button))
                    {
                        uiScale = 1.0f;
                        fontSize = 13;
                        toggleKey = KeyCode.Q;
                        windowRect.width = GetWindowWidthForFontSize(fontSize);
                        windowRect.height = 0;
                    }
                    
                    if (GUILayout.Button(Localizer.Format("#SWAOD_SaveUI"), GUI.skin.button))
                    {
                        SaveUISettings();
                    }
                    GUILayout.Space(5);
                    

                    bool newDebugMode = GUILayout.Toggle(debugMode, Localizer.Format("#SWAOD_DebugMode"));
                    if (newDebugMode != debugMode)
                    {
                        debugMode = newDebugMode;
                        if (!debugMode) windowRect.height = 0;
                    }

                    if (debugMode)
                    {
                        if (GUILayout.Button(Localizer.Format("#SWAOD_DumpLogs"), GUI.skin.button))
                        {
                             DumpAtmosphereLogs();
                        }
                        GUILayout.Space(5);
                    }
                    
                    // Storm Settings
                    GUILayout.BeginHorizontal();
#if KERBALISM
                    string stormStatus = stormDecayRate > 0 ? Localizer.Format("#SWAOD_Enabled") : Localizer.Format("#SWAOD_Disabled");
#else
                    string stormStatus = Localizer.Format("#SWAOD_NotAvailable");
#endif
                    GUILayout.Label(Localizer.Format("#SWAOD_StormDecay") + stormStatus, normal);
                    GUILayout.FlexibleSpace();
                    GUILayout.Label(Localizer.Format("#SWAOD_BaseRate", stormDecayRate.ToString("E2")), yellow);
                    GUILayout.EndHorizontal();
    
                    // Natural Decay Settings
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(Localizer.Format("#SWAOD_NaturalDecay") + (naturalDecayEnabled ? Localizer.Format("#SWAOD_Enabled") : Localizer.Format("#SWAOD_Disabled")), normal);
                    GUILayout.FlexibleSpace();
                    GUILayout.Label(Localizer.Format("#SWAOD_Multiplier", naturalDecayMultiplier.ToString("F2")), yellow);
                    GUILayout.EndHorizontal();
    
                    // Decay Limits (Based on current active vessel's body or Kerbin if none)
                    Vessel activeV = FlightGlobals.ActiveVessel;
                    if (activeV != null && activeV.mainBody.atmosphere)
                    {
                         double atmDepth = activeV.mainBody.atmosphereDepth;
                         double maxAlt = atmDepth * naturalDecayAltitudeCutoff;
                         
                         double nominalMax = atmDepth * 10.0;
                         GUILayout.Label(Localizer.Format("#SWAOD_NominalRange", (atmDepth/1000).ToString("F0"), (nominalMax/1000).ToString("F0")), subHeader);
                         
                         GUILayout.Label(Localizer.Format("#SWAOD_ActualRange", (atmDepth/1000).ToString("F0"), (maxAlt/1000).ToString("F0")), subHeader);
                    }
                    else
                    {
                         GUILayout.Label(Localizer.Format("#SWAOD_NoAtmosphere"), normal);
                    }
    
                    if (debugMode)
                    {
                        GUILayout.Space(5);
                        debugForceStorm = GUILayout.Toggle(debugForceStorm, Localizer.Format("#SWAOD_ForceStorm"));
                    }
                }
                GUILayout.EndVertical();
            }

            GUILayout.BeginHorizontal();
            if (bodyFilterNames.Length > 0)
            {
                string allBodiesText = Localizer.Format("#SWAOD_Filter_AllBodies");
                GUIStyle btnStyle = new GUIStyle(GUI.skin.button) { fontSize = fontSize, alignment = TextAnchor.MiddleCenter };
                float baselineWidth = btnStyle.CalcSize(new GUIContent(allBodiesText)).x;
                float buttonWidth = baselineWidth + BODY_BUTTON_PADDING;
                float labelH = normal.CalcHeight(new GUIContent(Localizer.Format("#SWAOD_Filter_Body")), 200f);
                float btnH = btnStyle.CalcSize(new GUIContent(allBodiesText)).y;
                float rowH = Math.Max(labelH, btnH);
                GUILayout.Label(Localizer.Format("#SWAOD_Filter_Body"), normal, GUILayout.Height(rowH));
                string displayText = bodyFilterNames[currentBodyFilterIndex];
                string truncatedDisplay = TruncateToWidth(displayText, btnStyle, baselineWidth);
                if (GUILayout.Button(truncatedDisplay, btnStyle, GUILayout.Width(buttonWidth), GUILayout.Height(rowH)))
                {
                    showBodyFilterPopup = true;
                    float logicalW = Screen.width / uiScale;
                    float logicalH = Screen.height / uiScale;
                    bodyFilterPopupRect = new Rect((logicalW - BODY_POPUP_DEFAULT_WIDTH) * 0.5f, (logicalH - BODY_POPUP_DEFAULT_HEIGHT) * 0.5f, BODY_POPUP_DEFAULT_WIDTH, BODY_POPUP_DEFAULT_HEIGHT);
                    bodyFilterPopupScrollPosition = Vector2.zero;
                }
            }
            else
            {
                GUILayout.Label(Localizer.Format("#SWAOD_Filter_Body"), normal);
            }
            GUILayout.EndHorizontal();
            GUILayout.Space(6);

            // --- Filter Buttons ---
            GUILayout.BeginHorizontal();
            FilterMode previousFilter = currentFilter;
            if (GUILayout.Toggle(currentFilter == FilterMode.All, Localizer.Format("#SWAOD_Filter_All"), GUI.skin.button)) currentFilter = FilterMode.All;
            if (GUILayout.Toggle(currentFilter == FilterMode.Stable, Localizer.Format("#SWAOD_Filter_Stable"), GUI.skin.button)) currentFilter = FilterMode.Stable;
            if (GUILayout.Toggle(currentFilter == FilterMode.Natural, Localizer.Format("#SWAOD_Filter_Natural"), GUI.skin.button)) currentFilter = FilterMode.Natural;
            if (GUILayout.Toggle(currentFilter == FilterMode.Storm, Localizer.Format("#SWAOD_Filter_Storm"), GUI.skin.button)) currentFilter = FilterMode.Storm;
            if (previousFilter != currentFilter) uiCacheDirty = true;
            GUILayout.EndHorizontal();
            
            GUILayout.BeginHorizontal();
            DebrisFilterMode previousDebrisFilter = currentDebrisFilter;
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.All, Localizer.Format("#SWAOD_Filter_Debris_All"), GUI.skin.button)) currentDebrisFilter = DebrisFilterMode.All;
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.OnlyDebris, Localizer.Format("#SWAOD_Filter_Debris_Only"), GUI.skin.button)) currentDebrisFilter = DebrisFilterMode.OnlyDebris;
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.ExcludeDebris, Localizer.Format("#SWAOD_Filter_Debris_Exclude"), GUI.skin.button)) currentDebrisFilter = DebrisFilterMode.ExcludeDebris;
            if (previousDebrisFilter != currentDebrisFilter) uiCacheDirty = true;
            GUILayout.EndHorizontal();

            // --- Vessel List ---
            GUILayout.Space(5);
            GUILayout.Label(Localizer.Format("#SWAOD_TrackedVessels", cachedVisibleVessels.Count), bold);

            GUIStyle scrollStyle = new GUIStyle(GUI.skin.scrollView);
            scrollStyle.padding.left = 0; 
            scrollStyle.padding.right = 0;
            
            float vesselListHeight = 600f;
            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, false, GUI.skin.horizontalScrollbar, GUI.skin.verticalScrollbar, scrollStyle, GUILayout.Height(vesselListHeight));

            foreach (VesselDisplayState state in cachedVisibleVessels)
            {
                Vessel v = state.Vessel;
                bool isNatural = state.IsNatural;
                bool stormInRange = state.StormInRange;

                GUILayout.BeginVertical("box");
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(state.VesselNameRich, vesselNameStyle);
                    GUILayout.FlexibleSpace();
                    GUILayout.Label(state.BodyName, yellow);
                    if (state.ShowDestroyButton)
                    {
                        GUILayout.Space(6);
                        if (GUILayout.Button(destroyLabel, destroyButtonStyle, GUILayout.Width(destroyButtonWidth)))
                        {
                            DestroyLoadedVessel(v);
                        }
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    GUILayout.BeginVertical();
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(state.PeAltText, normal, GUILayout.MinWidth(300f * (fontSize / (float)baseFontSize)));
                    GUILayout.Space(28f * (fontSize / (float)baseFontSize));
                    GUILayout.Label(state.PeTimeText, normal);
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(state.ApAltText, normal, GUILayout.MinWidth(300f * (fontSize / (float)baseFontSize)));
                    GUILayout.Space(28f * (fontSize / (float)baseFontSize));
                    GUILayout.Label(state.ApTimeText, normal);
                    GUILayout.EndHorizontal();
                    GUILayout.Label(state.IncText, normal);
                    GUILayout.Label(state.EccText, normal);
                    if (debugMode && stormInRange && !string.IsNullOrEmpty(state.StormRateText))
                    {
                        GUILayout.Label(state.StormRateText, red);
                    }
                    GUILayout.EndVertical();
                    GUILayout.FlexibleSpace();
                    GUILayout.EndHorizontal();

                    GUILayout.Space(4);

                    GUILayout.BeginHorizontal();
                    
                    string statusText = state.StatusText;
                    GUIStyle statusStyle = green;
                    
                    if (stormInRange && isNatural) { statusStyle = red; }
                    else if (stormInRange) { statusStyle = red; }
                    else if (isNatural) { statusStyle = yellow; }
                    
                    GUILayout.Label(statusText, statusStyle);
                    
                    GUILayout.EndHorizontal();
                }
                GUILayout.EndVertical();
            }

            GUILayout.EndScrollView();
            GUILayout.EndVertical();

            GUI.DragWindow();
        }

        private void RefreshUiCache(bool force)
        {
            float now = Time.realtimeSinceStartup;
            if (!force && !uiCacheDirty && uiLastRefreshTime >= 0f && now - uiLastRefreshTime < UI_CACHE_INTERVAL) return;
            uiLastRefreshTime = now;
            uiCacheDirty = false;
            UpdateBodyFilterCache();
            UpdateVesselCache();
            PopulateVisibleVessels();
        }

        private void UpdateBodyFilterCache()
        {
            int signature = 17;
            List<CelestialBody> bodies = FlightGlobals.Bodies;
            for (int i = 0; i < bodies.Count; i++)
            {
                CelestialBody body = bodies[i];
                signature = signature * 31 + body.name.GetHashCode();
            }

            if (signature == bodyFilterSignature && bodyFilterNames.Length > 0) return;

            bodyFilterSignature = signature;
            bodyFilterBodies.Clear();
            for (int i = 0; i < bodies.Count; i++)
            {
                bodyFilterBodies.Add(bodies[i]);
            }

            bodyFilterNames = new string[bodyFilterBodies.Count + 1];
            bodyFilterNames[0] = Localizer.Format("#SWAOD_Filter_AllBodies");
            for (int i = 0; i < bodyFilterBodies.Count; i++)
            {
                bodyFilterNames[i + 1] = bodyFilterBodies[i].name;
            }

            if (currentBodyFilterIndex >= bodyFilterNames.Length) currentBodyFilterIndex = 0;
        }

        private void UpdateVesselCache()
        {
            int signature = 17;
            Vessel activeVessel = FlightGlobals.ActiveVessel;
            Guid activeId = activeVessel != null ? activeVessel.id : Guid.Empty;
            signature = signature * 31 + activeId.GetHashCode();

            List<Vessel> vessels = FlightGlobals.Vessels;
            cachedVesselOrder.Clear();

            for (int i = 0; i < vessels.Count; i++)
            {
                Vessel v = vessels[i];
                if (!IsValidVessel(v)) continue;
                cachedVesselOrder.Add(v);
                signature = signature * 31 + v.id.GetHashCode();
                if (!string.IsNullOrEmpty(v.vesselName))
                    signature = signature * 31 + v.vesselName.GetHashCode();
            }

            int validCount = cachedVesselOrder.Count;

            cachedVesselOrder.Sort((a, b) =>
            {
                bool aIsActive = a.id == cachedActiveVesselId;
                bool bIsActive = b.id == cachedActiveVesselId;
                if (aIsActive && !bIsActive) return -1;
                if (!aIsActive && bIsActive) return 1;
                return string.Compare(a.vesselName, b.vesselName, StringComparison.OrdinalIgnoreCase);
            });

            cachedVesselSignature = signature;
            cachedValidVesselCount = validCount;
            cachedActiveVesselId = activeId;
        }

        private void PopulateVisibleVessels()
        {
            cachedVisibleVessels.Clear();
            for (int i = 0; i < cachedVesselOrder.Count; i++)
            {
                Vessel v = cachedVesselOrder[i];
                if (!TryBuildDisplayState(v, out VesselDisplayState state)) continue;
                cachedVisibleVessels.Add(state);
            }
        }

        private bool TryBuildDisplayState(Vessel v, out VesselDisplayState state)
        {
            state = new VesselDisplayState
            {
                Vessel = v,
                IsForced = debugForceStorm
            };

            if (currentBodyFilterIndex > 0)
            {
                int bodyIndex = currentBodyFilterIndex - 1;
                if (bodyIndex >= bodyFilterBodies.Count || v.mainBody != bodyFilterBodies[bodyIndex]) return false;
            }

            bool stormActive = false;
#if KERBALISM
            stormActive = StormInProgress(v);
#endif
            state.IsStorming = stormActive;
            state.IsNatural = false;
            state.StormInRange = false;
            state.EffectiveStormRate = 0;
            state.CurrentStormRate = 0;

            double periapsisAlt = v.orbit.PeA;
            if (naturalDecayEnabled && v.mainBody.atmosphere && periapsisAlt < v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff)
            {
                double dens = GetExosphericDensity(v.mainBody, periapsisAlt);
                if (periapsisAlt < v.mainBody.atmosphereDepth)
                {
                    dens *= 10.0;
                }
                if (dens > 1e-22)
                {
                    state.IsNatural = true;
                }
            }

            if (state.IsStorming || state.IsForced)
            {
                if (v.mainBody.atmosphere)
                {
                    double maxAlt = v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff;
                    state.StormInRange = periapsisAlt <= maxAlt;
                }
                else if (applyStormDecayToNoAtmosphereBody)
                {
                    double maxAlt = v.mainBody.sphereOfInfluence - v.mainBody.Radius;
                    state.StormInRange = periapsisAlt <= maxAlt;
                }
            }

            if (state.StormInRange)
            {
                double distanceFactor = 1.0;
                if (stormDistanceScaling)
                {
                    double dist = GetDistanceToSun(v);
                    dist = Math.Max(dist, 1000.0);
                    distanceFactor = Math.Pow(AU / dist, 2);
                }
                state.CurrentStormRate = stormDecayRate * distanceFactor;
                state.EffectiveStormRate = state.CurrentStormRate;
            }

            bool show = false;
            switch (currentFilter)
            {
                case FilterMode.All: show = true; break;
                case FilterMode.Stable: show = !state.IsNatural && !state.IsStorming && !state.IsForced; break;
                case FilterMode.Natural: show = state.IsNatural; break;
                case FilterMode.Storm: show = state.IsStorming || state.IsForced; break;
            }
            if (!show) return false;

            bool debrisMatch = true;
            switch (currentDebrisFilter)
            {
                case DebrisFilterMode.All: debrisMatch = true; break;
                case DebrisFilterMode.OnlyDebris: debrisMatch = v.vesselType == VesselType.Debris; break;
                case DebrisFilterMode.ExcludeDebris: debrisMatch = v.vesselType != VesselType.Debris; break;
            }
            if (!debrisMatch) return false;

            state.VesselNameRich = $"<b>{v.vesselName}</b>";
            state.BodyName = v.mainBody.name;
            state.PeAltText = Localizer.Format("#SWAOD_PeAlt", FormatAltitude(v.orbit.PeA));
            state.ApAltText = Localizer.Format("#SWAOD_ApAlt", FormatAltitude(v.orbit.ApA));
            state.PeTimeText = Localizer.Format("#SWAOD_DecayTime", GetDecayTimeDisplay(v, v.orbit.PeA, state.IsStorming, state.IsForced, state.EffectiveStormRate, cachedDecayTimesPe, lastDecayCalcTimePe));
            state.ApTimeText = Localizer.Format("#SWAOD_DecayTime", GetDecayTimeDisplay(v, v.orbit.ApA, state.IsStorming, state.IsForced, state.EffectiveStormRate, cachedDecayTimesAp, lastDecayCalcTimeAp));
            state.IncText = Localizer.Format("#SWAOD_Inc", v.orbit.inclination.ToString("F2"));
            state.EccText = Localizer.Format("#SWAOD_Ecc", v.orbit.eccentricity.ToString("F3"));

            string statusText = Localizer.Format("#SWAOD_Status_Stable");
            if (state.StormInRange && state.IsNatural) statusText = Localizer.Format("#SWAOD_Status_StormPlus");
            else if (state.StormInRange) statusText = Localizer.Format("#SWAOD_Status_StormDecay");
            else if (state.IsNatural) statusText = Localizer.Format("#SWAOD_Status_NaturalDecay");
            state.StatusText = statusText;
            state.StormRateText = state.StormInRange ? Localizer.Format("#SWAOD_StormRate_Debug", state.CurrentStormRate.ToString("E2")) : string.Empty;
            state.ShowDestroyButton = v.loaded && v.vesselType != VesselType.Debris && v.mainBody.atmosphere && v.orbit.PeA < v.mainBody.atmosphereDepth;

            return true;
        }

        private float GetWindowWidthForFontSize(int size)
        {
            return baseWindowWidth * (size / (float)baseFontSize);
        }

        private void ApplyWindowWidth(float newWidth)
        {
            float previousWidth = windowRect.width > 0f ? windowRect.width : baseWindowWidth;
            float right = windowRect.x + previousWidth;
            windowRect.width = newWidth;
            windowRect.x = right - newWidth;
        }

        private static string TruncateToWidth(string text, GUIStyle style, float maxWidth)
        {
            if (string.IsNullOrEmpty(text)) return text;
            Vector2 size = style.CalcSize(new GUIContent(text));
            if (size.x <= maxWidth) return text;
            for (int len = text.Length - 1; len >= 1; len--)
            {
                string truncated = text.Substring(0, len) + "..";
                if (style.CalcSize(new GUIContent(truncated)).x <= maxWidth) return truncated;
            }
            return "..";
        }

        private void DrawBodyFilterPopup(int windowID)
        {
            GUIStyle buttonStyle = new GUIStyle(GUI.skin.button) { fontSize = fontSize };
            float maxItemWidth = BODY_POPUP_MAX_WIDTH - 24f;
            float contentWidth = 0f;
            for (int i = 0; i < bodyFilterNames.Length; i++)
            {
                Vector2 sz = buttonStyle.CalcSize(new GUIContent(bodyFilterNames[i]));
                if (sz.x > contentWidth) contentWidth = Math.Min(sz.x, maxItemWidth);
            }
            float scrollWidth = Math.Max(contentWidth + 10f, BODY_POPUP_DEFAULT_WIDTH);
            scrollWidth = Math.Min(scrollWidth, BODY_POPUP_MAX_WIDTH);

            float btnH = buttonStyle.CalcSize(new GUIContent(Localizer.Format("#SWAOD_Close"))).y + 8f;
            const float TITLE_BAR_HEIGHT = 24f;
            float listHeight = bodyFilterPopupRect.height - TITLE_BAR_HEIGHT - 6f - 8f - btnH - 6f;
            listHeight = Math.Max(listHeight, BODY_POPUP_LIST_HEIGHT);

            GUILayout.Space(6);
            bodyFilterPopupScrollPosition = GUILayout.BeginScrollView(bodyFilterPopupScrollPosition, false, true, GUILayout.Width(scrollWidth), GUILayout.Height(listHeight));
            for (int i = 0; i < bodyFilterNames.Length; i++)
            {
                string name = bodyFilterNames[i];
                string displayName = TruncateToWidth(name, buttonStyle, maxItemWidth);
                if (GUILayout.Button(displayName, buttonStyle))
                {
                    currentBodyFilterIndex = i;
                    showBodyFilterPopup = false;
                    uiCacheDirty = true;
                }
            }
            GUILayout.EndScrollView();
            GUILayout.Space(8);
            if (GUILayout.Button(Localizer.Format("#SWAOD_Close"), buttonStyle))
            {
                showBodyFilterPopup = false;
            }
            GUILayout.Space(6);
            GUI.DragWindow();
        }
    }
}
