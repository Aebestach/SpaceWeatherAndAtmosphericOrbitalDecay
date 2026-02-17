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
        private double reentryDestroySeconds = 180.0;

        // Constants: 1 Astronomical Unit in meters (Kerbin SMA)
        private const double AU = 13599840256; 

        // State Variables
        private HashSet<Guid> lowOrbitWarned = new HashSet<Guid>();
        private HashSet<Guid> lowPeriapsisWarned = new HashSet<Guid>();
        private HashSet<Guid> atmosphereTopWarned = new HashSet<Guid>();
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
        private bool showSettings = false;
        private KeyCode toggleKey = KeyCode.Q;
        private bool isRebinding = false;
        
        // UI Filter
        private enum FilterMode { All, Stable, Natural, Storm }
        private FilterMode currentFilter = FilterMode.All;
        private enum DebrisFilterMode { All, OnlyDebris, ExcludeDebris }
        private DebrisFilterMode currentDebrisFilter = DebrisFilterMode.All;

        // Caching for Performance
        private Dictionary<Guid, double> cachedDecayTimesPe = new Dictionary<Guid, double>();
        private Dictionary<Guid, float> lastDecayCalcTimePe = new Dictionary<Guid, float>();
        private Dictionary<Guid, double> cachedDecayTimesAp = new Dictionary<Guid, double>();
        private Dictionary<Guid, float> lastDecayCalcTimeAp = new Dictionary<Guid, float>();
        private const float CACHE_INTERVAL = 1.0f;

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
                if (reentryDestroySeconds <= 0) reentryDestroySeconds = 180.0;

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
                        windowRect = new Rect(x, y, 500, 0);
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
                windowRect = new Rect(Screen.width - 520, 100, 500, 0);
                isWindowInitialized = true;
            }

            // Toggle UI with Alt + UserKey
            if ((Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt)) && Input.GetKeyDown(toggleKey))
            {
                showGui = !showGui;
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
                    ApplyStormDecay(v, dt);
                }

                // Natural Atmospheric Decay Logic
                if (naturalDecayEnabled)
                {
                    ApplyNaturalDecay(v, dt);
                }
                
                CheckActiveVesselAtmosphereTopWarning(v);

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

        private bool IsValidVessel(Vessel v)
        {
            if (v == null || v.state == Vessel.State.DEAD) return false;

            // Filter out flags, space objects (asteroids/comets), and unknown types
            if (v.vesselType == VesselType.Flag ||
                v.vesselType == VesselType.SpaceObject ||
                v.vesselType == VesselType.Unknown) return false;

            if (v.situation != Vessel.Situations.ORBITING && v.situation != Vessel.Situations.SUB_ORBITAL) return false;

            return true;
        }

        private bool IsCircularOrbit(Vessel v)
        {
            return v.orbit != null && v.orbit.eccentricity <= 0.02;
        }

        private void CheckActiveVesselAtmosphereTopWarning(Vessel v)
        {
            if (v == null || v != FlightGlobals.ActiveVessel) return;
            if (!v.mainBody.atmosphere) return;
            if (v.vesselType == VesselType.Debris) return;
            if (v.situation != Vessel.Situations.ORBITING && v.situation != Vessel.Situations.SUB_ORBITAL) return;

            double atmDepth = v.mainBody.atmosphereDepth;
            double peA = v.orbit.PeA;
            double apA = v.orbit.ApA;
            double topBuffer = Math.Max(2000.0, atmDepth * 0.02);

            bool atTop = v.altitude >= atmDepth && peA <= atmDepth && apA <= atmDepth + topBuffer && IsCircularOrbit(v);

            if (atTop)
            {
                if (!atmosphereTopWarned.Contains(v.id))
                {
                    ScreenMessages.PostScreenMessage(Localizer.Format("#SWAOD_Msg_AutoDestroy_AtmTop", v.vesselName, FormatTime(reentryDestroySeconds)), 6.0f, ScreenMessageStyle.UPPER_CENTER);
                    atmosphereTopWarned.Add(v.id);
                }
            }
            else
            {
                if (atmosphereTopWarned.Contains(v.id))
                {
                    atmosphereTopWarned.Remove(v.id);
                }
            }
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
        private void ApplyStormDecay(Vessel v, double dt)
        {
            // Check if we should apply decay to bodies without atmosphere
            if (!applyStormDecayToNoAtmosphereBody && !v.mainBody.atmosphere)
            {
                return;
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
            ApplyDecayToVessel(v, o, deltaSMA, decayFactor);
        }

        private void ApplyNaturalDecay(Vessel v, double dt)
        {
            // Only applies if the body has an atmosphere
            if (!v.mainBody.atmosphere) return;

            Orbit o = v.orbit;
            double atmDepth = v.mainBody.atmosphereDepth;
            double altitude = v.altitude;
            double currentUT = Planetarium.GetUniversalTime();

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
                }
                
                if (!pendingDestroyTimers.ContainsKey(v.id))
                {
                    pendingDestroyTimers.Add(v.id, reentryDestroySeconds);
                    pendingDestroyNextMessageTimes[v.id] = currentUT;
                }
                
                pendingDestroyTimers[v.id] -= dt;

                if (pendingDestroyNextMessageTimes.TryGetValue(v.id, out double nextMsgTime) && currentUT >= nextMsgTime)
                {
                    if (v.vesselType != VesselType.Debris)
                    {
                        string msg = Localizer.Format("#SWAOD_Msg_ReEntry_Body", v.vesselName, FormatTime(reentryDestroySeconds)) + "  T-" + FormatTime(Math.Max(0.0, pendingDestroyTimers[v.id])) + "/" + FormatTime(reentryDestroySeconds);
                        ScreenMessages.PostScreenMessage(msg, 1.0f, ScreenMessageStyle.UPPER_CENTER);
                    }
                    pendingDestroyNextMessageTimes[v.id] = currentUT + 1.0;
                }
                
                if (pendingDestroyTimers[v.id] <= 0)
                {
                    if (v.vesselType != VesselType.Debris)
                    {
                        ScreenMessages.PostScreenMessage(Localizer.Format("#SWAOD_Msg_Destroyed_Body", v.vesselName), 10.0f, ScreenMessageStyle.UPPER_CENTER);
                    }
                    v.Die();
                    pendingDestroyTimers.Remove(v.id);
                    pendingDestroyNextMessageTimes.Remove(v.id);
                    return;
                }
            }
            else
            {
                if (pendingDestroyTimers.ContainsKey(v.id))
                {
                    pendingDestroyTimers.Remove(v.id);
                    pendingDestroyNextMessageTimes.Remove(v.id);
                }
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
            ApplyDecayToVessel(v, o, deltaSMA, decayRatio);
        }

        private void ApplyDecayToVessel(Vessel v, Orbit o, double deltaSMA, double eccFactor)
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
            ModifyOrbit(o, smaFactor, eccFactor);
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

        private void ModifyOrbit(Orbit o, double smaFactor, double eccFactor)
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

            o.epoch = Planetarium.GetUniversalTime();
            o.meanAnomalyAtEpoch = currentMeanAnomaly;

            o.Init();
            o.UpdateFromUT(Planetarium.GetUniversalTime());
        }

        private double EstimateDecayTime(Vessel v, double startAlt, double effectiveStormRate)
        {
            // Reentry time calculated based on Periapsis, ignoring Apoapsis.
            double atmDepth = v.mainBody.atmosphereDepth;
            double targetAlt = atmDepth;
            
            if (startAlt <= atmDepth) return 0;
            
            double simAlt = startAlt;
            double totalTime = 0;
            double stepAlt = (startAlt - atmDepth) / 10.0;
            
            // Limit max iterations for performance
            int maxSteps = 100; 
            stepAlt = (startAlt - atmDepth) / (double)maxSteps;
            
            double mu = v.mainBody.gravParameter;
            double R = v.mainBody.Radius;
            double m_kg = v.GetTotalMass() * 1000.0;
            if (m_kg < 1.0) m_kg = 100.0; // Safety
            double area = Math.Pow(v.GetTotalMass(), 0.666) * 4.0;
            double Cd = 2.0;

            for (int i = 0; i < maxSteps; i++)
            {
                double midAlt = simAlt - (stepAlt * 0.5);
                double simR = R + midAlt;
                double simSMA = simR;
                double simVel = Math.Sqrt(mu / simR);
                
                double simRho = GetExosphericDensity(v.mainBody, midAlt);

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

            if (cache.ContainsKey(v.id)) cache[v.id] = newTime;
            else cache.Add(v.id, newTime);

            if (timeCache.ContainsKey(v.id)) timeCache[v.id] = currentTime;
            else timeCache.Add(v.id, currentTime);

            return newTime;
        }

        private string GetDecayTimeDisplay(Vessel v, double altitude, bool isStorming, bool isForced, double effectiveStormRate, Dictionary<Guid, double> cache, Dictionary<Guid, float> timeCache)
        {
            if (!v.mainBody.atmosphere) return Localizer.Format("#SWAOD_NotAvailable");

            double atmDepth = v.mainBody.atmosphereDepth;
            if (altitude <= atmDepth) return Localizer.Format("#SWAOD_ReEntry");

            bool canDecay = isStorming || isForced;
            if (!canDecay && naturalDecayEnabled)
            {
                double maxDecayAlt = atmDepth * naturalDecayAltitudeCutoff;
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
                windowRect = GUILayout.Window(884422, windowRect, DrawWindow, Localizer.Format("#SWAOD_Title"));  
                GUI.matrix = oldMatrix;
            }
        }

        private void DrawWindow(int windowID)
        {
            GUILayout.BeginVertical();

            GUIStyle bold = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = fontSize + 1 };
            GUIStyle normal = new GUIStyle(GUI.skin.label) { fontSize = fontSize };
            GUIStyle red = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 0.4f, 0.4f) }, fontSize = fontSize, fontStyle = FontStyle.Bold };
            GUIStyle green = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(0.4f, 1f, 0.4f) }, fontSize = fontSize, fontStyle = FontStyle.Bold };
            GUIStyle yellow = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 1f, 0.4f) }, fontSize = fontSize };
            GUIStyle subHeader = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = fontSize - 1, alignment = TextAnchor.MiddleLeft };

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
                        
                        windowRect.width = 500;
                        windowRect.height = 0;
                    }
                    
                    GUILayout.Label(Localizer.Format("#SWAOD_FontSize", fontSize), subHeader);
                    fontSize = (int)GUILayout.HorizontalSlider((float)fontSize, 10f, 20f);

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
                        windowRect.width = 500;
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

            // --- Filter Buttons ---
            GUILayout.BeginHorizontal();
            if (GUILayout.Toggle(currentFilter == FilterMode.All, Localizer.Format("#SWAOD_Filter_All"), GUI.skin.button)) currentFilter = FilterMode.All;
            if (GUILayout.Toggle(currentFilter == FilterMode.Stable, Localizer.Format("#SWAOD_Filter_Stable"), GUI.skin.button)) currentFilter = FilterMode.Stable;
            if (GUILayout.Toggle(currentFilter == FilterMode.Natural, Localizer.Format("#SWAOD_Filter_Natural"), GUI.skin.button)) currentFilter = FilterMode.Natural;
            if (GUILayout.Toggle(currentFilter == FilterMode.Storm, Localizer.Format("#SWAOD_Filter_Storm"), GUI.skin.button)) currentFilter = FilterMode.Storm;
            GUILayout.EndHorizontal();
            
            GUILayout.BeginHorizontal();
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.All, Localizer.Format("#SWAOD_Filter_Debris_All"), GUI.skin.button)) currentDebrisFilter = DebrisFilterMode.All;
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.OnlyDebris, Localizer.Format("#SWAOD_Filter_Debris_Only"), GUI.skin.button)) currentDebrisFilter = DebrisFilterMode.OnlyDebris;
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.ExcludeDebris, Localizer.Format("#SWAOD_Filter_Debris_Exclude"), GUI.skin.button)) currentDebrisFilter = DebrisFilterMode.ExcludeDebris;
            GUILayout.EndHorizontal();

            // --- Vessel List ---
            GUILayout.Space(5);
            GUILayout.Label(Localizer.Format("#SWAOD_TrackedVessels", FlightGlobals.Vessels.FindAll(v => IsValidVessel(v)).Count), bold);

            GUIStyle scrollStyle = new GUIStyle(GUI.skin.scrollView);
            scrollStyle.padding.left = 0; 
            scrollStyle.padding.right = 0;
            
            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, false, GUI.skin.horizontalScrollbar, GUI.skin.verticalScrollbar, scrollStyle, GUILayout.Height(400));

            List<Vessel> vesselOrder = new List<Vessel>(FlightGlobals.Vessels.Count);
            foreach (Vessel v in FlightGlobals.Vessels)
            {
                if (!IsValidVessel(v)) continue;
                vesselOrder.Add(v);
            }
            vesselOrder.Sort((a, b) => string.Compare(a.vesselName, b.vesselName, StringComparison.OrdinalIgnoreCase));

            foreach (Vessel v in vesselOrder)
            {
                bool stormActive = false;
#if KERBALISM
                stormActive = StormInProgress(v);
#endif
                bool isStorming = stormActive;
                bool isForced = debugForceStorm;
                bool isNatural = false;

                if (naturalDecayEnabled && v.mainBody.atmosphere && v.altitude < v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff)
                {
                    double dens = GetExosphericDensity(v.mainBody, v.altitude);
                    
                    if (v.altitude < v.mainBody.atmosphereDepth)
                    {
                         dens *= 10.0;
                    }

                    if (dens > 1e-22)
                    {
                        isNatural = true;
                    }
                }

                // Calculate Storm Decay Rate
                double effectiveStormRate = 0;
                double currentStormRate = 0;
                if (isStorming || isForced)
                {
                    double distanceFactor = 1.0;
                    if (stormDistanceScaling)
                    {
                        double dist = GetDistanceToSun(v);
                        dist = Math.Max(dist, 1000.0);
                        distanceFactor = Math.Pow(AU / dist, 2);
                    }
                    currentStormRate = stormDecayRate * distanceFactor;
                    
                    effectiveStormRate = currentStormRate;
                }

                bool show = false;
                switch (currentFilter)
                {
                    case FilterMode.All: show = true; break;
                    case FilterMode.Stable: show = !isNatural && !isStorming && !isForced; break;
                    case FilterMode.Natural: show = isNatural; break;
                    case FilterMode.Storm: show = isStorming || isForced; break;
                }
                if (!show) continue;
                
                bool debrisMatch = true;
                switch (currentDebrisFilter)
                {
                    case DebrisFilterMode.All: debrisMatch = true; break;
                    case DebrisFilterMode.OnlyDebris: debrisMatch = v.vesselType == VesselType.Debris; break;
                    case DebrisFilterMode.ExcludeDebris: debrisMatch = v.vesselType != VesselType.Debris; break;
                }
                if (!debrisMatch) continue;

                GUILayout.BeginVertical("box");
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label($"<b>{v.vesselName}</b>", new GUIStyle(GUI.skin.label) { richText = true, fontSize = fontSize, fontStyle = FontStyle.Bold });
                    GUILayout.FlexibleSpace();
                    GUILayout.Label($"{v.mainBody.name}", yellow);
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    GUILayout.BeginVertical();
                    string peAltText = Localizer.Format("#SWAOD_PeAlt", FormatAltitude(v.orbit.PeA));
                    string apAltText = Localizer.Format("#SWAOD_ApAlt", FormatAltitude(v.orbit.ApA));
                    string peTimeText = Localizer.Format("#SWAOD_DecayTime", GetDecayTimeDisplay(v, v.orbit.PeA, isStorming, isForced, effectiveStormRate, cachedDecayTimesPe, lastDecayCalcTimePe));
                    string apTimeText = Localizer.Format("#SWAOD_DecayTime", GetDecayTimeDisplay(v, v.orbit.ApA, isStorming, isForced, effectiveStormRate, cachedDecayTimesAp, lastDecayCalcTimeAp));
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(peAltText, normal, GUILayout.MinWidth(240));
                    GUILayout.Space(28);
                    GUILayout.Label(peTimeText, normal);
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(apAltText, normal, GUILayout.MinWidth(240));
                    GUILayout.Space(28);
                    GUILayout.Label(apTimeText, normal);
                    GUILayout.EndHorizontal();
                    GUILayout.Label($"Inc: {v.orbit.inclination:F2}°", normal);
                    GUILayout.Label($"Ecc: {v.orbit.eccentricity:F3}", normal);
                    GUILayout.EndVertical();
                    GUILayout.FlexibleSpace();
                    
                    if (debugMode && (isStorming || isForced))
                    {
                        GUILayout.BeginVertical();
                        GUILayout.Label(Localizer.Format("#SWAOD_StormRate_Debug", currentStormRate.ToString("E2")), red);
                        GUILayout.EndVertical();
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space(4);

                    GUILayout.BeginHorizontal();
                    
                    string statusText = Localizer.Format("#SWAOD_Status_Stable");
                    GUIStyle statusStyle = green;
                    
                    if ((isStorming || isForced) && isNatural) { statusText = Localizer.Format("#SWAOD_Status_StormPlus"); statusStyle = red; }
                    else if (isStorming || isForced) { statusText = Localizer.Format("#SWAOD_Status_StormDecay"); statusStyle = red; }
                    else if (isNatural) { statusText = Localizer.Format("#SWAOD_Status_NaturalDecay"); statusStyle = yellow; }
                    
                    GUILayout.Label(statusText, statusStyle);
                    
                    GUILayout.EndHorizontal();
                }
                GUILayout.EndVertical();
            }

            GUILayout.EndScrollView();
            GUILayout.EndVertical();

            GUI.DragWindow();
        }
    }
}
