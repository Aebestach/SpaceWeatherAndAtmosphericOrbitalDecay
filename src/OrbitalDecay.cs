using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using KSP.UI.Screens;
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

        // Constants
        private const double AU = 13599840256; // 1 Astronomical Unit in meters (Kerbin SMA)

        // State Variables
        private HashSet<Guid> exosphereWarned = new HashSet<Guid>();
        private HashSet<Guid> lowOrbitWarned = new HashSet<Guid>();
        private Dictionary<Guid, double> pendingDestroyTimers = new Dictionary<Guid, double>(); // For unloaded vessels in atmo

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
        
        // UI Filter
        private enum FilterMode { All, Stable, Natural, Storm }
        private FilterMode currentFilter = FilterMode.All;

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
                windowRect = new Rect(Screen.width - 520, 100, 500, 0); // Increased size
                isWindowInitialized = true;
            }

            // Toggle UI with Mod+F11
            if (GameSettings.MODIFIER_KEY.GetKey() && Input.GetKeyDown(KeyCode.F11))
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

                // 1. Solar Storm Decay Logic
                bool stormActive = false;
#if KERBALISM
                stormActive = StormInProgress(v);
#endif
                stormActive = stormActive || debugForceStorm;

                if (stormActive)
                {
                    ApplyStormDecay(v, dt);
                }

                // 2. Natural Atmospheric Decay Logic
                if (naturalDecayEnabled)
                {
                    ApplyNaturalDecay(v, dt);
                }

                // 3. Low Orbit Warning Logic
                if (warningEnabled)
                {
                    CheckLowOrbitWarning(v);
                }
            }
        }

        private void CheckLowOrbitWarning(Vessel v)
        {
            if (!v.mainBody.atmosphere) return;

            double limitAlt = v.mainBody.atmosphereDepth * (1.0 + warningThreshold);
            double recoveryAlt = v.mainBody.atmosphereDepth * (1.0 + warningThreshold + 0.05); // 5% hysteresis buffer

            bool isLow = v.orbit.PeA < limitAlt;

            if (isLow)
            {
                if (!lowOrbitWarned.Contains(v.id))
                {
                    // Trigger Warning
                    string msg = Localizer.Format("#SWAOD_Warning_LowOrbit", v.vesselName, (v.orbit.PeA / 1000).ToString("F1"));
                    ScreenMessages.PostScreenMessage(msg, 5.0f, ScreenMessageStyle.UPPER_CENTER);
                    lowOrbitWarned.Add(v.id);
                }
            }
            else if (v.orbit.PeA > recoveryAlt)
            {
                // Reset Warning if recovered
                if (lowOrbitWarned.Contains(v.id))
                {
                    lowOrbitWarned.Remove(v.id);
                }
            }
        }

        private string FormatTime(double seconds)
        {
            if (seconds > 315360000) return Localizer.Format("#SWAOD_Time_GT100y"); // Cap at 10 years
            if (seconds > 31536000) return $"{seconds / 31536000:F1}y";
            if (seconds > 86400) return $"{seconds / 86400:F1}d";
            if (seconds > 3600) return $"{seconds / 3600:F1}h";
            if (seconds > 60) return $"{seconds / 60:F1}m";
            return $"{seconds:F0}s";
        }

        private bool IsValidVessel(Vessel v)
        {
            if (v == null || v.state == Vessel.State.DEAD) return false;

            // Filter out flags, space objects (asteroids/comets), and unknown types
            if (v.vesselType == VesselType.Flag ||
                v.vesselType == VesselType.SpaceObject ||
                v.vesselType == VesselType.Unknown) return false;

            // Only affect orbiting vessels (including sub-orbital if desired, but usually ORBITING)
            // SUB_ORBITAL might be in physics range, where stock drag applies.
            // We focus on ORBITING (on rails or high altitude).
            if (v.situation != Vessel.Situations.ORBITING) return false;

            return true;
        }

        private double GetDistanceToSun(Vessel v)
        {
            CelestialBody sun = FlightGlobals.Bodies[0];
            if (v.mainBody == sun) return v.orbit.radius;

            // Calculate distance in world coordinates
            // v.orbit.pos is relative to the main body
            // We use (BodyPos + RelPos) to get VesselWorldPos
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

            Orbit o = v.orbit;

            // Distance Scaling: Inverse Square Law relative to Kerbin (1 AU)
            double distanceFactor = 1.0;
            if (stormDistanceScaling)
            {
                double dist = GetDistanceToSun(v);
                
                // Avoid division by zero or extreme values near sun
                dist = Math.Max(dist, 1000.0);

                // Factor = (AU / dist)^2
                distanceFactor = Math.Pow(AU / dist, 2);
            }

            // Calculate decay factor
            // Rate is modified by distance
            double effectiveRate = stormDecayRate * distanceFactor;
            double decayFactor = Math.Exp(-effectiveRate * dt);

            // Apply Decay (Circularization)
            // Solar storms generally increase drag/resistance from plasma/atmosphere expansion
            // This acts like drag -> lowers SMA, lowers Eccentricity
            ModifyOrbit(o, decayFactor, decayFactor);
        }

        private void ApplyNaturalDecay(Vessel v, double dt)
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
                        ScreenMessages.PostScreenMessage(Localizer.Format("#SWAOD_Warning_EnteredAtm", v.vesselName), 5.0f, ScreenMessageStyle.UPPER_CENTER);
                        lowOrbitWarned.Add(v.id);
                    }
                }
                else
                {
                    // Unloaded: Handle destruction logic
                    if (!pendingDestroyTimers.ContainsKey(v.id))
                    {
                        pendingDestroyTimers.Add(v.id, 60.0); // 60 seconds grace period
                        MessageSystem.Instance.AddMessage(new MessageSystem.Message(
                           Localizer.Format("#SWAOD_Msg_ReEntry_Title"),
                           Localizer.Format("#SWAOD_Msg_ReEntry_Body", v.vesselName),
                           MessageSystemButton.MessageButtonColor.RED,
                           MessageSystemButton.ButtonIcons.ALERT
                        ));
                    }
                    else
                    {
                        pendingDestroyTimers[v.id] -= dt;
                        if (pendingDestroyTimers[v.id] <= 0)
                        {
                            MessageSystem.Instance.AddMessage(new MessageSystem.Message(
                               Localizer.Format("#SWAOD_Msg_Destroyed_Title"),
                               Localizer.Format("#SWAOD_Msg_Destroyed_Body", v.vesselName),
                               MessageSystemButton.MessageButtonColor.RED,
                               MessageSystemButton.ButtonIcons.FAIL
                            ));
                            v.Die();
                            pendingDestroyTimers.Remove(v.id);
                            return;
                        }
                    }
                    // Continue decay even while counting down
                }
            }
            else
            {
                // Reset timer if it bounces out
                if (pendingDestroyTimers.ContainsKey(v.id))
                {
                    pendingDestroyTimers.Remove(v.id);
                }
            }

            // Check altitude cutoff (Multiplier of AtmDepth)
            double maxAlt = atmDepth * naturalDecayAltitudeCutoff;
            if (altitude > maxAlt) return;

            // Calculate Exospheric Density
            // KSP returns 0 density outside atmosphere, so we extrapolate.
            double density = GetExosphericDensity(v.mainBody, altitude);

            // Ensure minimal effective density for decay if within cutoff range
            // This addresses the issue where density drops too fast at 120km
            // If we are within the cutoff, we assume there is SOME atmosphere.
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
            double mass = v.GetTotalMass(); // in tonnes
            if (mass <= 0.001) mass = 0.1; // Safety

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

            // Boost decay for loaded vessels inside atmosphere
            // Stock drag might be insufficient at very high altitudes (70km-80km),
            // so we add an extra "push" to ensure they de-orbit reasonably fast.
            // Extended range: Apply boost down to 2km below atmosphere top
            if (v.loaded && altitude < atmDepth && altitude > atmDepth - 2000.0)
            {
                da_dt *= 10.0; // Boost factor
            }

            // Calculate change for this time step
            double deltaSMA = da_dt * dt; // deltaSMA is negative (decay)

            // Warning Logic
            // Warn when entering the natural decay zone (exosphere)
            if (naturalDecayEnabled && !exosphereWarned.Contains(v.id))
            {
                // Check if inside decay zone
                double decayStartAlt = v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff;
                if (altitude < decayStartAlt)
                {
                    // Only warn if density is significant enough to cause decay
                    double _density = GetExosphericDensity(v.mainBody, altitude);
                    if (_density > 1e-20)
                    {
                        // Use KSP Message System (Top Right)
                        MessageSystem.Instance.AddMessage(new MessageSystem.Message(
                           Localizer.Format("#SWAOD_Warning_Decay_Title"),
                           Localizer.Format("#SWAOD_Warning_Decay_Body", v.vesselName, (altitude / 1000).ToString("F1")),
                           MessageSystemButton.MessageButtonColor.RED,
                           MessageSystemButton.ButtonIcons.ALERT
                        ));
                        exosphereWarned.Add(v.id);
                    }
                }
            }

            // If vessel goes back above the decay zone (e.g. burn), reset warning
            if (exosphereWarned.Contains(v.id))
            {
                double decayStartAlt = v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff;
                // Add a 5% buffer to prevent flickering
                if (altitude > decayStartAlt * 1.05)
                {
                    exosphereWarned.Remove(v.id);
                }
            }

            // Existing critical warning
            double criticalAlt = v.mainBody.atmosphereDepth * (1.0 + warningThreshold); // e.g. 1.2 * 70km = 84km
            if (warningEnabled && altitude < criticalAlt && !lowOrbitWarned.Contains(v.id))
            {
                // Re-warn for critical low altitude
                // Note: HashSet check prevents double warning, so we might need a separate set or flag for critical
                // For now, let's just use ScreenMessages unique check implicitly (spam prevention)
            }

            // Safety Check: If deltaSMA is excessively large (e.g. due to high time warp or math singularity), 
            // clamp it to a percentage of the remaining altitude buffer to prevent "teleporting" to the atmosphere.
            // This ensures smooth decay even at high warp.
            double currentBuffer = altitude - atmDepth;
            double maxDecayPerFrame = currentBuffer * 0.1; // Max 10% of remaining altitude per frame
            if (-deltaSMA > maxDecayPerFrame)
            {
                deltaSMA = -maxDecayPerFrame;
            }

            // If deltaSMA is extremely small (e.g. high orbit), skip to save precision issues
            if (Math.Abs(deltaSMA) < 1e-10) return;

            // Apply changes
            double newSMA = a + deltaSMA;

            // Hard Floor: Never go below atmosphere depth + 1 meter via this mod
            // KSP Stock physics will take over once inside atmosphere.
            // Removing the atmDepth clamp for loaded vessels to allow them to sink deeper where stock drag is stronger.
            double minSMA = v.mainBody.Radius + 100.0;

            if (newSMA < minSMA)
                newSMA = minSMA;

            double decayRatio = newSMA / a;
            ModifyOrbit(o, decayRatio, decayRatio);
        }

        private double GetExosphericDensity(CelestialBody body, double altitude)
        {
            double atmDepth = body.atmosphereDepth;
            double maxCutoffAlt = atmDepth * naturalDecayAltitudeCutoff;

            // 1. Absolute Cutoff
            if (altitude > maxCutoffAlt) return 0.0;

            // 2. Base Density at Atmosphere Edge (using 95% to 100% transition)
            // We sample at 95% to get a reliable "top of atmosphere" density
            double h_base = atmDepth * 0.95;
            double p_base = body.GetPressure(h_base);
            double t_base = body.GetTemperature(h_base);
            double rho_base = FlightGlobals.getAtmDensity(p_base, t_base, body);
            
            // Safety: If base density is effectively zero (thin atmo bodies), use a minimal fallback
            if (rho_base < 1e-15) rho_base = 1e-15;

            // 3. Density Curve Strategy
            // We use a FIXED reference scale to define the density curve shape.
            // This ensures that density at a specific altitude (e.g. 200km) remains constant
            // regardless of the user's maximum cutoff setting.
            //
            // The curve is calibrated such that density smoothly transitions from:
            // - Top of Atmosphere (AtmDepth * 0.95) -> Stock Density
            // - Reference Vacuum (AtmDepth * 10.0) -> Target Vacuum Density (1e-14)
            
            double curveRefScale = 10.0; 
            double curveMaxAlt = atmDepth * curveRefScale;
            double rho_target = 1e-14;   
            
            if (altitude <= h_base)
            {
                // If we are below the sampling point (but above actual surface/deep atmo),
                // just return the calculated density from stock, as it's likely accurate enough.
                 double p = body.GetPressure(altitude);
                 double t = body.GetTemperature(altitude);
                 return FlightGlobals.getAtmDensity(p, t, body);
            }
            
            // Calculate interpolation factor t based on the FIXED curve reference
            // t can be > 1.0 if altitude > curveMaxAlt (which is fine, it means density continues to drop)
            double t_factor = (altitude - h_base) / (curveMaxAlt - h_base);
            
            // Power Curve Adjustment (Power 0.5)
            // Flattens the curve: Lowers density at low altitudes (relative to linear), raises it at high altitudes.
            // This creates a distinct difference between "Low Orbit" (rapid decay) and "Medium Orbit" (slow decay).
            double t_curved = Math.Pow(t_factor, 0.5);
            
            // Log-Linear Interpolation
            // ln(rho) = ln(rho_base) * (1-t) + ln(rho_target) * t
            double logRho = Math.Log(rho_base) * (1.0 - t_curved) + Math.Log(rho_target) * t_curved;
            
            return Math.Exp(logRho);
        }

        private void ModifyOrbit(Orbit o, double smaFactor, double eccFactor)
        {
            // Capture current phase
            double currentMeanAnomaly = o.meanAnomaly;

            o.semiMajorAxis *= smaFactor;
            o.eccentricity *= eccFactor;

            // Prevent eccentricity from becoming negative
            if (o.eccentricity < 0) o.eccentricity = 0;

            // Reset epoch to maintain position
            o.epoch = Planetarium.GetUniversalTime();
            o.meanAnomalyAtEpoch = currentMeanAnomaly;

            // Recalculate
            o.Init();
            o.UpdateFromUT(Planetarium.GetUniversalTime());
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
            
            // Determine max altitude for dump (use configured cutoff)
            double maxAlt = atmDepth * naturalDecayAltitudeCutoff;
            
            // Sample points from near top of atmosphere to the cutoff altitude
            // Dynamic step to avoid log spam if cutoff is high
            double range = maxAlt - (atmDepth * 0.8);
            double step = range > 0 ? Math.Max(atmDepth * 0.05, range / 40.0) : atmDepth * 0.05;

            for (double alt = atmDepth * 0.8; alt <= maxAlt; alt += step)
            {
                double p = b.GetPressure(alt);
                double t = b.GetTemperature(alt);
                double rho = FlightGlobals.getAtmDensity(p, t, b);
                double calcRho = GetExosphericDensity(b, alt);
                
                // Estimate Decay Rate (assuming standard vessel: 1000kg, 2m^2 area, Cd=2.0, Multiplier=1.0)
                // This helps verification without waiting for actual decay
                // da/dt = - (2 * a^2 * v * Fd) / (mu * m)
                // Fd = 0.5 * rho * v^2 * Cd * A
                // da/dt = - (rho * sqrt(mu * a) * Cd * A) / m
                
                double R = b.Radius;
                double r = R + alt;
                double a = r; // Circular orbit
                double mu = b.gravParameter;
                double orbVel = Math.Sqrt(mu / r);
                double m = 1000.0;
                double A = 2.0;
                double Cd = 2.0;
                
                double Fd = 0.5 * calcRho * orbVel * orbVel * Cd * A;
                double da_dt = -(2.0 * a * a * orbVel * Fd) / (mu * m);
                da_dt *= naturalDecayMultiplier; // Apply current multiplier
                
                string timeStr = "N/A";
                if (Math.Abs(da_dt) > 1e-10)
                {
                    double timeSec = 10000.0 / Math.Abs(da_dt); // Time to decay 10km
                    
                    // Prevent overflow for extremely long durations
                    if (timeSec > 3153600000.0) // > 100 years
                    {
                         timeStr = Localizer.Format("#SWAOD_Time_GT100y");
                    }
                    else
                    {
                        TimeSpan ts = TimeSpan.FromSeconds(timeSec);
                        timeStr = timeSec > 86400 * 365 ? 
                            Localizer.Format("#SWAOD_Time_YearsDays", (int)(timeSec / (86400 * 365)), (int)((timeSec % (86400 * 365)) / 86400)) : 
                            Localizer.Format("#SWAOD_Time_DaysHours", ts.Days, ts.Hours);
                    }
                }

                Debug.Log($"[OrbitalDecay] Alt: {alt/1000:F1}km | P: {p:E2} | Rho: {calcRho:E2} | da/dt: {da_dt:E2} m/s | 10km Time: {timeStr}");
            }
        }

        void OnGUI()
        {
            if (showGui)
            {
                // Set skin BEFORE creating the window to affect the window frame itself
                GUI.skin = HighLogic.Skin;

                // Apply Scaling Globally to the Matrix
                Matrix4x4 oldMatrix = GUI.matrix;
                GUIUtility.ScaleAroundPivot(new Vector2(uiScale, uiScale), Vector2.zero);
                
                windowRect = GUILayout.Window(884422, windowRect, DrawWindow, Localizer.Format("#SWAOD_Title"));
                
                // Restore Matrix
                GUI.matrix = oldMatrix;
            }
        }

        private void DrawWindow(int windowID)
        {
            GUILayout.BeginVertical();

            // Styles - Adjusted for HighLogic.Skin and User Settings
            GUIStyle bold = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = fontSize + 1 };
            GUIStyle normal = new GUIStyle(GUI.skin.label) { fontSize = fontSize };
            GUIStyle red = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 0.4f, 0.4f) }, fontSize = fontSize, fontStyle = FontStyle.Bold };
            GUIStyle green = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(0.4f, 1f, 0.4f) }, fontSize = fontSize, fontStyle = FontStyle.Bold };
            GUIStyle yellow = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 1f, 0.4f) }, fontSize = fontSize };
            GUIStyle subHeader = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = fontSize - 1, alignment = TextAnchor.MiddleLeft };

            // --- Header & Settings Toggle ---
            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("#SWAOD_Config"), bold);
            GUILayout.FlexibleSpace();
            if (GUILayout.Button(showSettings ? Localizer.Format("#SWAOD_HideConfig") : Localizer.Format("#SWAOD_ShowConfig"), GUI.skin.button, GUILayout.Width(100)))
            {
                showSettings = !showSettings;
                if (!showSettings)
                {
                    windowRect.height = 0; // Force height recalculation to remove blank space
                }
            }
            GUILayout.EndHorizontal();

            // --- Settings Block ---
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
                    if (GUILayout.Button(Localizer.Format("#SWAOD_Reset"), GUI.skin.button))
                    {
                        uiScale = 1.0f;
                        fontSize = 13;
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
                        // Reset window height when debug mode is toggled off (hides extra buttons)
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
                         
                         // Show Nominal (Standard 10x)
                         double nominalMax = atmDepth * 10.0;
                         GUILayout.Label(Localizer.Format("#SWAOD_NominalRange", (atmDepth/1000).ToString("F0"), (nominalMax/1000).ToString("F0")), subHeader);
                         
                         // Show Actual (Configured)
                         GUILayout.Label(Localizer.Format("#SWAOD_ActualRange", (atmDepth/1000).ToString("F0"), (maxAlt/1000).ToString("F0")), subHeader);
                    }
                    else
                    {
                         GUILayout.Label(Localizer.Format("#SWAOD_NoAtmosphere"), normal);
                    }
    
                    // Debug Switch
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

            // --- Vessel List ---
            GUILayout.Space(5);
            GUILayout.Label(Localizer.Format("#SWAOD_TrackedVessels", FlightGlobals.Vessels.FindAll(v => IsValidVessel(v)).Count), bold);

            GUIStyle scrollStyle = new GUIStyle(GUI.skin.scrollView);
            scrollStyle.padding.left = 0; 
            scrollStyle.padding.right = 0;
            
            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, false, GUI.skin.horizontalScrollbar, GUI.skin.verticalScrollbar, scrollStyle, GUILayout.Height(400));

            foreach (Vessel v in FlightGlobals.Vessels)
            {
                if (!IsValidVessel(v)) continue;

                // Determine Status
                bool stormActive = false;
#if KERBALISM
                stormActive = StormInProgress(v);
#endif
                bool isStorming = stormActive;
                bool isForced = debugForceStorm;
                bool isNatural = false;

                // Check Natural Decay conditions
                double da_dt_display = 0; // For UI display

                // Calculate Natural Decay Rate
                // Include vessels inside atmosphere for display purposes
                if (naturalDecayEnabled && v.mainBody.atmosphere && v.altitude < v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff)
                {
                    double dens = GetExosphericDensity(v.mainBody, v.altitude);
                    
                    // Boost density calculation for display inside atmosphere to match boosted physics
                    if (v.altitude < v.mainBody.atmosphereDepth)
                    {
                         dens *= 10.0; // Visual match for the boost applied in ApplyNaturalDecay
                    }

                    if (dens > 1e-22) // Use lower threshold for UI too
                    {
                        isNatural = true;
                        double vSq = v.mainBody.gravParameter * (2.0 / (v.altitude + v.mainBody.Radius) - 1.0 / v.orbit.semiMajorAxis);
                        double vel = Math.Sqrt(Math.Max(0, vSq));
                        double area = Math.Pow(v.GetTotalMass(), 0.666) * 4.0;
                        double drag = 0.5 * dens * vSq * 2.0 * area;
                        double natural_da_dt = -(2.0 * v.orbit.semiMajorAxis * v.orbit.semiMajorAxis * vel * drag) / (v.mainBody.gravParameter * v.GetTotalMass() * 1000.0);
                        da_dt_display += natural_da_dt * naturalDecayMultiplier;
                    }
                }

                // Calculate Storm Decay Rate
                double effectiveRate = 0;
                if (isStorming || isForced)
                {
                    double distanceFactor = 1.0;
                    if (stormDistanceScaling)
                    {
                        double dist = GetDistanceToSun(v);
                        dist = Math.Max(dist, 1000.0);
                        distanceFactor = Math.Pow(AU / dist, 2);
                    }
                    effectiveRate = stormDecayRate * distanceFactor;
                    
                    double storm_da_dt = -v.orbit.semiMajorAxis * effectiveRate;
                    da_dt_display += storm_da_dt;
                }

                // Apply Filters
                bool show = false;
                switch (currentFilter)
                {
                    case FilterMode.All: show = true; break;
                    case FilterMode.Stable: show = !isNatural && !isStorming && !isForced; break;
                    case FilterMode.Natural: show = isNatural; break;
                    case FilterMode.Storm: show = isStorming || isForced; break;
                }
                if (!show) continue;

                GUILayout.BeginVertical("box"); // Vessel Block
                {
                    // Row 1: Name & Body
                    GUILayout.BeginHorizontal();
                    GUILayout.Label($"<b>{v.vesselName}</b>", new GUIStyle(GUI.skin.label) { richText = true, fontSize = fontSize, fontStyle = FontStyle.Bold });
                    GUILayout.FlexibleSpace();
                    GUILayout.Label($"{v.mainBody.name}", yellow);
                    GUILayout.EndHorizontal();

                    // Row 2: Orbital Parameters
                    GUILayout.BeginHorizontal();
                    GUILayout.BeginVertical();
                    GUILayout.Label($"Alt: {v.altitude / 1000:F3} km", normal);
                    GUILayout.Label($"Inc: {v.orbit.inclination:F2}°", normal);
                    GUILayout.Label($"Ecc: {v.orbit.eccentricity:F3}", normal);
                    GUILayout.EndVertical();
                    GUILayout.FlexibleSpace();
                    
                    // Show detailed rate only in Debug Mode
                    if (debugMode && (isStorming || isForced))
                    {
                        GUILayout.BeginVertical();
                        GUILayout.Label(Localizer.Format("#SWAOD_StormRate_Debug", effectiveRate.ToString("E2")), red);
                        GUILayout.EndVertical();
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.Space(4);

                    // Row 3: Status & Prediction
                    GUILayout.BeginHorizontal();
                    
                    // Status Text
                    string statusText = Localizer.Format("#SWAOD_Status_Stable");
                    GUIStyle statusStyle = green;
                    
                    if ((isStorming || isForced) && isNatural) { statusText = Localizer.Format("#SWAOD_Status_StormPlus"); statusStyle = red; }
                    else if (isStorming || isForced) { statusText = Localizer.Format("#SWAOD_Status_StormDecay"); statusStyle = red; }
                    else if (isNatural) { statusText = Localizer.Format("#SWAOD_Status_NaturalDecay"); statusStyle = yellow; }
                    
                    GUILayout.Label(statusText, statusStyle);
                    
                    GUILayout.FlexibleSpace();

                    // Prediction
                    // Show prediction if there is ANY decay (even small) or if vessel is already re-entering
                    if (da_dt_display < -1e-20)
                    {
                        // Use Periapsis altitude (PeA) for decay prediction
                        // because re-entry happens when PeA drops into the atmosphere
                        double peA = v.orbit.PeA;
                        double distToAtm = peA - v.mainBody.atmosphereDepth;
                        
                        // Show prediction even if inside atmosphere (negative distance)
                        if (distToAtm > 0)
                        {
                            double timeSeconds = distToAtm / Math.Abs(da_dt_display);
                            
                            string timeStr;
                            
                            // Check for extreme values to prevent TimeSpan overflow
                            if (timeSeconds > 3153600000.0) // > 100 years
                            {
                                timeStr = Localizer.Format("#SWAOD_Time_GT100y");
                            }
                            else
                            {
                                TimeSpan ts = TimeSpan.FromSeconds(timeSeconds);
                                
                                if (timeSeconds > 86400 * 365)
                                {
                                    int years = (int)(timeSeconds / (86400 * 365));
                                    int days = (int)((timeSeconds % (86400 * 365)) / 86400);
                                    timeStr = Localizer.Format("#SWAOD_Time_YearsDays", years, days);
                                }
                                else if (timeSeconds > 86400)
                                {
                                    timeStr = Localizer.Format("#SWAOD_Time_DaysHours", ts.Days, ts.Hours);
                                }
                                else if (timeSeconds > 3600)
                                {
                                    timeStr = Localizer.Format("#SWAOD_Time_HoursMins", ts.Hours, ts.Minutes);
                                }
                                else
                                {
                                    timeStr = Localizer.Format("#SWAOD_Time_MinsSecs", ts.Minutes, ts.Seconds);
                                }
                            }
                            
                            GUILayout.Label(Localizer.Format("#SWAOD_EntryTime", timeStr), yellow);
                        }
                        else
                        {
                            GUILayout.Label(Localizer.Format("#SWAOD_ReEntry"), red);
                        }
                    }
                    else
                    {
                        GUILayout.Label("--", normal);
                    }
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
