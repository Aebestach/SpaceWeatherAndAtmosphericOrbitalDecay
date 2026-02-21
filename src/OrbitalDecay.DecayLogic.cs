using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.Localization;

#if KERBALISM
using static KERBALISM.API;
#endif

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    /// <summary>
    /// Decay physics and orbit modification logic (partial of OrbitalDecay).
    /// </summary>
    public partial class OrbitalDecay
    {
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
    }
}
