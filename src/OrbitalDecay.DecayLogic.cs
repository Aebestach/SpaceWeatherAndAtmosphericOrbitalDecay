using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.Localization;

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
            if (o.PeA > maxAlt) return;

            double mass = v.GetTotalMass();
            if (mass <= 0.001) mass = 0.1;

            AtmosphericDecayModel.DecaySettings decaySettings = GetDecaySettings();
            double originalSma = o.semiMajorAxis;
            double currentSma = originalSma;
            double currentEcc = o.eccentricity;
            double remainingDt = dt;
            int integrationSteps = 0;

            while (remainingDt > 0.0 && integrationSteps < 256)
            {
                double stepDt = remainingDt;

                if (stepDt > 3600.0) stepDt = 3600.0;

                double currentDaDt = AtmosphericDecayModel.EstimateNaturalDaDt(
                    v.mainBody, currentSma, currentEcc, mass, decaySettings);
                if (Math.Abs(currentDaDt) <= 1e-12)
                    break;

                double stepDeltaSma = currentDaDt * stepDt;
                double periapsisBuffer = currentSma * (1.0 - currentEcc) - v.mainBody.Radius - atmDepth;
                if (periapsisBuffer > 0.0)
                {
                    double maxSafeDrop = Math.Max(periapsisBuffer * 0.5, 10000.0);
                    if (-stepDeltaSma > maxSafeDrop)
                        stepDeltaSma = -maxSafeDrop;
                }

                AtmosphericDecayModel.OrbitElements next =
                    AtmosphericDecayModel.ApplyDecayToElements(v.mainBody, currentSma, currentEcc, stepDeltaSma);
                if (Math.Abs(next.SemiMajorAxis - currentSma) < 1e-8)
                    break;

                currentSma = next.SemiMajorAxis;
                currentEcc = next.Eccentricity;
                remainingDt -= stepDt;
                integrationSteps++;
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

            double deltaSMA = currentSma - originalSma;
            if (Math.Abs(deltaSMA) < 1e-10) return;

            if (v.loaded && !v.packed)
            {
                double decayRatio = currentSma / originalSma;
                ApplyDecayToVessel(v, o, deltaSMA, decayRatio, currentUT);
            }
            else
            {
                ModifyOrbitToElements(o, currentSma, currentEcc, currentUT);
            }
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
            return AtmosphericDecayModel.GetDensity(body, altitude, GetDecaySettings());
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

        private void ModifyOrbitToElements(Orbit o, double newA, double newE, double currentUT)
        {
            double currentMeanAnomaly = o.meanAnomaly;

            if (newE < 0.0) newE = 0.0;
            if (newE >= 1.0) newE = 0.999999;

            o.semiMajorAxis = newA;
            o.eccentricity = newE;

            o.epoch = currentUT;
            o.meanAnomalyAtEpoch = currentMeanAnomaly;

            o.Init();
            o.UpdateFromUT(currentUT);
        }

        private double EstimateDecayTime(Vessel v, bool useApoapsis, double effectiveStormRate)
        {
            if (v == null || v.orbit == null || v.mainBody == null)
                return double.PositiveInfinity;

            double targetAltitude = useApoapsis ? v.orbit.ApA : v.orbit.PeA;
            if (targetAltitude <= v.mainBody.atmosphereDepth)
                return 0.0;

            double mass = v.GetTotalMass();
            if (mass <= 0.001) mass = 0.1;

            return AtmosphericDecayModel.EstimateTimeToAtmosphere(
                v.mainBody,
                v.orbit.semiMajorAxis,
                v.orbit.eccentricity,
                mass,
                effectiveStormRate,
                useApoapsis,
                GetDecaySettings());
        }

        private double GetCachedDecayTime(Vessel v, bool useApoapsis, double effectiveStormRate, Dictionary<Guid, double> cache, Dictionary<Guid, float> timeCache)
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

            double newTime = EstimateDecayTime(v, useApoapsis, effectiveStormRate);

            cache[v.id] = newTime;
            timeCache[v.id] = currentTime;

            return newTime;
        }

        private string GetDecayTimeDisplay(Vessel v, bool useApoapsis, bool isStorming, bool isForced, double effectiveStormRate, Dictionary<Guid, double> cache, Dictionary<Guid, float> timeCache)
        {
            if (!v.mainBody.atmosphere) return Localizer.Format("#SWAOD_NotAvailable");

            double atmDepth = v.mainBody.atmosphereDepth;
            double altitude = useApoapsis ? v.orbit.ApA : v.orbit.PeA;
            if (altitude <= atmDepth) return Localizer.Format("#SWAOD_ReEntry");

            double maxDecayAlt = atmDepth * naturalDecayAltitudeCutoff;

            bool canDecay = isStorming || isForced;
            if (!canDecay && naturalDecayEnabled)
            {
                if (v.orbit.PeA <= maxDecayAlt) canDecay = true;
            }

            if (!canDecay) return Localizer.Format("#SWAOD_NotAvailable");

            double timeSeconds = GetCachedDecayTime(v, useApoapsis, effectiveStormRate, cache, timeCache);
            return FormatTime(timeSeconds);
        }
    }
}
