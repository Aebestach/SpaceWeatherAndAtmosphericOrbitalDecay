using System;
using UnityEngine;

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    /// <summary>
    /// Public read-only API for other mods that want SWAOD's decay estimates
    /// without depending on OrbitalDecay's MonoBehaviour internals.
    /// </summary>
    public static class OrbitalDecayApi
    {
        public struct StationKeepingEstimate
        {
            public bool Available;
            public bool IsStormEstimate;
            public double SecondsToTolerance;
            public double DecayRate;
            public double ToleranceDrop;
            public double DeltaVToRestoreToleranceDrop;
            public string Reason;
        }

        private struct ApiSettings
        {
            public bool Loaded;
            public double StormDecayRate;
            public bool StormDistanceScaling;
            public bool ApplyStormDecayToNoAtmosphereBody;
            public bool NaturalDecayEnabled;
            public double NaturalDecayMultiplier;
            public double NaturalDecayAltitudeCutoff;
            public double ExosphereFitStart;
            public double ExosphereFitEnd;
            public double ExosphereScaleHeightMin;
            public double ExosphereScaleHeightMax;
            public int ExosphereFitSamples;
            public int OrbitAverageSamples;
        }

        private const double AU = 13599840256.0;
        private static bool settingsLoaded;
        private static ApiSettings settings;

        /// <summary>
        /// Estimates how long it should take for SWAOD decay to push a vessel outside
        /// an OrbitalKeeper-style apoapsis/periapsis tolerance band.
        /// </summary>
        public static bool TryEstimateStationKeepingCadence(
            Vessel vessel,
            double targetApoapsis,
            double targetPeriapsis,
            double tolerancePercent,
            out StationKeepingEstimate estimate)
        {
            estimate = new StationKeepingEstimate
            {
                Available = false,
                Reason = "Unavailable"
            };

            if (vessel == null || vessel.orbit == null || vessel.mainBody == null)
            {
                estimate.Reason = "No vessel";
                return false;
            }

            if (vessel.situation != Vessel.Situations.ORBITING || vessel.orbit.eccentricity >= 1.0)
            {
                estimate.Reason = "Invalid orbit";
                return false;
            }

            double toleranceDrop = EstimateToleranceDrop(
                vessel, targetApoapsis, targetPeriapsis, tolerancePercent);
            if (toleranceDrop <= 0.0)
            {
                estimate.Reason = "No tolerance margin";
                return false;
            }

            double daDt = EstimateDaDt(vessel, out bool stormEstimate);
            double decayRate = Math.Abs(daDt);
            if (decayRate <= 1e-12)
            {
                estimate.Reason = "No active decay";
                return false;
            }

            estimate.Available = true;
            estimate.IsStormEstimate = stormEstimate;
            estimate.ToleranceDrop = toleranceDrop;
            estimate.DecayRate = decayRate;
            estimate.SecondsToTolerance = Math.Max(1.0, toleranceDrop / decayRate);
            estimate.DeltaVToRestoreToleranceDrop = EstimateDeltaVToRestoreDrop(vessel, toleranceDrop);
            estimate.Reason = string.Empty;
            return true;
        }

        /// <summary>
        /// Estimates station-keeping cadence for editor/planning tools where no
        /// Vessel instance exists yet. Solar-storm estimates are not available in
        /// this path because there is no launched vessel position or Kerbalism state.
        /// </summary>
        public static bool TryEstimateStationKeepingCadenceForOrbit(
            CelestialBody body,
            double targetApoapsis,
            double targetPeriapsis,
            double tolerancePercent,
            double vesselMass,
            out StationKeepingEstimate estimate)
        {
            estimate = new StationKeepingEstimate
            {
                Available = false,
                Reason = "Unavailable"
            };

            if (body == null)
            {
                estimate.Reason = "No body";
                return false;
            }

            double targetApR = body.Radius + targetApoapsis;
            double targetPeR = body.Radius + targetPeriapsis;
            if (targetApoapsis < targetPeriapsis || targetPeR <= body.Radius || targetApR < targetPeR)
            {
                estimate.Reason = "Invalid orbit";
                return false;
            }

            double toleranceDrop = EstimateToleranceDropForTargetOrbit(
                targetApoapsis, targetPeriapsis, tolerancePercent);
            if (toleranceDrop <= 0.0)
            {
                estimate.Reason = "No tolerance margin";
                return false;
            }

            double daDt = EstimateDaDtForOrbit(body, targetApoapsis, targetPeriapsis, vesselMass);
            double decayRate = Math.Abs(daDt);
            if (decayRate <= 1e-12)
            {
                estimate.Reason = "No active decay";
                return false;
            }

            double targetSma = (targetApR + targetPeR) * 0.5;
            estimate.Available = true;
            estimate.IsStormEstimate = false;
            estimate.ToleranceDrop = toleranceDrop;
            estimate.DecayRate = decayRate;
            estimate.SecondsToTolerance = Math.Max(1.0, toleranceDrop / decayRate);
            estimate.DeltaVToRestoreToleranceDrop =
                EstimateDeltaVToRestoreDrop(body, targetSma, targetPeriapsis, toleranceDrop);
            estimate.Reason = string.Empty;
            return true;
        }

        /// <summary>
        /// Estimates the current SWAOD semi-major-axis decay rate in m/s.
        /// Negative values mean orbital decay, zero means SWAOD is not acting.
        /// </summary>
        public static double EstimateCurrentDaDt(Vessel vessel, out bool stormEstimate)
        {
            return EstimateDaDt(vessel, out stormEstimate);
        }

        private static double EstimateDaDt(Vessel vessel, out bool stormEstimate)
        {
            stormEstimate = false;
            if (vessel == null || vessel.orbit == null || vessel.mainBody == null)
                return 0.0;

            ApiSettings cfg = GetSettings();
            CelestialBody body = vessel.mainBody;
            Orbit orbit = vessel.orbit;
            double periapsisAlt = Math.Max(orbit.PeA, 0.0);
            double daDt = 0.0;

            if (cfg.NaturalDecayEnabled && body.atmosphere)
            {
                double maxAlt = body.atmosphereDepth * cfg.NaturalDecayAltitudeCutoff;
                if (periapsisAlt <= maxAlt)
                {
                    double mass = vessel.GetTotalMass();
                    if (mass <= 0.001)
                        mass = 0.1;
                    daDt += AtmosphericDecayModel.EstimateNaturalDaDt(
                        body, orbit, mass, GetDecaySettings(cfg));
                }
            }

            if (KerbalismIntegration.IsStormInProgress(vessel))
            {
                bool stormInRange = false;
                if (body.atmosphere)
                {
                    stormInRange = periapsisAlt <= body.atmosphereDepth * cfg.NaturalDecayAltitudeCutoff;
                }
                else if (cfg.ApplyStormDecayToNoAtmosphereBody)
                {
                    stormInRange = periapsisAlt <= body.sphereOfInfluence - body.Radius;
                }

                if (stormInRange)
                {
                    double distanceFactor = 1.0;
                    if (cfg.StormDistanceScaling)
                    {
                        double dist = Math.Max(GetDistanceToSun(vessel), 1000.0);
                        distanceFactor = Math.Pow(AU / dist, 2.0);
                    }

                    daDt += -orbit.semiMajorAxis * cfg.StormDecayRate * distanceFactor;
                    stormEstimate = true;
                }
            }

            return daDt;
        }

        private static double EstimateDaDtForOrbit(
            CelestialBody body,
            double targetApoapsis,
            double targetPeriapsis,
            double vesselMass)
        {
            if (body == null)
                return 0.0;

            ApiSettings cfg = GetSettings();
            if (!cfg.NaturalDecayEnabled || !body.atmosphere)
                return 0.0;

            double periapsisAlt = Math.Max(targetPeriapsis, 0.0);
            double maxAlt = body.atmosphereDepth * cfg.NaturalDecayAltitudeCutoff;
            if (periapsisAlt > maxAlt)
                return 0.0;

            double targetApR = body.Radius + targetApoapsis;
            double targetPeR = body.Radius + targetPeriapsis;
            double semiMajorAxis = (targetApR + targetPeR) * 0.5;
            if (semiMajorAxis <= 0.0)
                return 0.0;

            double eccentricity = (targetApR - targetPeR) / (targetApR + targetPeR);
            return AtmosphericDecayModel.EstimateNaturalDaDt(
                body,
                semiMajorAxis,
                Math.Max(0.0, eccentricity),
                Math.Max(vesselMass, 0.1),
                GetDecaySettings(cfg));
        }

        private static double EstimateToleranceDrop(
            Vessel vessel,
            double targetApoapsis,
            double targetPeriapsis,
            double tolerancePercent)
        {
            double tolerance = Math.Max(0.01, tolerancePercent / 100.0);
            double apRoom = vessel.orbit.ApA - GetLowerToleranceAltitude(targetApoapsis, tolerance);
            double peRoom = vessel.orbit.PeA - GetLowerToleranceAltitude(targetPeriapsis, tolerance);

            double room = double.PositiveInfinity;
            if (apRoom > 0.0) room = Math.Min(room, apRoom);
            if (peRoom > 0.0) room = Math.Min(room, peRoom);

            if (!double.IsInfinity(room))
                return Math.Max(1.0, room);

            double apBand = GetToleranceBand(targetApoapsis, tolerance);
            double peBand = GetToleranceBand(targetPeriapsis, tolerance);
            return Math.Max(1.0, Math.Min(apBand, peBand));
        }

        private static double EstimateToleranceDropForTargetOrbit(
            double targetApoapsis,
            double targetPeriapsis,
            double tolerancePercent)
        {
            double tolerance = Math.Max(0.01, tolerancePercent / 100.0);
            double apBand = GetToleranceBand(targetApoapsis, tolerance);
            double peBand = GetToleranceBand(targetPeriapsis, tolerance);
            return Math.Max(1.0, Math.Min(apBand, peBand));
        }

        private static double GetLowerToleranceAltitude(double targetAltitude, double tolerance)
        {
            if (Math.Abs(targetAltitude) < 1.0)
                return targetAltitude - 1000.0 * tolerance;
            return targetAltitude * (1.0 - tolerance);
        }

        private static double GetToleranceBand(double targetAltitude, double tolerance)
        {
            if (Math.Abs(targetAltitude) < 1.0)
                return 1000.0 * tolerance;
            return Math.Abs(targetAltitude) * tolerance;
        }

        private static double EstimateDeltaVToRestoreDrop(Vessel vessel, double dropMeters)
        {
            if (vessel == null || vessel.orbit == null || dropMeters <= 0.0)
                return 0.0;

            Orbit orbit = vessel.orbit;
            CelestialBody body = orbit.referenceBody;
            double a = orbit.semiMajorAxis;
            double r = body.Radius + Math.Max(orbit.PeA, 0.0);
            double vSq = body.gravParameter * (2.0 / r - 1.0 / a);
            double speed = Math.Sqrt(Math.Max(0.0, vSq));

            if (a <= 0.0 || speed <= 1e-6)
                return 0.0;

            return Math.Abs((body.gravParameter / (2.0 * a * a * speed)) * dropMeters);
        }

        private static double EstimateDeltaVToRestoreDrop(
            CelestialBody body,
            double semiMajorAxis,
            double periapsisAltitude,
            double dropMeters)
        {
            if (body == null || semiMajorAxis <= 0.0 || dropMeters <= 0.0)
                return 0.0;

            double r = body.Radius + Math.Max(periapsisAltitude, 0.0);
            double vSq = body.gravParameter * (2.0 / r - 1.0 / semiMajorAxis);
            double speed = Math.Sqrt(Math.Max(0.0, vSq));

            if (speed <= 1e-6)
                return 0.0;

            return Math.Abs((body.gravParameter / (2.0 * semiMajorAxis * semiMajorAxis * speed)) * dropMeters);
        }

        private static ApiSettings GetSettings()
        {
            if (settingsLoaded)
                return settings;

            settingsLoaded = true;
            settings = new ApiSettings
            {
                Loaded = true,
                StormDecayRate = 1.5e-7,
                StormDistanceScaling = true,
                ApplyStormDecayToNoAtmosphereBody = false,
                NaturalDecayEnabled = true,
                NaturalDecayMultiplier = 1.0,
                NaturalDecayAltitudeCutoff = 10.0,
                ExosphereFitStart = 0.80,
                ExosphereFitEnd = 0.90,
                ExosphereScaleHeightMin = 0.03,
                ExosphereScaleHeightMax = 0.30,
                ExosphereFitSamples = 8,
                OrbitAverageSamples = 24
            };

            ConfigNode[] nodes = GameDatabase.Instance?.GetConfigNodes("ORBITAL_DECAY");
            if (nodes == null || nodes.Length == 0)
                return settings;

            ConfigNode cfg = nodes[0];
            cfg.TryGetValue("stormDecayRate", ref settings.StormDecayRate);
            cfg.TryGetValue("stormDistanceScaling", ref settings.StormDistanceScaling);
            cfg.TryGetValue("applyStormDecayToNoAtmosphereBody", ref settings.ApplyStormDecayToNoAtmosphereBody);
            cfg.TryGetValue("naturalDecayEnabled", ref settings.NaturalDecayEnabled);
            cfg.TryGetValue("naturalDecayMultiplier", ref settings.NaturalDecayMultiplier);
            cfg.TryGetValue("naturalDecayAltitudeCutoff", ref settings.NaturalDecayAltitudeCutoff);
            cfg.TryGetValue("exosphereFitStart", ref settings.ExosphereFitStart);
            cfg.TryGetValue("exosphereFitEnd", ref settings.ExosphereFitEnd);
            cfg.TryGetValue("exosphereScaleHeightMin", ref settings.ExosphereScaleHeightMin);
            cfg.TryGetValue("exosphereScaleHeightMax", ref settings.ExosphereScaleHeightMax);
            cfg.TryGetValue("exosphereFitSamples", ref settings.ExosphereFitSamples);
            cfg.TryGetValue("orbitAverageSamples", ref settings.OrbitAverageSamples);

            return settings;
        }

        private static AtmosphericDecayModel.DecaySettings GetDecaySettings(ApiSettings cfg)
        {
            AtmosphericDecayModel.DecaySettings decaySettings = AtmosphericDecayModel.GetDefaultSettings();
            decaySettings.NaturalDecayMultiplier = cfg.NaturalDecayMultiplier;
            decaySettings.NaturalDecayAltitudeCutoff = cfg.NaturalDecayAltitudeCutoff;
            decaySettings.ExosphereFitStart = cfg.ExosphereFitStart;
            decaySettings.ExosphereFitEnd = cfg.ExosphereFitEnd;
            decaySettings.ExosphereScaleHeightMin = cfg.ExosphereScaleHeightMin;
            decaySettings.ExosphereScaleHeightMax = cfg.ExosphereScaleHeightMax;
            decaySettings.ExosphereFitSamples = cfg.ExosphereFitSamples;
            decaySettings.OrbitAverageSamples = cfg.OrbitAverageSamples;
            return decaySettings;
        }

        private static double GetDistanceToSun(Vessel vessel)
        {
            CelestialBody sun = FlightGlobals.Bodies[0];
            if (vessel.mainBody == sun)
                return vessel.orbit.radius;

            Vector3d sunPos = sun.position;
            Vector3d bodyPos = vessel.mainBody.position;
            Vector3d relPos = vessel.orbit.pos;

            return Vector3d.Distance(bodyPos + relPos, sunPos);
        }
    }
}
