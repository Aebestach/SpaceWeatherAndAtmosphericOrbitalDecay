using System;
using System.Collections.Generic;

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    /// <summary>
    /// Shared natural atmospheric decay model used by runtime decay, UI estimates,
    /// and the public API.
    /// </summary>
    internal static class AtmosphericDecayModel
    {
        internal struct DecaySettings
        {
            public double NaturalDecayMultiplier;
            public double NaturalDecayAltitudeCutoff;
            public double ExosphereFitStart;
            public double ExosphereFitEnd;
            public double ExosphereScaleHeightMin;
            public double ExosphereScaleHeightMax;
            public int ExosphereFitSamples;
            public int OrbitAverageSamples;
        }

        internal struct OrbitElements
        {
            public double SemiMajorAxis;
            public double Eccentricity;
        }

        private sealed class ExosphereProfile
        {
            public CelestialBody Body;
            public double AtmosphereDepth;
            public double FitStart;
            public double FitEnd;
            public double MinScaleHeightRatio;
            public double MaxScaleHeightRatio;
            public int FitSamples;
            public double ReferenceAltitude;
            public double ReferenceDensity;
            public double ScaleHeight;
        }

        private const double TwoPi = Math.PI * 2.0;
        private const double MinimumDensity = 1e-22;
        private const double MinimumReliableDensity = 1e-30;
        private const double Cd = 2.0;
        private const int MaxTimeEstimateSteps = 180;
        private static readonly Dictionary<CelestialBody, ExosphereProfile> exosphereProfiles =
            new Dictionary<CelestialBody, ExosphereProfile>();

        internal static double GetDensity(CelestialBody body, double altitude, DecaySettings settings)
        {
            if (body == null || !body.atmosphere || body.atmosphereDepth <= 0.0)
                return 0.0;

            double atmDepth = body.atmosphereDepth;
            double maxCutoffAlt = atmDepth * Math.Max(settings.NaturalDecayAltitudeCutoff, 1.0);
            if (altitude > maxCutoffAlt)
                return 0.0;

            if (altitude <= atmDepth)
                return GetBodyDensity(body, Math.Max(0.0, altitude));

            ExosphereProfile profile = GetExosphereProfile(body, settings);
            if (profile.ReferenceDensity <= 0.0 || profile.ScaleHeight <= 0.0)
                return 0.0;

            double exponent = -(altitude - profile.ReferenceAltitude) / profile.ScaleHeight;
            if (exponent < -700.0)
                return 0.0;

            return profile.ReferenceDensity * Math.Exp(exponent);
        }

        internal static double EstimateNaturalDaDt(
            CelestialBody body,
            Orbit orbit,
            double vesselMassTons,
            DecaySettings settings)
        {
            if (orbit == null)
                return 0.0;

            return EstimateNaturalDaDt(
                body,
                orbit.semiMajorAxis,
                orbit.eccentricity,
                vesselMassTons,
                settings);
        }

        internal static double EstimateNaturalDaDt(
            CelestialBody body,
            double semiMajorAxis,
            double eccentricity,
            double vesselMassTons,
            DecaySettings settings)
        {
            if (body == null || !body.atmosphere || semiMajorAxis <= 0.0 || eccentricity < 0.0 || eccentricity >= 1.0)
                return 0.0;

            double atmDepth = body.atmosphereDepth;
            if (atmDepth <= 0.0)
                return 0.0;

            double periapsisAltitude = semiMajorAxis * (1.0 - eccentricity) - body.Radius;
            double maxAlt = atmDepth * Math.Max(settings.NaturalDecayAltitudeCutoff, 1.0);
            if (periapsisAltitude > maxAlt)
                return 0.0;

            double mu = body.gravParameter;
            if (mu <= 0.0)
                return 0.0;

            double massTons = Math.Max(vesselMassTons, 0.1);
            double massKg = massTons * 1000.0;
            double area = Math.Pow(massTons, 0.666) * 4.0;
            int samples = GetOrbitSampleCount(eccentricity, settings.OrbitAverageSamples);
            double rhoV3Sum = 0.0;

            if (samples == 1)
            {
                AccumulateOrbitSample(body, semiMajorAxis, eccentricity, 0.0, mu, settings, ref rhoV3Sum);
            }
            else
            {
                for (int i = 0; i < samples; i++)
                {
                    double meanAnomaly = TwoPi * ((double)i + 0.5) / samples;
                    AccumulateOrbitSample(body, semiMajorAxis, eccentricity, meanAnomaly, mu, settings, ref rhoV3Sum);
                }
            }

            double averageRhoV3 = rhoV3Sum / samples;
            if (averageRhoV3 <= 0.0)
                return 0.0;

            double daDt = -((semiMajorAxis * semiMajorAxis) / mu) * ((Cd * area) / massKg) * averageRhoV3;
            return daDt * Math.Max(settings.NaturalDecayMultiplier, 0.0);
        }

        internal static double EstimateTimeToAtmosphere(
            CelestialBody body,
            double semiMajorAxis,
            double eccentricity,
            double vesselMassTons,
            double effectiveStormRate,
            bool useApoapsis,
            DecaySettings settings)
        {
            if (body == null || body.Radius <= 0.0 || semiMajorAxis <= 0.0 || eccentricity < 0.0 || eccentricity >= 1.0)
                return 0.0;

            double atmDepth = body.atmosphereDepth;
            if (!body.atmosphere || atmDepth <= 0.0)
                return double.PositiveInfinity;

            double totalTime = 0.0;
            double a = semiMajorAxis;
            double e = eccentricity;
            double maxEstimateTime = 86400.0 * 365.0 * 100.0;

            for (int i = 0; i < MaxTimeEstimateSteps; i++)
            {
                double targetAltitude = GetTargetAltitude(body, a, e, useApoapsis);
                if (targetAltitude <= atmDepth)
                    return totalTime;

                double naturalDaDt = EstimateNaturalDaDt(body, a, e, vesselMassTons, settings);
                double stormDaDt = effectiveStormRate > 0.0 ? -a * effectiveStormRate : 0.0;
                double daDt = naturalDaDt + stormDaDt;

                if (Math.Abs(daDt) < 1e-20)
                    return double.PositiveInfinity;

                double remainingAltitude = targetAltitude - atmDepth;
                double stepSeconds = Math.Abs(remainingAltitude / daDt) / 12.0;
                stepSeconds = Clamp(stepSeconds, 60.0, 21600.0);

                double deltaA = daDt * stepSeconds;
                if (-deltaA > Math.Max(remainingAltitude * 0.5, 100.0))
                    deltaA = -Math.Max(remainingAltitude * 0.5, 100.0);

                OrbitElements next = ApplyDecayToElements(body, a, e, deltaA);
                if (Math.Abs(next.SemiMajorAxis - a) < 1e-6)
                    return double.PositiveInfinity;

                a = next.SemiMajorAxis;
                e = next.Eccentricity;
                totalTime += stepSeconds;

                if (totalTime > maxEstimateTime)
                    return totalTime;
            }

            return totalTime;
        }

        internal static OrbitElements ApplyDecayToElements(
            CelestialBody body,
            double semiMajorAxis,
            double eccentricity,
            double deltaSma)
        {
            OrbitElements elements = new OrbitElements
            {
                SemiMajorAxis = semiMajorAxis,
                Eccentricity = eccentricity
            };

            if (body == null || semiMajorAxis <= 0.0)
                return elements;

            double minSma = body.Radius + 100.0;
            double newA = Math.Max(minSma, semiMajorAxis + deltaSma);
            double ratio = newA / semiMajorAxis;
            double newE = eccentricity * ratio;
            double oldRp = semiMajorAxis * (1.0 - eccentricity);

            if (eccentricity >= 0.0 && eccentricity < 1.0 && newA > 0.0)
            {
                double minE = 1.0 - (oldRp / newA);
                if (minE > newE)
                    newE = minE;
            }

            if (newE < 0.0)
                newE = 0.0;
            if (eccentricity < 1.0 && newE >= 1.0)
                newE = 0.999999;

            elements.SemiMajorAxis = newA;
            elements.Eccentricity = newE;
            return elements;
        }

        internal static DecaySettings GetDefaultSettings()
        {
            return new DecaySettings
            {
                NaturalDecayMultiplier = 1.0,
                NaturalDecayAltitudeCutoff = 10.0,
                ExosphereFitStart = 0.80,
                ExosphereFitEnd = 0.90,
                ExosphereScaleHeightMin = 0.03,
                ExosphereScaleHeightMax = 0.30,
                ExosphereFitSamples = 8,
                OrbitAverageSamples = 24
            };
        }

        private static void AccumulateOrbitSample(
            CelestialBody body,
            double semiMajorAxis,
            double eccentricity,
            double meanAnomaly,
            double mu,
            DecaySettings settings,
            ref double rhoV3Sum)
        {
            double r;
            if (eccentricity < 1e-5)
            {
                r = semiMajorAxis;
            }
            else
            {
                double eccentricAnomaly = SolveEccentricAnomaly(meanAnomaly, eccentricity);
                r = semiMajorAxis * (1.0 - eccentricity * Math.Cos(eccentricAnomaly));
            }

            if (r <= body.Radius)
                r = body.Radius + 1.0;

            double altitude = r - body.Radius;
            double maxAlt = body.atmosphereDepth * Math.Max(settings.NaturalDecayAltitudeCutoff, 1.0);
            if (altitude > maxAlt)
                return;

            double density = GetDensity(body, altitude, settings);
            if (density <= 0.0)
                density = MinimumDensity;
            else if (density < MinimumDensity)
                density = MinimumDensity;

            double velocitySquared = mu * (2.0 / r - 1.0 / semiMajorAxis);
            if (velocitySquared <= 0.0)
                return;

            double velocity = Math.Sqrt(velocitySquared);
            rhoV3Sum += density * velocitySquared * velocity;
        }

        private static ExosphereProfile GetExosphereProfile(CelestialBody body, DecaySettings settings)
        {
            ExosphereProfile profile;
            if (exosphereProfiles.TryGetValue(body, out profile) && ProfileMatches(profile, body, settings))
                return profile;

            profile = BuildExosphereProfile(body, settings);
            exosphereProfiles[body] = profile;
            return profile;
        }

        private static ExosphereProfile BuildExosphereProfile(CelestialBody body, DecaySettings settings)
        {
            double atmDepth = body.atmosphereDepth;
            double fitStart = Clamp(settings.ExosphereFitStart, 0.05, 0.98);
            double fitEnd = Clamp(settings.ExosphereFitEnd, fitStart + 0.01, 0.99);
            int fitSamples = Math.Max(3, Math.Min(32, settings.ExosphereFitSamples));

            double xMean = 0.0;
            double yMean = 0.0;
            double highestAltitude = 0.0;
            double highestDensity = 0.0;
            int validSamples = 0;

            for (int i = 0; i < fitSamples; i++)
            {
                double t = fitSamples == 1 ? 0.0 : (double)i / (fitSamples - 1);
                double altitude = atmDepth * (fitStart + (fitEnd - fitStart) * t);
                double density = GetBodyDensity(body, altitude);
                if (density <= MinimumReliableDensity || double.IsNaN(density) || double.IsInfinity(density))
                    continue;

                double logDensity = Math.Log(density);
                xMean += altitude;
                yMean += logDensity;
                validSamples++;

                if (altitude >= highestAltitude)
                {
                    highestAltitude = altitude;
                    highestDensity = density;
                }
            }

            double minScaleHeight = Math.Max(1000.0, atmDepth * Math.Max(settings.ExosphereScaleHeightMin, 0.005));
            double maxScaleHeight = Math.Max(minScaleHeight, atmDepth * Math.Max(settings.ExosphereScaleHeightMax, settings.ExosphereScaleHeightMin));
            double scaleHeight = Clamp(atmDepth * 0.10, minScaleHeight, maxScaleHeight);

            if (validSamples >= 2)
            {
                xMean /= validSamples;
                yMean /= validSamples;

                double numerator = 0.0;
                double denominator = 0.0;
                for (int i = 0; i < fitSamples; i++)
                {
                    double t = fitSamples == 1 ? 0.0 : (double)i / (fitSamples - 1);
                    double altitude = atmDepth * (fitStart + (fitEnd - fitStart) * t);
                    double density = GetBodyDensity(body, altitude);
                    if (density <= MinimumReliableDensity || double.IsNaN(density) || double.IsInfinity(density))
                        continue;

                    double dx = altitude - xMean;
                    numerator += dx * (Math.Log(density) - yMean);
                    denominator += dx * dx;
                }

                if (denominator > 0.0)
                {
                    double slope = numerator / denominator;
                    if (slope < -1e-12)
                        scaleHeight = Clamp(-1.0 / slope, minScaleHeight, maxScaleHeight);
                }
            }

            if (highestDensity <= MinimumReliableDensity)
            {
                highestAltitude = atmDepth * fitEnd;
                highestDensity = MinimumDensity;
            }

            return new ExosphereProfile
            {
                Body = body,
                AtmosphereDepth = atmDepth,
                FitStart = fitStart,
                FitEnd = fitEnd,
                MinScaleHeightRatio = settings.ExosphereScaleHeightMin,
                MaxScaleHeightRatio = settings.ExosphereScaleHeightMax,
                FitSamples = fitSamples,
                ReferenceAltitude = highestAltitude,
                ReferenceDensity = highestDensity,
                ScaleHeight = scaleHeight
            };
        }

        private static bool ProfileMatches(ExosphereProfile profile, CelestialBody body, DecaySettings settings)
        {
            return profile.Body == body &&
                   Math.Abs(profile.AtmosphereDepth - body.atmosphereDepth) < 1e-6 &&
                   Math.Abs(profile.FitStart - Clamp(settings.ExosphereFitStart, 0.05, 0.98)) < 1e-9 &&
                   Math.Abs(profile.FitEnd - Clamp(settings.ExosphereFitEnd, Clamp(settings.ExosphereFitStart, 0.05, 0.98) + 0.01, 0.99)) < 1e-9 &&
                   Math.Abs(profile.MinScaleHeightRatio - settings.ExosphereScaleHeightMin) < 1e-9 &&
                   Math.Abs(profile.MaxScaleHeightRatio - settings.ExosphereScaleHeightMax) < 1e-9 &&
                   profile.FitSamples == Math.Max(3, Math.Min(32, settings.ExosphereFitSamples));
        }

        private static double GetBodyDensity(CelestialBody body, double altitude)
        {
            double pressure = body.GetPressure(altitude);
            double temperature = body.GetTemperature(altitude);
            double density = FlightGlobals.getAtmDensity(pressure, temperature, body);
            if (double.IsNaN(density) || double.IsInfinity(density) || density < 0.0)
                return 0.0;
            return density;
        }

        private static int GetOrbitSampleCount(double eccentricity, int configuredSamples)
        {
            int samples = Math.Max(1, Math.Min(96, configuredSamples));
            if (eccentricity < 1e-5)
                return 1;
            if (eccentricity < 0.02 && samples > 8)
                return 8;
            if (eccentricity < 0.10 && samples > 16)
                return 16;
            return samples;
        }

        private static double SolveEccentricAnomaly(double meanAnomaly, double eccentricity)
        {
            double m = meanAnomaly % TwoPi;
            if (m < 0.0)
                m += TwoPi;

            double e = eccentricity < 0.8 ? m : Math.PI;
            for (int i = 0; i < 8; i++)
            {
                double sinE = Math.Sin(e);
                double cosE = Math.Cos(e);
                double f = e - eccentricity * sinE - m;
                double fp = 1.0 - eccentricity * cosE;
                if (Math.Abs(fp) < 1e-12)
                    break;

                double delta = f / fp;
                e -= delta;
                if (Math.Abs(delta) < 1e-8)
                    break;
            }

            return e;
        }

        private static double GetTargetAltitude(CelestialBody body, double semiMajorAxis, double eccentricity, bool useApoapsis)
        {
            double radius = useApoapsis
                ? semiMajorAxis * (1.0 + eccentricity)
                : semiMajorAxis * (1.0 - eccentricity);
            return radius - body.Radius;
        }

        private static double Clamp(double value, double min, double max)
        {
            if (value < min)
                return min;
            if (value > max)
                return max;
            return value;
        }
    }
}
