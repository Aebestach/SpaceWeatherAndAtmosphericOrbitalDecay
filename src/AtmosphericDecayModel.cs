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

        internal struct NaturalDecayRates
        {
            public double DaDt;
            public double DeDt;
            public double PeriapsisDaDt;
            public double ApoapsisDaDt;
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
        private const int MaxTimeEstimateSteps = 1024;
        private static readonly double[] Gauss8Nodes =
        {
            -0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
             0.1834346424956498,  0.5255324099163290,  0.7966664774136267,  0.9602898564975363
        };
        private static readonly double[] Gauss8Weights =
        {
            0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
            0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763
        };
        private static readonly double[] Gauss12Nodes =
        {
            -0.9815606342467192, -0.9041172563704749, -0.7699026741943047, -0.5873179542866175,
            -0.3678314989981802, -0.1252334085114689,  0.1252334085114689,  0.3678314989981802,
             0.5873179542866175,  0.7699026741943047,  0.9041172563704749,  0.9815606342467192
        };
        private static readonly double[] Gauss12Weights =
        {
            0.0471753363865118, 0.1069393259953184, 0.1600783285433462, 0.2031674267230659,
            0.2334925365383548, 0.2491470458134028, 0.2491470458134028, 0.2334925365383548,
            0.2031674267230659, 0.1600783285433462, 0.1069393259953184, 0.0471753363865118
        };
        private static readonly double[] Gauss16Nodes =
        {
            -0.9894009349916499, -0.9445750230732326, -0.8656312023878318, -0.7554044083550030,
            -0.6178762444026438, -0.4580167776572274, -0.2816035507792589, -0.0950125098376374,
             0.0950125098376374,  0.2816035507792589,  0.4580167776572274,  0.6178762444026438,
             0.7554044083550030,  0.8656312023878318,  0.9445750230732326,  0.9894009349916499
        };
        private static readonly double[] Gauss16Weights =
        {
            0.0271524594117541, 0.0622535239386479, 0.0951585116824928, 0.1246289712555339,
            0.1495959888165767, 0.1691565193950025, 0.1826034150449236, 0.1894506104550685,
            0.1894506104550685, 0.1826034150449236, 0.1691565193950025, 0.1495959888165767,
            0.1246289712555339, 0.0951585116824928, 0.0622535239386479, 0.0271524594117541
        };
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

        internal static NaturalDecayRates EstimateNaturalDecayRates(
            CelestialBody body,
            Orbit orbit,
            double vesselMassTons,
            DecaySettings settings)
        {
            if (orbit == null)
                return new NaturalDecayRates();

            return EstimateNaturalDecayRates(
                body,
                orbit.semiMajorAxis,
                orbit.eccentricity,
                vesselMassTons,
                settings);
        }

        internal static NaturalDecayRates EstimateNaturalDecayRates(
            CelestialBody body,
            double semiMajorAxis,
            double eccentricity,
            double vesselMassTons,
            DecaySettings settings)
        {
            if (body == null || !body.atmosphere || semiMajorAxis <= 0.0 || eccentricity < 0.0 || eccentricity >= 1.0)
                return new NaturalDecayRates();

            double atmDepth = body.atmosphereDepth;
            if (atmDepth <= 0.0)
                return new NaturalDecayRates();

            double periapsisRadius = semiMajorAxis * (1.0 - eccentricity);
            double periapsisAltitude = periapsisRadius - body.Radius;
            double maxAlt = atmDepth * Math.Max(settings.NaturalDecayAltitudeCutoff, 1.0);
            double maxRadius = body.Radius + maxAlt;
            if (periapsisAltitude > maxAlt)
                return new NaturalDecayRates();

            double mu = body.gravParameter;
            if (mu <= 0.0)
                return new NaturalDecayRates();

            double massTons = Math.Max(vesselMassTons, 0.1);
            double massKg = massTons * 1000.0;
            double area = Math.Pow(massTons, 0.666) * 4.0;
            double multiplier = Math.Max(settings.NaturalDecayMultiplier, 0.0);
            if (multiplier <= 0.0)
                return new NaturalDecayRates();

            double h = Math.Sqrt(Math.Max(0.0, mu * semiMajorAxis * (1.0 - eccentricity * eccentricity)));
            if (h <= 0.0)
                return new NaturalDecayRates();

            double energyRate = 0.0;
            double angularMomentumRate = 0.0;

            if (eccentricity < 1e-5)
            {
                double altitude = semiMajorAxis - body.Radius;
                if (altitude > maxAlt)
                    return new NaturalDecayRates();

                AccumulateDecayAtRadius(
                    body,
                    semiMajorAxis,
                    semiMajorAxis,
                    altitude,
                    mu,
                    massKg,
                    area,
                    h,
                    settings,
                    1.0,
                    ref energyRate,
                    ref angularMomentumRate);
            }
            else
            {
                double apoapsisRadius = semiMajorAxis * (1.0 + eccentricity);
                double eccentricAnomalyLimit = Math.PI;
                if (apoapsisRadius > maxRadius)
                {
                    double cosLimit = (1.0 - maxRadius / semiMajorAxis) / eccentricity;
                    cosLimit = Clamp(cosLimit, -1.0, 1.0);
                    eccentricAnomalyLimit = Math.Acos(cosLimit);
                }

                SelectQuadrature(eccentricity, eccentricAnomalyLimit, settings.OrbitAverageSamples, out double[] nodes, out double[] weights);
                double anomalyScale = eccentricAnomalyLimit;
                double averageScale = anomalyScale / TwoPi;

                for (int i = 0; i < nodes.Length; i++)
                {
                    double eccentricAnomaly = nodes[i] * anomalyScale;
                    double cosE = Math.Cos(eccentricAnomaly);
                    double radius = semiMajorAxis * (1.0 - eccentricity * cosE);
                    if (radius <= body.Radius)
                        radius = body.Radius + 1.0;

                    double altitude = radius - body.Radius;
                    if (altitude > maxAlt)
                        continue;

                    double meanAnomalyWeight = 1.0 - eccentricity * cosE;
                    double orbitAverageWeight = weights[i] * averageScale * meanAnomalyWeight;
                    AccumulateDecayAtRadius(
                        body,
                        semiMajorAxis,
                        radius,
                        altitude,
                        mu,
                        massKg,
                        area,
                        h,
                        settings,
                        orbitAverageWeight,
                        ref energyRate,
                        ref angularMomentumRate);
                }
            }

            energyRate *= multiplier;
            angularMomentumRate *= multiplier;

            if (energyRate >= 0.0 || Math.Abs(energyRate) <= 1e-30)
                return new NaturalDecayRates();

            double daDt = (2.0 * semiMajorAxis * semiMajorAxis / mu) * energyRate;
            double deDt = 0.0;
            if (eccentricity > 1e-6)
            {
                double specificEnergy = -mu / (2.0 * semiMajorAxis);
                deDt = (h * h * energyRate + 2.0 * specificEnergy * h * angularMomentumRate) /
                       (eccentricity * mu * mu);
            }

            return BuildRates(semiMajorAxis, eccentricity, daDt, deDt);
        }

        internal static double EstimateTimeToAtmosphere(
            CelestialBody body,
            double semiMajorAxis,
            double eccentricity,
            double vesselMassTons,
            double extraDragMultiplier,
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

                DecaySettings stepSettings = settings;
                stepSettings.NaturalDecayMultiplier =
                    Math.Max(settings.NaturalDecayMultiplier, 0.0) +
                    Math.Max(extraDragMultiplier, 0.0);
                NaturalDecayRates rates = EstimateNaturalDecayRates(body, a, e, vesselMassTons, stepSettings);
                double targetRate = useApoapsis ? rates.ApoapsisDaDt : rates.PeriapsisDaDt;

                if (targetRate >= -1e-20)
                    return double.PositiveInfinity;

                double remainingAltitude = targetAltitude - atmDepth;
                double stepSeconds = Math.Abs(remainingAltitude / targetRate) / 12.0;
                stepSeconds = Clamp(stepSeconds, 60.0, 86400.0 * 5.0);

                double maxTargetDrop = Math.Max(remainingAltitude * 0.5, 100.0);
                double targetDrop = -targetRate * stepSeconds;
                if (targetDrop > maxTargetDrop)
                    stepSeconds *= maxTargetDrop / targetDrop;

                OrbitElements next = ApplyDecayToElements(
                    body,
                    a,
                    e,
                    rates.DaDt * stepSeconds,
                    rates.DeDt * stepSeconds);
                if (Math.Abs(next.SemiMajorAxis - a) < 1e-6)
                    return double.PositiveInfinity;

                a = next.SemiMajorAxis;
                e = next.Eccentricity;
                totalTime += stepSeconds;

                if (totalTime > maxEstimateTime)
                    return totalTime;
            }

            return double.PositiveInfinity;
        }

        internal static OrbitElements ApplyDecayToElements(
            CelestialBody body,
            double semiMajorAxis,
            double eccentricity,
            double deltaSma,
            double deltaEccentricity)
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
            double newE = eccentricity + deltaEccentricity;

            if (newE < 0.0)
                newE = 0.0;
            if (newE >= 1.0)
                newE = 0.999999;

            double minPeriapsisRadius = body.Radius + 1.0;
            double periapsisRadius = newA * (1.0 - newE);
            if (periapsisRadius < minPeriapsisRadius)
                newE = Math.Max(0.0, 1.0 - minPeriapsisRadius / newA);

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

        private static NaturalDecayRates BuildRates(
            double semiMajorAxis,
            double eccentricity,
            double daDt,
            double deDt)
        {
            return new NaturalDecayRates
            {
                DaDt = daDt,
                DeDt = deDt,
                PeriapsisDaDt = daDt * (1.0 - eccentricity) - semiMajorAxis * deDt,
                ApoapsisDaDt = daDt * (1.0 + eccentricity) + semiMajorAxis * deDt
            };
        }

        private static void SelectQuadrature(
            double eccentricity,
            double eccentricAnomalyLimit,
            int configuredSamples,
            out double[] nodes,
            out double[] weights)
        {
            int sampleBudget = Math.Max(8, Math.Min(16, configuredSamples));
            bool narrowPeriapsisArc = eccentricAnomalyLimit < 0.70;

            if (sampleBudget >= 16 && (eccentricity >= 0.45 || narrowPeriapsisArc))
            {
                nodes = Gauss16Nodes;
                weights = Gauss16Weights;
                return;
            }

            if (sampleBudget >= 12 && (eccentricity >= 0.20 || eccentricAnomalyLimit < 1.20))
            {
                nodes = Gauss12Nodes;
                weights = Gauss12Weights;
                return;
            }

            nodes = Gauss8Nodes;
            weights = Gauss8Weights;
        }

        private static void AccumulateDecayAtRadius(
            CelestialBody body,
            double semiMajorAxis,
            double radius,
            double altitude,
            double mu,
            double massKg,
            double area,
            double angularMomentum,
            DecaySettings settings,
            double orbitAverageWeight,
            ref double energyRate,
            ref double angularMomentumRate)
        {
            double density = GetDensity(body, altitude, settings);
            if (density <= 0.0)
                density = MinimumDensity;
            else if (density < MinimumDensity)
                density = MinimumDensity;

            double velocitySquared = mu * (2.0 / radius - 1.0 / semiMajorAxis);
            if (velocitySquared <= 0.0)
                return;

            double velocity = Math.Sqrt(velocitySquared);
            double dragAcceleration = 0.5 * Cd * area / massKg * density * velocitySquared;
            double specificEnergyRate = -dragAcceleration * velocity;
            double specificAngularMomentumRate = -dragAcceleration * angularMomentum / velocity;

            energyRate += specificEnergyRate * orbitAverageWeight;
            angularMomentumRate += specificAngularMomentumRate * orbitAverageWeight;
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
