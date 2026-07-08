using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KSP.Localization;
using UnityEngine;

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    /// <summary>
    /// Decay gameplay settings migrated from Settings.cfg.
    /// </summary>
    public class SwaodGameplayParameters : GameParameters.CustomParameterNode
    {
        public override string Title => Localizer.Format("#LOC_SWAOD_ParamGameplayTitle");
        public override GameParameters.GameMode GameMode => GameParameters.GameMode.ANY;
        public override string Section => Localizer.Format("#LOC_SWAOD_ParamSection");
        public override string DisplaySection => Section;
        public override int SectionOrder => 0;
        public override bool HasPresets => false;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamStormDecayRate",
            toolTip = "#LOC_SWAOD_ParamStormDecayRate_tip",
            minValue = 0f,
            maxValue = 5e-6f,
            stepCount = 100,
            displayFormat = "E2")]
        public float stormDecayRate = 1.5e-7f;

        [GameParameters.CustomParameterUI(
            "#LOC_SWAOD_ParamStormDistanceScaling",
            toolTip = "#LOC_SWAOD_ParamStormDistanceScaling_tip")]
        public bool stormDistanceScaling = true;

        [GameParameters.CustomParameterUI(
            "#LOC_SWAOD_ParamNaturalDecayEnabled",
            toolTip = "#LOC_SWAOD_ParamNaturalDecayEnabled_tip")]
        public bool naturalDecayEnabled = true;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamNaturalDecayMultiplier",
            toolTip = "#LOC_SWAOD_ParamNaturalDecayMultiplier_tip",
            minValue = 0f,
            maxValue = 10f,
            stepCount = 100,
            displayFormat = "F2")]
        public float naturalDecayMultiplier = 1f;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamNaturalDecayAltitudeCutoff",
            toolTip = "#LOC_SWAOD_ParamNaturalDecayAltitudeCutoff_tip",
            minValue = 1f,
            maxValue = 50f,
            stepCount = 49,
            displayFormat = "F1")]
        public float naturalDecayAltitudeCutoff = 10f;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamExosphereFitStart",
            toolTip = "#LOC_SWAOD_ParamExosphereFitStart_tip",
            minValue = 0.5f,
            maxValue = 0.95f,
            stepCount = 45,
            displayFormat = "F2")]
        public float exosphereFitStart = 0.80f;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamExosphereFitEnd",
            toolTip = "#LOC_SWAOD_ParamExosphereFitEnd_tip",
            minValue = 0.5f,
            maxValue = 0.99f,
            stepCount = 49,
            displayFormat = "F2")]
        public float exosphereFitEnd = 0.90f;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamExosphereScaleHeightMin",
            toolTip = "#LOC_SWAOD_ParamExosphereScaleHeightMin_tip",
            minValue = 0.01f,
            maxValue = 1f,
            stepCount = 99,
            displayFormat = "F2")]
        public float exosphereScaleHeightMin = 0.03f;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamExosphereScaleHeightMax",
            toolTip = "#LOC_SWAOD_ParamExosphereScaleHeightMax_tip",
            minValue = 0.01f,
            maxValue = 1f,
            stepCount = 99,
            displayFormat = "F2")]
        public float exosphereScaleHeightMax = 0.30f;

        [GameParameters.CustomIntParameterUI(
            "#LOC_SWAOD_ParamExosphereFitSamples",
            toolTip = "#LOC_SWAOD_ParamExosphereFitSamples_tip",
            minValue = 2,
            maxValue = 32,
            stepSize = 1)]
        public int exosphereFitSamples = 8;

        [GameParameters.CustomIntParameterUI(
            "#LOC_SWAOD_ParamOrbitAverageSamples",
            toolTip = "#LOC_SWAOD_ParamOrbitAverageSamples_tip",
            minValue = 4,
            maxValue = 64,
            stepSize = 1)]
        public int orbitAverageSamples = 24;

        [GameParameters.CustomParameterUI(
            "#LOC_SWAOD_ParamWarningEnabled",
            toolTip = "#LOC_SWAOD_ParamWarningEnabled_tip")]
        public bool warningEnabled = true;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamWarningThreshold",
            toolTip = "#LOC_SWAOD_ParamWarningThreshold_tip",
            minValue = 0f,
            maxValue = 2f,
            stepCount = 40,
            displayFormat = "F2")]
        public float warningThreshold = 0.2f;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamReentryDestroySeconds",
            toolTip = "#LOC_SWAOD_ParamReentryDestroySeconds_tip",
            minValue = 1f,
            maxValue = 600f,
            stepCount = 599,
            displayFormat = "N0")]
        public float reentryDestroySeconds = 60f;

        private static SwaodGameplayParameters instance;

        public static SwaodGameplayParameters Instance
        {
            get
            {
                if (instance == null && HighLogic.CurrentGame != null)
                    instance = HighLogic.CurrentGame.Parameters.CustomParams<SwaodGameplayParameters>();
                return instance;
            }
        }

        internal void ApplyTo(
            ref double stormRate,
            ref bool distanceScaling,
            ref bool naturalEnabled,
            ref double naturalMultiplier,
            ref double altitudeCutoff,
            ref double fitStart,
            ref double fitEnd,
            ref double scaleHeightMin,
            ref double scaleHeightMax,
            ref int fitSamples,
            ref int orbitSamples,
            ref bool warningsEnabled,
            ref double warnThreshold,
            ref double destroySeconds)
        {
            stormRate = stormDecayRate;
            distanceScaling = stormDistanceScaling;
            naturalEnabled = naturalDecayEnabled;
            naturalMultiplier = naturalDecayMultiplier;
            altitudeCutoff = naturalDecayAltitudeCutoff;
            fitStart = exosphereFitStart;
            fitEnd = exosphereFitEnd;
            scaleHeightMin = exosphereScaleHeightMin;
            scaleHeightMax = exosphereScaleHeightMax;
            fitSamples = exosphereFitSamples;
            orbitSamples = orbitAverageSamples;
            warningsEnabled = warningEnabled;
            warnThreshold = warningThreshold;
            destroySeconds = reentryDestroySeconds > 0f ? reentryDestroySeconds : 60f;
        }

        public override void OnLoad(ConfigNode node)
        {
            base.OnLoad(node);
            instance = null;

            if (!node.HasValue("stormDecayRate"))
                TryMigrateFromLegacyCfg();
        }

        private void TryMigrateFromLegacyCfg()
        {
            ConfigNode[] nodes = GameDatabase.Instance?.GetConfigNodes("ORBITAL_DECAY");
            if (nodes == null || nodes.Length == 0)
                return;

            ConfigNode cfg = nodes[0];
            cfg.TryGetValue("stormDecayRate", ref stormDecayRate);
            cfg.TryGetValue("stormDistanceScaling", ref stormDistanceScaling);
            cfg.TryGetValue("naturalDecayEnabled", ref naturalDecayEnabled);
            cfg.TryGetValue("naturalDecayMultiplier", ref naturalDecayMultiplier);
            cfg.TryGetValue("naturalDecayAltitudeCutoff", ref naturalDecayAltitudeCutoff);
            cfg.TryGetValue("exosphereFitStart", ref exosphereFitStart);
            cfg.TryGetValue("exosphereFitEnd", ref exosphereFitEnd);
            cfg.TryGetValue("exosphereScaleHeightMin", ref exosphereScaleHeightMin);
            cfg.TryGetValue("exosphereScaleHeightMax", ref exosphereScaleHeightMax);
            cfg.TryGetValue("exosphereFitSamples", ref exosphereFitSamples);
            cfg.TryGetValue("orbitAverageSamples", ref orbitAverageSamples);
            cfg.TryGetValue("warningEnabled", ref warningEnabled);
            cfg.TryGetValue("warningThreshold", ref warningThreshold);
            cfg.TryGetValue("reentryDestroySeconds", ref reentryDestroySeconds);
            if (reentryDestroySeconds <= 0f)
                reentryDestroySeconds = 60f;
        }
    }
}
