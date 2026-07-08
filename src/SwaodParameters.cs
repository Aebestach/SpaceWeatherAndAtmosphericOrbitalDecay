using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KSP.Localization;
using UnityEngine;

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    public class SwaodParameters : GameParameters.CustomParameterNode
    {
        public override string Title => Localizer.Format("#LOC_SWAOD_ParamTitle");
        public override GameParameters.GameMode GameMode => GameParameters.GameMode.ANY;
        public override string Section => Localizer.Format("#LOC_SWAOD_ParamSection");
        public override string DisplaySection => Section;
        public override int SectionOrder => 1;
        public override bool HasPresets => false;

        [GameParameters.CustomParameterUI(
            "#LOC_SWAOD_ParamUiScaleAuto",
            toolTip = "#LOC_SWAOD_ParamUiScaleAuto_tip")]
        public bool uiScaleAuto = true;

        [GameParameters.CustomFloatParameterUI(
            "#LOC_SWAOD_ParamUiScalePercent",
            toolTip = "#LOC_SWAOD_ParamUiScalePercent_tip",
            minValue = 50f,
            maxValue = 150f,
            stepCount = 100,
            displayFormat = "N0")]
        public float uiScalePercent = 100f;

        [GameParameters.CustomParameterUI(
            "#LOC_SWAOD_ParamDebugMode",
            toolTip = "#LOC_SWAOD_ParamDebugMode_tip")]
        public bool debugMode = false;

        [GameParameters.CustomStringParameterUI(
            "#LOC_SWAOD_ParamHotkeyKey",
            toolTip = "#LOC_SWAOD_ParamHotkeyKey_tip")]
        public string hotkeyKey = "Q";

        [GameParameters.CustomParameterUI(
            "#LOC_SWAOD_ParamHotkeyAlt",
            toolTip = "#LOC_SWAOD_ParamHotkeyAlt_tip")]
        public bool hotkeyAlt = true;

        [GameParameters.CustomParameterUI(
            "#LOC_SWAOD_ParamHotkeyCtrl",
            toolTip = "#LOC_SWAOD_ParamHotkeyCtrl_tip")]
        public bool hotkeyCtrl = false;

        [GameParameters.CustomParameterUI(
            "#LOC_SWAOD_ParamHotkeyShift",
            toolTip = "#LOC_SWAOD_ParamHotkeyShift_tip")]
        public bool hotkeyShift = false;

        private static SwaodParameters instance;

        public static SwaodParameters Instance
        {
            get
            {
                if (instance == null && HighLogic.CurrentGame != null)
                    instance = HighLogic.CurrentGame.Parameters.CustomParams<SwaodParameters>();
                return instance;
            }
        }

        internal static bool IsDebugModeEnabled => Instance?.debugMode ?? false;

        internal KeyCode ResolveHotkeyKey()
        {
            if (string.IsNullOrEmpty(hotkeyKey) || string.Equals(hotkeyKey, "None", StringComparison.OrdinalIgnoreCase))
                return KeyCode.None;

            return Enum.TryParse(hotkeyKey, true, out KeyCode parsed) ? parsed : KeyCode.Q;
        }

        internal bool IsHotkeyPressed()
        {
            KeyCode key = ResolveHotkeyKey();
            if (key == KeyCode.None || !Input.GetKeyDown(key))
                return false;
            if (hotkeyAlt && !(Input.GetKey(KeyCode.LeftAlt) || Input.GetKey(KeyCode.RightAlt)))
                return false;
            if (hotkeyCtrl && !(Input.GetKey(KeyCode.LeftControl) || Input.GetKey(KeyCode.RightControl)))
                return false;
            if (hotkeyShift && !(Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift)))
                return false;
            return true;
        }

        public override void OnLoad(ConfigNode node)
        {
            bool hadAutoFlag = node != null && node.HasValue("uiScaleAuto");
            bool hadUiScale = node != null && node.HasValue("uiScalePercent");
            base.OnLoad(node);
            instance = null;

            if (!hadAutoFlag)
            {
                uiScaleAuto = !hadUiScale ||
                    Mathf.Approximately(uiScalePercent, 100f) ||
                    Mathf.Approximately(uiScalePercent, 80f) ||
                    Mathf.Approximately(uiScalePercent, 75f);
            }

            ApplyAutoUiScale();
        }

        internal void ApplyAutoUiScale()
        {
            if (!uiScaleAuto)
                return;
            uiScalePercent = UIScale.DefaultUiScalePercent;
        }

        public override bool Enabled(MemberInfo member, GameParameters parameters)
        {
            var swaod = parameters?.CustomParams<SwaodParameters>();
            if (swaod != null && swaod.uiScaleAuto)
                swaod.ApplyAutoUiScale();
            return true;
        }

        public override bool Interactible(MemberInfo member, GameParameters parameters)
        {
            var swaod = parameters?.CustomParams<SwaodParameters>();
            if (member.Name == "uiScalePercent" && swaod != null && swaod.uiScaleAuto)
                return false;
            return true;
        }

        public override IList ValidValues(MemberInfo member)
        {
            if (member.Name != "hotkeyKey")
                return null;

            var keys = new List<string> { "None" };
            for (char letter = 'A'; letter <= 'Z'; letter++)
                keys.Add(letter.ToString());
            return keys;
        }
    }
}
