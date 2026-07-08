using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.Localization;
using ClickThroughFix;

namespace SpaceWeatherAndAtmosphericOrbitalDecay
{
    /// <summary>
    /// UI logic and rendering (partial of OrbitalDecay).
    /// </summary>
    public partial class OrbitalDecay
    {
        private struct VesselDisplayState
        {
            public Vessel Vessel;
            public bool IsStorming;
            public bool IsForced;
            public bool IsNatural;
            public bool StormInRange;
            public double StormDragMultiplier;
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
            
            double predictedTime = EstimateDecayTime(v, false, 0);
            Debug.Log($"[OrbitalDecay] FULL PREDICTION from {v.altitude/1000:F1}km: {FormatTime(predictedTime)}");
        }

        void OnGUI()
        {
            if (showGui)
            {
                float uiScale = UIScale.Factor;
                if (!Mathf.Approximately(uiScale, _lastUiScaleFactor))
                {
                    if (_lastUiScaleFactor > 0f)
                    {
                        ApplyUiScaleChange(_lastUiScaleFactor, uiScale);
                    }
                    _lastUiScaleFactor = uiScale;
                    windowRect.height = 0;
                    windowRect = UIScale.ClampToGuiScreen(windowRect);
                }

                GUI.skin = HighLogic.Skin;
                UIScale.BeginGUI();
                try
                {
                    GUIStyle windowTitleStyle = new GUIStyle(GUI.skin.window) { fontSize = FontSize + 1 };
                    windowRect = ClickThruBlocker.GUILayoutWindow(884422, windowRect, DrawWindow, Localizer.Format("#SWAOD_Title"), windowTitleStyle);
                    windowRect = UIScale.ClampToGuiScreen(windowRect);
                    if (showBodyFilterPopup && bodyFilterNames.Length > 0)
                    {
                        GUIStyle popupTitleStyle = new GUIStyle(GUI.skin.window) { fontSize = FontSize + 1, fontStyle = FontStyle.Bold };
                        bodyFilterPopupRect = ClickThruBlocker.GUILayoutWindow(884423, bodyFilterPopupRect, DrawBodyFilterPopup, Localizer.Format("#SWAOD_SelectBody"), popupTitleStyle);
                        bodyFilterPopupRect = UIScale.ClampToGuiScreen(bodyFilterPopupRect);
                    }
                }
                finally
                {
                    UIScale.EndGUI();
                }
            }
        }

        private void ApplyUiScaleChange(float oldScale, float newScale)
        {
            if (oldScale <= 0f || newScale <= 0f)
                return;

            windowRect = ScaleWindowPosition(windowRect, oldScale, newScale);
            bodyFilterPopupRect = ScaleWindowPosition(bodyFilterPopupRect, oldScale, newScale);
        }

        private static Rect ScaleWindowPosition(Rect rect, float oldScale, float newScale)
        {
            if (rect.width <= 0f && rect.height <= 0f)
                return rect;

            float ratio = oldScale / newScale;
            rect.x *= ratio;
            rect.y *= ratio;
            return UIScale.ClampToGuiScreen(rect);
        }

        private void DrawWindow(int windowID)
        {
            GUILayout.BeginVertical();

            if (_cachedFontSizeForStyles != FontSize)
            {
                _cachedFontSizeForStyles = FontSize;
                _cachedBold = CreateSingleLineStyle(GUI.skin.label, FontSize + 1, TextAnchor.MiddleLeft, FontStyle.Bold);
                _cachedNormal = CreateSingleLineStyle(GUI.skin.label, FontSize);
                _cachedRow = CreateSingleLineStyle(GUI.skin.label, FontSize, TextAnchor.MiddleLeft);
                _cachedRed = CreateSingleLineStyle(GUI.skin.label, FontSize, TextAnchor.MiddleLeft, FontStyle.Bold);
                _cachedRed.normal.textColor = new Color(1f, 0.4f, 0.4f);
                _cachedGreen = CreateSingleLineStyle(GUI.skin.label, FontSize, TextAnchor.MiddleLeft, FontStyle.Bold);
                _cachedGreen.normal.textColor = new Color(0.4f, 1f, 0.4f);
                _cachedYellow = CreateSingleLineStyle(GUI.skin.label, FontSize, TextAnchor.MiddleLeft);
                _cachedYellow.normal.textColor = new Color(1f, 1f, 0.4f);
                _cachedSubHeader = CreateSingleLineStyle(GUI.skin.label, FontSize - 1, TextAnchor.MiddleLeft, FontStyle.Bold);
                _cachedVesselNameStyle = CreateSingleLineStyle(GUI.skin.label, FontSize, TextAnchor.MiddleLeft, FontStyle.Bold);
                _cachedVesselNameStyle.richText = true;
                _cachedButton = CreateSingleLineStyle(GUI.skin.button, FontSize, TextAnchor.MiddleCenter, FontStyle.Bold);
                _cachedButton.padding = new RectOffset(GUI.skin.button.padding.left, GUI.skin.button.padding.right, 6, 6);
                _cachedBoxStyle = new GUIStyle(GUI.skin.box)
                {
                    fontSize = FontSize,
                    padding = new RectOffset(8, 8, 6, 6),
                    stretchWidth = true
                };
            }

            GUIStyle bold = _cachedBold;
            GUIStyle normal = _cachedNormal;
            GUIStyle row = _cachedRow;
            GUIStyle red = _cachedRed;
            GUIStyle green = _cachedGreen;
            GUIStyle yellow = _cachedYellow;
            GUIStyle subHeader = _cachedSubHeader;
            GUIStyle vesselNameStyle = _cachedVesselNameStyle;
            string destroyLabel = Localizer.Format("#SWAOD_DestroyNow");
            float destroyButtonWidth = _cachedButton.CalcSize(new GUIContent(destroyLabel)).x + 12f;
            float rowH = ButtonHeight;
            float requiredWidth = GetWindowWidthForFontSize(FontSize);
            if (windowRect.width < requiredWidth - 0.5f)
                ApplyWindowWidth(requiredWidth);
            bool debugModeEnabled = IsDebugModeEnabled;

            if (debugModeEnabled)
            {
                GUILayout.BeginHorizontal(GUILayout.Height(rowH));
                GUILayout.Label(Localizer.Format("#SWAOD_Config"), bold, GUILayout.Height(rowH));
                GUILayout.FlexibleSpace();
                float showConfigWidth = _cachedButton.CalcSize(new GUIContent(Localizer.Format("#SWAOD_ShowConfig"))).x + BODY_BUTTON_PADDING;
                if (GUILayout.Button(showSettings ? Localizer.Format("#SWAOD_HideConfig") : Localizer.Format("#SWAOD_ShowConfig"), _cachedButton, GUILayout.Width(showConfigWidth), GUILayout.Height(rowH)))
                {
                    showSettings = !showSettings;
                    if (!showSettings)
                        windowRect.height = 0;
                }
                GUILayout.EndHorizontal();

                if (showSettings)
                {
                    GUILayout.BeginVertical(_cachedBoxStyle);
                    if (GUILayout.Button(Localizer.Format("#SWAOD_DumpLogs"), _cachedButton, GUILayout.Height(rowH)))
                        DumpAtmosphereLogs();

                    GUILayout.Space(5);

                    // Storm Settings
                    GUILayout.BeginHorizontal(GUILayout.Height(rowH));
                    string stormStatus = KerbalismIntegration.IsAvailable
                        ? (stormDecayRate > 0 ? Localizer.Format("#SWAOD_Enabled") : Localizer.Format("#SWAOD_Disabled"))
                        : Localizer.Format("#SWAOD_NotAvailable");
                    GUILayout.Label(Localizer.Format("#SWAOD_StormDecay") + stormStatus, row, GUILayout.ExpandWidth(true), GUILayout.Height(rowH));
                    GUILayout.Label(Localizer.Format("#SWAOD_BaseRate", stormDecayRate.ToString("E2")), yellow, GUILayout.Height(rowH));
                    GUILayout.EndHorizontal();

                    // Natural Decay Settings
                    GUILayout.BeginHorizontal(GUILayout.Height(rowH));
                    GUILayout.Label(Localizer.Format("#SWAOD_NaturalDecay") + (naturalDecayEnabled ? Localizer.Format("#SWAOD_Enabled") : Localizer.Format("#SWAOD_Disabled")), row, GUILayout.ExpandWidth(true), GUILayout.Height(rowH));
                    GUILayout.FlexibleSpace();
                    GUILayout.Label(Localizer.Format("#SWAOD_Multiplier", naturalDecayMultiplier.ToString("F2")), yellow, GUILayout.Height(rowH));
                    GUILayout.EndHorizontal();

                    // Decay Limits (Based on current active vessel's body or Kerbin if none)
                    Vessel activeV = FlightGlobals.ActiveVessel;
                    if (activeV != null && activeV.mainBody.atmosphere)
                    {
                        double atmDepth = activeV.mainBody.atmosphereDepth;
                        double maxAlt = atmDepth * naturalDecayAltitudeCutoff;

                        double nominalMax = atmDepth * 10.0;
                        GUILayout.Label(Localizer.Format("#SWAOD_NominalRange", (atmDepth / 1000).ToString("F0"), (nominalMax / 1000).ToString("F0")), subHeader, GUILayout.ExpandWidth(true));
                        GUILayout.Label(Localizer.Format("#SWAOD_ActualRange", (atmDepth / 1000).ToString("F0"), (maxAlt / 1000).ToString("F0")), subHeader, GUILayout.ExpandWidth(true));
                    }
                    else
                    {
                        GUILayout.Label(Localizer.Format("#SWAOD_NoAtmosphere"), normal);
                    }

                    GUILayout.Space(5);
                    debugForceStorm = GUILayout.Toggle(debugForceStorm, Localizer.Format("#SWAOD_ForceStorm"), _cachedButton, GUILayout.Height(rowH));
                    GUILayout.EndVertical();
                }
            }

            GUILayout.BeginHorizontal();
            if (bodyFilterNames.Length > 0)
            {
                string allBodiesText = Localizer.Format("#SWAOD_Filter_AllBodies");
                string displayText = bodyFilterNames[currentBodyFilterIndex];
                float buttonWidth = ButtonWidth(displayText, ButtonWidth(allBodiesText, BODY_POPUP_DEFAULT_WIDTH * 0.5f));
                float labelH = row.CalcHeight(new GUIContent(Localizer.Format("#SWAOD_Filter_Body")), 200f);
                float btnH = _cachedButton.CalcSize(new GUIContent(allBodiesText)).y;
                float bodyRowH = Math.Max(rowH, Math.Max(labelH, btnH));
                GUILayout.Label(Localizer.Format("#SWAOD_Filter_Body"), row, GUILayout.Height(bodyRowH));
                string truncatedDisplay = TruncateToWidth(displayText, _cachedButton, buttonWidth - BODY_BUTTON_PADDING);
                if (GUILayout.Button(truncatedDisplay, _cachedButton, GUILayout.Width(buttonWidth), GUILayout.Height(bodyRowH)))
                {
                    showBodyFilterPopup = true;
                    Vector2 screen = UIScale.GuiScreenSize();
                    float logicalW = screen.x;
                    float logicalH = screen.y;
                    float closeBtnHeight = _cachedButton.CalcSize(new GUIContent(Localizer.Format("#SWAOD_Close"))).y + 8f;
                    float popupHeight = 24f + 6f + BODY_POPUP_LIST_HEIGHT + 8f + closeBtnHeight;
                    bodyFilterPopupRect = new Rect((logicalW - BODY_POPUP_DEFAULT_WIDTH) * 0.5f, (logicalH - popupHeight) * 0.5f, BODY_POPUP_DEFAULT_WIDTH, popupHeight);
                    bodyFilterPopupScrollPosition = Vector2.zero;
                }
            }
            else
            {
                GUILayout.Label(Localizer.Format("#SWAOD_Filter_Body"), row);
            }
            GUILayout.EndHorizontal();
            GUILayout.Space(6);

            // --- Filter Buttons ---
            DrawFilterButtons(rowH);
            
            DrawDebrisFilterButtons(rowH);

            // --- Vessel List ---
            GUILayout.Space(5);
            GUILayout.Label(Localizer.Format("#SWAOD_TrackedVessels", cachedVisibleVessels.Count), bold);

            GUIStyle scrollStyle = new GUIStyle(GUI.skin.scrollView);
            scrollStyle.padding.left = 0; 
            scrollStyle.padding.right = 0;
            
            float vesselListHeight = VESSEL_LIST_HEIGHT;
            float orbitColumnWidth = GetOrbitColumnWidth();
            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, false, GUIStyle.none, GUI.skin.verticalScrollbar, scrollStyle, GUILayout.Height(vesselListHeight));

            for (int i = 0; i < cachedVisibleVessels.Count; i++)
            {
                VesselDisplayState state = cachedVisibleVessels[i];
                Vessel v = state.Vessel;
                bool isNatural = state.IsNatural;
                bool stormInRange = state.StormInRange;

                GUILayout.BeginVertical(_cachedBoxStyle);
                GUILayout.BeginHorizontal(GUILayout.Height(rowH));
                GUILayout.Label(state.VesselNameRich, vesselNameStyle, GUILayout.ExpandWidth(true), GUILayout.Height(rowH));
                GUILayout.FlexibleSpace();
                GUILayout.Label(state.BodyName, yellow, GUILayout.Height(rowH));
                if (state.ShowDestroyButton)
                {
                    GUILayout.Space(6);
                    if (GUILayout.Button(destroyLabel, _cachedButton, GUILayout.Width(destroyButtonWidth), GUILayout.Height(rowH)))
                        DestroyLoadedVessel(v);
                }
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                GUILayout.BeginVertical(GUILayout.ExpandWidth(true));
                GUILayout.BeginHorizontal(GUILayout.Height(rowH));
                GUILayout.Label(state.PeAltText, row, GUILayout.MinWidth(orbitColumnWidth), GUILayout.Height(rowH));
                GUILayout.Space(28f);
                GUILayout.Label(state.PeTimeText, row, GUILayout.Height(rowH));
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal(GUILayout.Height(rowH));
                GUILayout.Label(state.ApAltText, row, GUILayout.MinWidth(orbitColumnWidth), GUILayout.Height(rowH));
                GUILayout.Space(28f);
                GUILayout.Label(state.ApTimeText, row, GUILayout.Height(rowH));
                GUILayout.EndHorizontal();
                GUILayout.BeginHorizontal(GUILayout.Height(rowH));
                GUILayout.Label(state.IncText, row, GUILayout.ExpandWidth(true), GUILayout.Height(rowH));
                GUILayout.Label(state.EccText, row, GUILayout.ExpandWidth(true), GUILayout.Height(rowH));
                GUILayout.EndHorizontal();
                if (debugModeEnabled && stormInRange && !string.IsNullOrEmpty(state.StormRateText))
                    GUILayout.Label(state.StormRateText, red, GUILayout.Height(rowH));
                GUILayout.EndVertical();
                GUILayout.EndHorizontal();

                GUILayout.Space(4);

                GUILayout.BeginHorizontal(GUILayout.Height(rowH));
                string statusText = state.StatusText;
                GUIStyle statusStyle = green;
                if (!vesselDecayDisabled.Contains(v.id))
                {
                    if (stormInRange && isNatural) { statusStyle = red; }
                    else if (stormInRange) { statusStyle = red; }
                    else if (isNatural) { statusStyle = yellow; }
                }
                GUILayout.Label(statusText, statusStyle, GUILayout.Height(rowH));
                if (debugModeEnabled)
                {
                    GUILayout.FlexibleSpace();
                    bool disabled = vesselDecayDisabled.Contains(v.id);
                    bool newDisabled = GUILayout.Toggle(disabled, Localizer.Format("#SWAOD_DisableDecayToggle"), _cachedButton, GUILayout.Height(rowH));
                    if (newDisabled != disabled)
                    {
                        if (newDisabled) vesselDecayDisabled.Add(v.id);
                        else vesselDecayDisabled.Remove(v.id);
                    }
                }
                GUILayout.EndHorizontal();
                GUILayout.EndVertical();

                if (i < cachedVisibleVessels.Count - 1)
                    GUILayout.Space(VESSEL_ENTRY_SPACING);
            }

            GUILayout.EndScrollView();
            GUILayout.EndVertical();

            GUI.DragWindow();
        }

        private static GUIStyle CreateSingleLineStyle(GUIStyle template, int fontSize, TextAnchor alignment = TextAnchor.MiddleLeft, FontStyle fontStyle = FontStyle.Normal)
        {
            return new GUIStyle(template)
            {
                fontSize = fontSize,
                fontStyle = fontStyle,
                alignment = alignment,
                wordWrap = false,
                clipping = TextClipping.Clip,
                padding = new RectOffset(0, 0, 3, 3)
            };
        }

        private static float ButtonHeight => FontSize + 16f;

        private static bool IsDebugModeEnabled => SwaodParameters.IsDebugModeEnabled;

        private void DrawFilterButtons(float rowH)
        {
            FilterMode previousFilter = currentFilter;
            string all = Localizer.Format("#SWAOD_Filter_All");
            string stable = Localizer.Format("#SWAOD_Filter_Stable");
            string natural = Localizer.Format("#SWAOD_Filter_Natural");
            string storm = Localizer.Format("#SWAOD_Filter_Storm");

            GUILayout.BeginHorizontal(GUILayout.Height(rowH), GUILayout.ExpandWidth(true));
            DrawStretchFilterButton(FilterMode.All, all, rowH);
            DrawStretchFilterButton(FilterMode.Stable, stable, rowH);
            DrawStretchFilterButton(FilterMode.Natural, natural, rowH);
            DrawStretchFilterButton(FilterMode.Storm, storm, rowH);
            GUILayout.EndHorizontal();

            if (previousFilter != currentFilter)
                uiCacheDirty = true;
        }

        private void DrawStretchFilterButton(FilterMode mode, string label, float rowH)
        {
            if (GUILayout.Toggle(currentFilter == mode, label, _cachedButton, GUILayout.ExpandWidth(true), GUILayout.Height(rowH)))
                currentFilter = mode;
        }

        private void DrawDebrisFilterButtons(float rowH)
        {
            DebrisFilterMode previousDebrisFilter = currentDebrisFilter;
            string all = Localizer.Format("#SWAOD_Filter_Debris_All");
            string only = Localizer.Format("#SWAOD_Filter_Debris_Only");
            string exclude = Localizer.Format("#SWAOD_Filter_Debris_Exclude");

            GUILayout.BeginHorizontal(GUILayout.Height(rowH), GUILayout.ExpandWidth(true));
            DrawStretchDebrisFilterButton(DebrisFilterMode.All, all, rowH);
            DrawStretchDebrisFilterButton(DebrisFilterMode.OnlyDebris, only, rowH);
            DrawStretchDebrisFilterButton(DebrisFilterMode.ExcludeDebris, exclude, rowH);
            GUILayout.EndHorizontal();

            if (previousDebrisFilter != currentDebrisFilter)
                uiCacheDirty = true;
        }

        private void DrawStretchDebrisFilterButton(DebrisFilterMode mode, string label, float rowH)
        {
            if (GUILayout.Toggle(currentDebrisFilter == mode, label, _cachedButton, GUILayout.ExpandWidth(true), GUILayout.Height(rowH)))
                currentDebrisFilter = mode;
        }

        private float ButtonWidth(string label, float minWidth)
        {
            float width;
            if (_cachedButton != null)
            {
                width = _cachedButton.CalcSize(new GUIContent(label ?? string.Empty)).x + 28f;
            }
            else
            {
                // Update() can ask for window width before IMGUI is active; avoid GUI.skin there.
                width = (label ?? string.Empty).Length * FontSize * 0.75f + 28f;
            }
            return Mathf.Ceil(Mathf.Max(minWidth, width));
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

        /// <summary>
        /// The celestial body list does not change between game sessions (adding celestial bodies requires restarting the game), 
        /// so it is only initialized once when needed for the first time.
        /// </summary>
        private void UpdateBodyFilterCache()
        {
            if (bodyFilterCacheInitialized) return;

            List<CelestialBody> bodies = FlightGlobals.Bodies;
            if (bodies == null || bodies.Count == 0) return;

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
            bodyFilterCacheInitialized = true;
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

            bool stormActive = KerbalismIntegration.IsStormInProgress(v);
            state.IsStorming = stormActive;
            state.IsNatural = false;
            state.StormInRange = false;
            state.StormDragMultiplier = 0;
            state.CurrentStormRate = 0;

            double periapsisAlt = v.orbit.PeA;
            double mass = v.GetTotalMass();
            if (mass <= 0.001) mass = 0.1;
            if (naturalDecayEnabled && v.mainBody.atmosphere && periapsisAlt < v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff)
            {
                AtmosphericDecayModel.NaturalDecayRates naturalRates = AtmosphericDecayModel.EstimateNaturalDecayRates(
                    v.mainBody, v.orbit, mass, GetDecaySettings());
                if (Math.Abs(naturalRates.DaDt) > 1e-12 ||
                    Math.Abs(naturalRates.DeDt) > 1e-12 ||
                    Math.Abs(naturalRates.PeriapsisDaDt) > 1e-12 ||
                    Math.Abs(naturalRates.ApoapsisDaDt) > 1e-12)
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
                state.StormDragMultiplier = GetStormDragMultiplier(
                    v,
                    v.orbit.semiMajorAxis,
                    v.orbit.eccentricity,
                    mass,
                    GetDecaySettings(),
                    true);
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
            state.PeTimeText = Localizer.Format("#SWAOD_DecayTime", GetDecayTimeDisplay(v, false, state.IsStorming, state.IsForced, state.StormDragMultiplier, cachedDecayTimesPe, lastDecayCalcTimePe));
            state.ApTimeText = Localizer.Format("#SWAOD_DecayTime", GetDecayTimeDisplay(v, true, state.IsStorming, state.IsForced, state.StormDragMultiplier, cachedDecayTimesAp, lastDecayCalcTimeAp));
            state.IncText = Localizer.Format("#SWAOD_Inc", v.orbit.inclination.ToString("F2"));
            state.EccText = Localizer.Format("#SWAOD_Ecc", v.orbit.eccentricity.ToString("F3"));

            string statusText;
            if (vesselDecayDisabled.Contains(v.id))
                statusText = Localizer.Format("#SWAOD_Status_Stable");
            else if (state.StormInRange && state.IsNatural)
                statusText = Localizer.Format("#SWAOD_Status_StormPlus");
            else if (state.StormInRange)
                statusText = Localizer.Format("#SWAOD_Status_StormDecay");
            else if (state.IsNatural)
                statusText = Localizer.Format("#SWAOD_Status_NaturalDecay");
            else
                statusText = Localizer.Format("#SWAOD_Status_Stable");
            state.StatusText = statusText;
            state.StormRateText = state.StormInRange ? Localizer.Format("#SWAOD_StormRate_Debug", state.CurrentStormRate.ToString("E2")) : string.Empty;
            state.ShowDestroyButton = v.loaded && v.vesselType != VesselType.Debris && v.mainBody.atmosphere && v.orbit.PeA < v.mainBody.atmosphereDepth;

            return true;
        }

        private float EstimateTextWidth(string text)
        {
            if (_cachedRow != null)
                return _cachedRow.CalcSize(new GUIContent(text ?? string.Empty)).x;
            return (text ?? string.Empty).Length * FontSize * 0.75f;
        }

        private float GetOrbitColumnWidth()
        {
            float pe = EstimateTextWidth(Localizer.Format("#SWAOD_PeAlt", "999.999 km"));
            float ap = EstimateTextWidth(Localizer.Format("#SWAOD_ApAlt", "999.999 km"));
            return Mathf.Ceil(Mathf.Max(pe, ap, 200f));
        }

        private float GetVesselEntryMinWidth()
        {
            const float columnGap = 28f;
            const float boxPadding = 16f;
            const float windowPadding = 24f;
            string timeSample = Localizer.Format("#SWAOD_DecayTime", Localizer.Format("#SWAOD_Time_DaysHours", "99", "99"));
            float timeWidth = EstimateTextWidth(timeSample);
            return GetOrbitColumnWidth() + columnGap + timeWidth + boxPadding + windowPadding;
        }

        private float GetWindowWidthForFontSize(int size)
        {
            float filterWidth =
                ButtonWidth(Localizer.Format("#SWAOD_Filter_All"), 92f) +
                ButtonWidth(Localizer.Format("#SWAOD_Filter_Stable"), 118f) +
                ButtonWidth(Localizer.Format("#SWAOD_Filter_Natural"), 118f) +
                ButtonWidth(Localizer.Format("#SWAOD_Filter_Storm"), 132f) +
                32f;
            float debrisWidth =
                ButtonWidth(Localizer.Format("#SWAOD_Filter_Debris_All"), 130f) +
                ButtonWidth(Localizer.Format("#SWAOD_Filter_Debris_Only"), 150f) +
                ButtonWidth(Localizer.Format("#SWAOD_Filter_Debris_Exclude"), 150f) +
                24f;
            return Mathf.Max(BaseWindowWidth, filterWidth, debrisWidth, GetVesselEntryMinWidth());
        }

        private void ApplyWindowWidth(float newWidth)
        {
            float previousWidth = windowRect.width > 0f ? windowRect.width : BaseWindowWidth;
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
            float maxItemWidth = BODY_POPUP_MAX_WIDTH - 24f;
            float contentWidth = 0f;
            for (int i = 0; i < bodyFilterNames.Length; i++)
            {
                Vector2 sz = _cachedButton.CalcSize(new GUIContent(bodyFilterNames[i]));
                if (sz.x > contentWidth) contentWidth = Math.Min(sz.x, maxItemWidth);
            }
            float scrollWidth = Math.Max(contentWidth + 10f, BODY_POPUP_DEFAULT_WIDTH);
            scrollWidth = Math.Min(scrollWidth, BODY_POPUP_MAX_WIDTH);

            GUILayout.Space(6);
            bodyFilterPopupScrollPosition = GUILayout.BeginScrollView(bodyFilterPopupScrollPosition, false, true, GUILayout.Width(scrollWidth), GUILayout.Height(BODY_POPUP_LIST_HEIGHT));
            for (int i = 0; i < bodyFilterNames.Length; i++)
            {
                string name = bodyFilterNames[i];
                string displayName = TruncateToWidth(name, _cachedButton, maxItemWidth);
                if (GUILayout.Button(displayName, _cachedButton, GUILayout.Height(ButtonHeight)))
                {
                    currentBodyFilterIndex = i;
                    showBodyFilterPopup = false;
                    uiCacheDirty = true;
                }
            }
            GUILayout.EndScrollView();
            GUILayout.Space(8);
            if (GUILayout.Button(Localizer.Format("#SWAOD_Close"), _cachedButton, GUILayout.Height(ButtonHeight)))
            {
                showBodyFilterPopup = false;
            }
            GUI.DragWindow();
        }
    }
}
