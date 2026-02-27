using System;
using System.Collections.Generic;
using UnityEngine;
using KSP.Localization;
using ClickThroughFix;

#if KERBALISM
using static KERBALISM.API;
#endif

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
            public double EffectiveStormRate;
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
                GUIStyle windowTitleStyle = new GUIStyle(GUI.skin.window) { fontSize = fontSize + 1 };
                windowRect = ClickThruBlocker.GUILayoutWindow(884422, windowRect, DrawWindow, Localizer.Format("#SWAOD_Title"), windowTitleStyle);
                if (showBodyFilterPopup && bodyFilterNames.Length > 0)
                {
                    GUIStyle popupTitleStyle = new GUIStyle(GUI.skin.window) { fontSize = fontSize + 1, fontStyle = FontStyle.Bold };
                    bodyFilterPopupRect = ClickThruBlocker.GUILayoutWindow(884423, bodyFilterPopupRect, DrawBodyFilterPopup, Localizer.Format("#SWAOD_SelectBody"), popupTitleStyle);
                }
                GUI.matrix = oldMatrix;
            }
        }

        private void DrawWindow(int windowID)
        {
            GUILayout.BeginVertical();

            if (_cachedFontSizeForStyles != fontSize)
            {
                _cachedFontSizeForStyles = fontSize;
                _cachedBold = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = fontSize + 1 };
                _cachedNormal = new GUIStyle(GUI.skin.label) { fontSize = fontSize };
                _cachedRed = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 0.4f, 0.4f) }, fontSize = fontSize, fontStyle = FontStyle.Bold };
                _cachedGreen = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(0.4f, 1f, 0.4f) }, fontSize = fontSize, fontStyle = FontStyle.Bold };
                _cachedYellow = new GUIStyle(GUI.skin.label) { normal = { textColor = new Color(1f, 1f, 0.4f) }, fontSize = fontSize };
                _cachedSubHeader = new GUIStyle(GUI.skin.label) { fontStyle = FontStyle.Bold, fontSize = fontSize - 1, alignment = TextAnchor.MiddleLeft };
                _cachedVesselNameStyle = new GUIStyle(GUI.skin.label) { richText = true, fontSize = fontSize, fontStyle = FontStyle.Bold };
                _cachedButton = new GUIStyle(GUI.skin.button) { fontSize = fontSize, alignment = TextAnchor.MiddleCenter };
            }

            GUIStyle bold = _cachedBold;
            GUIStyle normal = _cachedNormal;
            GUIStyle red = _cachedRed;
            GUIStyle green = _cachedGreen;
            GUIStyle yellow = _cachedYellow;
            GUIStyle subHeader = _cachedSubHeader;
            GUIStyle vesselNameStyle = _cachedVesselNameStyle;
            string destroyLabel = Localizer.Format("#SWAOD_DestroyNow");
            float destroyButtonWidth = _cachedButton.CalcSize(new GUIContent(destroyLabel)).x + 12f;

            GUILayout.BeginHorizontal();
            GUILayout.Label(Localizer.Format("#SWAOD_Config"), bold);
            GUILayout.FlexibleSpace();
            float showConfigWidth = _cachedButton.CalcSize(new GUIContent(Localizer.Format("#SWAOD_ShowConfig"))).x + BODY_BUTTON_PADDING;
            if (GUILayout.Button(showSettings ? Localizer.Format("#SWAOD_HideConfig") : Localizer.Format("#SWAOD_ShowConfig"), _cachedButton, GUILayout.Width(showConfigWidth)))
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
                        
                        windowRect.width = GetWindowWidthForFontSize(fontSize);
                        windowRect.height = 0;
                    }
                    
                    GUILayout.Label(Localizer.Format("#SWAOD_FontSize", fontSize), subHeader);
                    int oldFontSize = fontSize;
                    fontSize = (int)GUILayout.HorizontalSlider((float)fontSize, 10f, 20f);
                    if (oldFontSize != fontSize)
                    {
                        ApplyWindowWidth(GetWindowWidthForFontSize(fontSize));
                        windowRect.height = 0;
                    }

                    GUILayout.Space(5);

                    // Key Binding UI
                    GUILayout.BeginHorizontal();
                    string shortcutLabel = Localizer.Format("#SWAOD_Shortcut", toggleKey.ToString());
                    float shortcutLabelH = subHeader.CalcHeight(new GUIContent(shortcutLabel), 200f);
                    float shortcutBtnH = _cachedButton.CalcSize(new GUIContent(Localizer.Format("#SWAOD_ChangeKey"))).y;
                    float shortcutRowH = Math.Max(shortcutLabelH, shortcutBtnH);
                    GUILayout.Label(shortcutLabel, subHeader, GUILayout.Height(shortcutRowH));
                    if (GUILayout.Button(isRebinding ? Localizer.Format("#SWAOD_PressKey") : Localizer.Format("#SWAOD_ChangeKey"), _cachedButton, GUILayout.Width(100), GUILayout.Height(shortcutRowH)))
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
                    if (GUILayout.Button(Localizer.Format("#SWAOD_Reset"), _cachedButton))
                    {
                        uiScale = 1.0f;
                        fontSize = 13;
                        toggleKey = KeyCode.Q;
                        windowRect.width = GetWindowWidthForFontSize(fontSize);
                        windowRect.height = 0;
                    }
                    
                    if (GUILayout.Button(Localizer.Format("#SWAOD_SaveUI"), _cachedButton))
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
                        if (GUILayout.Button(Localizer.Format("#SWAOD_DumpLogs"), _cachedButton))
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

            GUILayout.BeginHorizontal();
            if (bodyFilterNames.Length > 0)
            {
                string allBodiesText = Localizer.Format("#SWAOD_Filter_AllBodies");
                float baselineWidth = _cachedButton.CalcSize(new GUIContent(allBodiesText)).x;
                float buttonWidth = baselineWidth + BODY_BUTTON_PADDING;
                float labelH = normal.CalcHeight(new GUIContent(Localizer.Format("#SWAOD_Filter_Body")), 200f);
                float btnH = _cachedButton.CalcSize(new GUIContent(allBodiesText)).y;
                float rowH = Math.Max(labelH, btnH);
                GUILayout.Label(Localizer.Format("#SWAOD_Filter_Body"), normal, GUILayout.Height(rowH));
                string displayText = bodyFilterNames[currentBodyFilterIndex];
                string truncatedDisplay = TruncateToWidth(displayText, _cachedButton, baselineWidth);
                if (GUILayout.Button(truncatedDisplay, _cachedButton, GUILayout.Width(buttonWidth), GUILayout.Height(rowH)))
                {
                    showBodyFilterPopup = true;
                    float logicalW = Screen.width / uiScale;
                    float logicalH = Screen.height / uiScale;
                    float closeBtnHeight = _cachedButton.CalcSize(new GUIContent(Localizer.Format("#SWAOD_Close"))).y + 8f;
                    float popupHeight = 24f + 6f + BODY_POPUP_LIST_HEIGHT + 8f + closeBtnHeight;
                    bodyFilterPopupRect = new Rect((logicalW - BODY_POPUP_DEFAULT_WIDTH) * 0.5f, (logicalH - popupHeight) * 0.5f, BODY_POPUP_DEFAULT_WIDTH, popupHeight);
                    bodyFilterPopupScrollPosition = Vector2.zero;
                }
            }
            else
            {
                GUILayout.Label(Localizer.Format("#SWAOD_Filter_Body"), normal);
            }
            GUILayout.EndHorizontal();
            GUILayout.Space(6);

            // --- Filter Buttons ---
            GUILayout.BeginHorizontal();
            FilterMode previousFilter = currentFilter;
            if (GUILayout.Toggle(currentFilter == FilterMode.All, Localizer.Format("#SWAOD_Filter_All"), _cachedButton)) currentFilter = FilterMode.All;
            if (GUILayout.Toggle(currentFilter == FilterMode.Stable, Localizer.Format("#SWAOD_Filter_Stable"), _cachedButton)) currentFilter = FilterMode.Stable;
            if (GUILayout.Toggle(currentFilter == FilterMode.Natural, Localizer.Format("#SWAOD_Filter_Natural"), _cachedButton)) currentFilter = FilterMode.Natural;
            if (GUILayout.Toggle(currentFilter == FilterMode.Storm, Localizer.Format("#SWAOD_Filter_Storm"), _cachedButton)) currentFilter = FilterMode.Storm;
            if (previousFilter != currentFilter) uiCacheDirty = true;
            GUILayout.EndHorizontal();
            
            GUILayout.BeginHorizontal();
            DebrisFilterMode previousDebrisFilter = currentDebrisFilter;
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.All, Localizer.Format("#SWAOD_Filter_Debris_All"), _cachedButton)) currentDebrisFilter = DebrisFilterMode.All;
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.OnlyDebris, Localizer.Format("#SWAOD_Filter_Debris_Only"), _cachedButton)) currentDebrisFilter = DebrisFilterMode.OnlyDebris;
            if (GUILayout.Toggle(currentDebrisFilter == DebrisFilterMode.ExcludeDebris, Localizer.Format("#SWAOD_Filter_Debris_Exclude"), _cachedButton)) currentDebrisFilter = DebrisFilterMode.ExcludeDebris;
            if (previousDebrisFilter != currentDebrisFilter) uiCacheDirty = true;
            GUILayout.EndHorizontal();

            // --- Vessel List ---
            GUILayout.Space(5);
            GUILayout.Label(Localizer.Format("#SWAOD_TrackedVessels", cachedVisibleVessels.Count), bold);

            GUIStyle scrollStyle = new GUIStyle(GUI.skin.scrollView);
            scrollStyle.padding.left = 0; 
            scrollStyle.padding.right = 0;
            
            float vesselListHeight = 600f;
            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, false, GUI.skin.horizontalScrollbar, GUI.skin.verticalScrollbar, scrollStyle, GUILayout.Height(vesselListHeight));

            foreach (VesselDisplayState state in cachedVisibleVessels)
            {
                Vessel v = state.Vessel;
                bool isNatural = state.IsNatural;
                bool stormInRange = state.StormInRange;

                GUILayout.BeginVertical("box");
                {
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(state.VesselNameRich, vesselNameStyle);
                    GUILayout.FlexibleSpace();
                    GUILayout.Label(state.BodyName, yellow);
                    if (state.ShowDestroyButton)
                    {
                        GUILayout.Space(6);
                        if (GUILayout.Button(destroyLabel, _cachedButton, GUILayout.Width(destroyButtonWidth)))
                        {
                            DestroyLoadedVessel(v);
                        }
                    }
                    GUILayout.EndHorizontal();

                    GUILayout.BeginHorizontal();
                    GUILayout.BeginVertical();
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(state.PeAltText, normal, GUILayout.MinWidth(300f * (fontSize / (float)baseFontSize)));
                    GUILayout.Space(28f * (fontSize / (float)baseFontSize));
                    GUILayout.Label(state.PeTimeText, normal);
                    GUILayout.EndHorizontal();
                    GUILayout.BeginHorizontal();
                    GUILayout.Label(state.ApAltText, normal, GUILayout.MinWidth(300f * (fontSize / (float)baseFontSize)));
                    GUILayout.Space(28f * (fontSize / (float)baseFontSize));
                    GUILayout.Label(state.ApTimeText, normal);
                    GUILayout.EndHorizontal();
                    GUILayout.Label(state.IncText, normal);
                    GUILayout.Label(state.EccText, normal);
                    if (debugMode && stormInRange && !string.IsNullOrEmpty(state.StormRateText))
                    {
                        GUILayout.Label(state.StormRateText, red);
                    }
                    GUILayout.EndVertical();
                    GUILayout.FlexibleSpace();
                    GUILayout.EndHorizontal();

                    GUILayout.Space(4);

                    GUILayout.BeginHorizontal();
                    
                    string statusText = state.StatusText;
                    GUIStyle statusStyle = green;
                    if (!vesselDecayDisabled.Contains(v.id))
                    {
                        if (stormInRange && isNatural) { statusStyle = red; }
                        else if (stormInRange) { statusStyle = red; }
                        else if (isNatural) { statusStyle = yellow; }
                    }
                    GUILayout.Label(statusText, statusStyle);
                    if (debugMode)
                    {
                        GUILayout.FlexibleSpace();
                        bool disabled = vesselDecayDisabled.Contains(v.id);
                        bool newDisabled = GUILayout.Toggle(disabled, Localizer.Format("#SWAOD_DisableDecayToggle"), _cachedButton);
                        if (newDisabled != disabled)
                        {
                            if (newDisabled) vesselDecayDisabled.Add(v.id);
                            else vesselDecayDisabled.Remove(v.id);
                        }
                    }
                    GUILayout.EndHorizontal();
                }
                GUILayout.EndVertical();
            }

            GUILayout.EndScrollView();
            GUILayout.EndVertical();

            GUI.DragWindow();
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

            bool stormActive = false;
#if KERBALISM
            stormActive = StormInProgress(v);
#endif
            state.IsStorming = stormActive;
            state.IsNatural = false;
            state.StormInRange = false;
            state.EffectiveStormRate = 0;
            state.CurrentStormRate = 0;

            double periapsisAlt = v.orbit.PeA;
            if (naturalDecayEnabled && v.mainBody.atmosphere && periapsisAlt < v.mainBody.atmosphereDepth * naturalDecayAltitudeCutoff)
            {
                double dens = GetExosphericDensity(v.mainBody, periapsisAlt);
                if (periapsisAlt < v.mainBody.atmosphereDepth)
                {
                    dens *= 10.0;
                }
                if (dens > 1e-22)
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
                else if (applyStormDecayToNoAtmosphereBody)
                {
                    double maxAlt = v.mainBody.sphereOfInfluence - v.mainBody.Radius;
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
                state.EffectiveStormRate = state.CurrentStormRate;
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
            state.PeTimeText = Localizer.Format("#SWAOD_DecayTime", GetDecayTimeDisplay(v, v.orbit.PeA, state.IsStorming, state.IsForced, state.EffectiveStormRate, cachedDecayTimesPe, lastDecayCalcTimePe));
            state.ApTimeText = Localizer.Format("#SWAOD_DecayTime", GetDecayTimeDisplay(v, v.orbit.ApA, state.IsStorming, state.IsForced, state.EffectiveStormRate, cachedDecayTimesAp, lastDecayCalcTimeAp));
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

        private float GetWindowWidthForFontSize(int size)
        {
            return baseWindowWidth * (size / (float)baseFontSize);
        }

        private void ApplyWindowWidth(float newWidth)
        {
            float previousWidth = windowRect.width > 0f ? windowRect.width : baseWindowWidth;
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
                if (GUILayout.Button(displayName, _cachedButton))
                {
                    currentBodyFilterIndex = i;
                    showBodyFilterPopup = false;
                    uiCacheDirty = true;
                }
            }
            GUILayout.EndScrollView();
            GUILayout.Space(8);
            if (GUILayout.Button(Localizer.Format("#SWAOD_Close"), _cachedButton))
            {
                showBodyFilterPopup = false;
            }
            GUI.DragWindow();
        }
    }
}
