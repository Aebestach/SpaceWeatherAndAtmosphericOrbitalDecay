# Space Weather And Atmospheric Orbital Decay

<div align="center">

<img src="https://imgur.com/2bk9Zad.jpg" alt="Banner"/>

[![License](https://img.shields.io/github/license/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay)](LICENSE)
[![Release](https://img.shields.io/github/v/release/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay)](https://github.com/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay/releases)

[English](README.md) | [中文](README_CN.md)

</div>

---

## Introduction

**Space Weather And Atmospheric Orbital Decay (SWAOD)** is a mod for **Kerbal Space Program (KSP)** that introduces realistic orbital decay mechanics.

In the stock game, vessels in a vacuum never experience drag. This mod changes that by simulating continuous atmospheric drag on vessels in the upper atmosphere and integrating with **Kerbalism** to simulate severe orbital decay caused by atmospheric expansion during solar storms.

<div align="center">
    <img src="https://imgur.com/n6RjwUC.jpg" alt="UI Screenshot" width="600" />
</div>

## Features

*   **Natural Orbital Decay**
    *   Simulates atmospheric drag for unloaded and loaded vessels in the upper atmosphere/exosphere.
    *   Effectively cleans up orbital debris, preventing low-orbit junk accumulation.
    *   **Realistic Physics**: Uses area-to-mass ratio and real-time atmospheric density to derive changes in `a` and `e` from energy and angular-momentum loss. Highly eccentric orbits use adaptive periapsis-arc sampling.
    *   Exosphere density uses exponential scale-height extrapolation; orbit evolution and drag integration follow the physics model above.

*   **Solar Storm Effects** (Requires Kerbalism)
    *   Enabled only when Kerbalism is installed and the vessel is in an active solar storm.
    *   Storm decay is modeled as **extra atmospheric drag** within the unified drag model, simulating atmospheric expansion from solar activity.
    *   Applies only to orbiting vessels around bodies with atmospheres.

*   **Real-time Monitoring UI**
    *   Provide a control panel to monitor the status of on-orbit/sub-orbit vehicles.
    *   Real-time view of apogee/perigee orbit height, attenuation status and **estimated re-entry time of apogee/perigee**
    *   Filter vessel lists by status.

*   **Smart Warning System**
    *   Send a notification when the vehicle first re-enters the atmosphere and automatic vehicle destruction begins.

## Dependencies

*   **Click Through Blocker**
    *   Required for the mod's UI to function.

*   **Kerbalism** (Optional)
    *   *Recommended*: To experience the full solar storm decay features.
    *   *Without Kerbalism*: The mod functions as a standalone "Natural Atmospheric Decay" mod.
    *   Kerbalism is detected automatically at runtime; no need to manually choose or swap DLL versions.

## Compatibility

*   ✅ **Planet Packs**: Theoretically compatible with all planet packs.
*   ❌ **Principia**: Not supported.

## Installation

1.  Download the [Latest Release](https://github.com/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay/releases).
2.  Extract the `GameData` folder into your KSP installation directory:
    *   `Kerbal Space Program/GameData/SpaceWeatherAndAtmosphericOrbitalDecay/`
3.  Ensure **Click Through Blocker** is installed (required).
4.  **Kerbalism** is optional: the mod detects it automatically at runtime. With Kerbalism installed, solar storm decay is enabled; without it, natural atmospheric decay still works with no extra steps.

## Usage Guide

### Shortcuts
*   **Toggle UI**: `Alt + Q`

### UI Functions
*   **Monitor List**: Lists all orbiting vessels and their current status.
*   **Filters**:
    *   `Stable`
    *   `Natural` (Natural Decay)
    *   `Storm` (Storm Decay)
*   **Config Panel**: Click `Show Config` to adjust UI scale, font size, view debug info, etc. in-game.

### Important Notes
*   The re-entry time shown in the UI is an estimate that uses the same drag model as runtime decay. For eccentric orbits, periapsis/apoapsis decay rates are integrated step by step.
*   Long time-warps, changing vessel mass, and the simplified prograde/retrograde drag application used for **loaded** vessels can introduce differences between estimates and the actual orbit.

## Configuration

Besides the in-game UI, advanced configuration can be done by editing:
`GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Config/Settings.cfg`

| Setting | Description | Default |
| :--- | :--- | :--- |
| `stormDecayRate` | Base decay rate during solar storms | `1.5e-7` |
| `stormDistanceScaling` | Does storm intensity scale with distance from Sun | `true` |
| `naturalDecayEnabled` | Enable natural atmospheric decay | `true` |
| `naturalDecayMultiplier` | Multiplier for natural decay force | `1.0` |
| `naturalDecayAltitudeCutoff` | Max altitude multiplier for natural decay (Relative to Atmo Height) | `10.0` |
| `exosphereFitStart` | Start of the upper-atmosphere density fit window (fraction of atmosphere height) | `0.80` |
| `exosphereFitEnd` | End of the upper-atmosphere density fit window (fraction of atmosphere height) | `0.90` |
| `exosphereScaleHeightMin` | Minimum extrapolated scale height (fraction of atmosphere height) | `0.03` |
| `exosphereScaleHeightMax` | Maximum extrapolated scale height (fraction of atmosphere height) | `0.30` |
| `exosphereFitSamples` | Samples used to fit the upper-atmosphere `ln(density)` slope | `8` |
| `orbitAverageSamples` | Samples used for orbit-averaged drag estimates | `24` |
| `warningEnabled` | Enable low orbit warnings | `true` |
| `warningThreshold` | Low orbit warning threshold (Periapsis < AtmoHeight * (1.0 + Threshold)) | `0.2` |
| `reentryDestroySeconds` | Countdown seconds until an unloaded vehicle is destroyed after entering the atmosphere | `60.0` |

## Public API

Other mods can query SWAOD's decay cadence through `SpaceWeatherAndAtmosphericOrbitalDecay.OrbitalDecayApi`:

| Method | Use case |
| :--- | :--- |
| `TryEstimateStationKeepingCadence(Vessel, targetAp, targetPe, tolerance%, out estimate)` | In-flight estimate using the launched vessel's current state; includes storm-enhanced atmospheric drag only when Kerbalism is installed and the vessel is in an active solar storm. |
| `TryEstimateStationKeepingCadenceForOrbit(CelestialBody, targetAp, targetPe, tolerance%, mass, out estimate)` | VAB/editor planning with no `Vessel` instance; uses natural decay at the target orbit only (no storm estimate). |
| `TryEstimateCurrentDecayRates(Vessel, out rates)` | Query the vessel's current instantaneous decay rates; includes storm enhancement only during an active Kerbalism storm. |

All of these APIs return read-only estimates and do not modify vessel orbits. `StationKeepingEstimate` and `CurrentDecayRates` provide seconds to the tolerance band, an effective decay rate, `DaDt`/`DeDt`/`PeriapsisDaDt`/`ApoapsisDaDt`, a storm flag, tolerance drop, and estimated delta-v to restore that drop (`CurrentDecayRates` omits tolerance-related fields). Storm decay is modeled as extra atmospheric drag within the unified drag model. [Orbital Keeper](https://github.com/Aebestach/OrbitalKeeper) uses these APIs for its lifetime and VAB planning estimates.

## Credits

*   Special thanks to **Gemini 3 Pro** for assisting in the development of this mod.
