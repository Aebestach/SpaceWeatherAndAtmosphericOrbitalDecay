# Space Weather And Atmospheric Orbital Decay

<div align="center">

<img src="https://imgur.com/2bk9Zad.jpg" alt="Banner" width="600" />

[![License](https://img.shields.io/github/license/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay)](LICENSE)
[![Release](https://img.shields.io/github/v/release/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay)](https://github.com/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay/releases)

[English](README.md) | [中文](README_CN.md)

</div>

---

## 📖 Introduction

**Space Weather And Atmospheric Orbital Decay (SWAOD)** is a mod for **Kerbal Space Program (KSP)** that introduces realistic orbital decay mechanics.

In the stock game, vessels in a vacuum never experience drag. This mod changes that by simulating continuous atmospheric drag on vessels in the upper atmosphere and integrating with **Kerbalism** to simulate severe orbital decay caused by atmospheric expansion during solar storms.

<div align="center">
    <img src="https://imgur.com/4oK8ftS.jpg" alt="UI Screenshot" width="600" />
</div>

## ✨ Features

*   **🪐 Natural Orbital Decay**
    *   Simulates atmospheric drag for unloaded and loaded vessels in the upper atmosphere/exosphere.
    *   Effectively cleans up orbital debris, preventing low-orbit junk accumulation.
    *   **Realistic Physics**: Calculates drag based on the vessel's estimated Area-to-Mass Ratio and real-time atmospheric density.

*   **☀️ Solar Storm Effects** (Requires Kerbalism)
    *   Drastically increases orbital decay rates during solar storms, simulating atmospheric expansion caused by solar activity.
    *   **Configurable**: Toggle whether bodies without atmospheres (e.g., Mun) are affected by storm decay in `Settings.cfg` (Disabled by default).

*   **📊 Real-time Monitoring UI**
    *   Provides a control panel to monitor the status of all tracked vessels.
    *   View orbital altitude, decay status, and **estimated re-entry time** in real-time.
    *   Filter vessel lists by status.

*   **⚠️ Smart Warning System**
    *   Sends notifications when vessels enter the decay zone, reach critically low orbits, or are about to re-enter.

## 🛠️ Dependencies

*   **Kerbalism** (Optional)
    *   *Recommended*: To experience the full solar storm decay features.
    *   *Without Kerbalism*: The mod functions as a standalone "Natural Atmospheric Decay" mod.

## 📥 Installation

1.  Download the [Latest Release](https://github.com/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay/releases).
2.  Extract the `GameData` folder into your KSP installation directory:
    *   `Kerbal Space Program/GameData/SpaceWeatherAndAtmosphericOrbitalDecay/`
3.  **⚠️ Important**:
    *   **Using Kerbalism**: Default installation works out of the box.
    *   **Not using Kerbalism**: Replace the `.dll` file in `GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Plugin/` with the one found in `Extra/Non-Kerbalism/`.

## 🎮 Usage Guide

### Shortcuts
*   **Toggle UI**: `Alt + F11`

### UI Functions
*   **Monitor List**: Lists all orbiting vessels and their current status.
*   **Filters**:
    *   `Stable`
    *   `Natural` (Natural Decay)
    *   `Storm` (Storm Decay)
*   **Config Panel**: Click "Show Config" to adjust UI scale, font size, view debug info, etc. in-game.

## ⚙️ Configuration

Besides the in-game UI, advanced configuration can be done by editing:
`GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Config/Settings.cfg`

| Setting | Description | Default |
| :--- | :--- | :--- |
| `stormDecayRate` | Base decay rate during solar storms | `1.5e-7` |
| `stormDistanceScaling` | Does storm intensity scale with distance from Sun | `true` |
| `applyStormDecayToNoAtmosphereBody` | Apply storm decay to bodies without atmosphere | `false` |
| `naturalDecayEnabled` | Enable natural atmospheric decay | `true` |
| `naturalDecayMultiplier` | Multiplier for natural decay force | `0.5` |
| `naturalDecayAltitudeCutoff` | Max altitude multiplier for natural decay (Relative to Atmo Height) | `10.0` |
| `warningEnabled` | Enable low orbit warnings | `true` |
| `warningThreshold` | Low orbit warning threshold (Periapsis < AtmoHeight * (1.0 + Threshold)) | `0.2` |

## 🤝 Credits

*   Special thanks to **Gemini 3 Pro** for assisting in the development of this mod.
