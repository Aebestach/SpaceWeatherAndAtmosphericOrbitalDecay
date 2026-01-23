# Space Weather And Atmospheric Orbital Decay

## Introduction
Space Weather And Atmospheric Orbital Decay (SWAOD) is a Kerbal Space Program (KSP) mod that introduces orbital decay mechanics. It simulates the effect of atmospheric drag on vessels (both loaded and unloaded) and integrates with the Kerbalism mod to simulate intensified orbital decay effects during solar storms.

## Features
- **Natural Orbital Decay**: Simulates atmospheric drag in the upper atmosphere/exosphere for unloaded vessels (stock KSP typically ignores drag here, causing debris to remain in orbit indefinitely).
- **Solar Storm Effects**: (Requires Kerbalism) Significantly increases orbital decay rates during solar storms, simulating atmospheric expansion caused by solar activity (players can configure whether airless planets are affected by solar storm orbital decay in `Settings.cfg`).
- **Realistic Physics Simulation**: Drag is calculated based on the vessel's estimated Area-to-Mass Ratio and atmospheric density.
- **In-Game UI**: Provides a control panel to monitor all tracked vessels, view their decay status, and estimated reentry time.

## Dependencies
- **Kerbalism** (Optional: This mod is required for solar storm decay features. If Kerbalism is not installed, this mod functions as a standalone atmospheric orbital decay mod.)

## Installation
1. Download the latest version.
2. Extract the `GameData` folder into your KSP installation directory (e.g., `C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\`).
3. **Important**:
   - If you **use Kerbalism**: Use the default installation.
   - If you **do not use Kerbalism**: Replace the `.dll` file in `GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Plugin/` with the one from the `Extra/Non-Kerbalism/` directory.

## Usage
- **Open UI**: Press `Mod + F11` (usually `Alt + F11` on Windows, `RightShift + F11` on Linux) to open the control window.
- **Monitor**: The window lists all orbiting vessels, their altitudes, and current decay status.
- **Filter**: Use the buttons at the bottom to filter the vessel list by "Stable", "Natural Decay", or "Storm Decay".
- **Warnings**: You will receive notifications when a vessel enters a decay zone, its orbit becomes too low, or reentry is imminent.

## Configuration
Configuration can be done by editing the `GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Config/Settings.cfg` file.
