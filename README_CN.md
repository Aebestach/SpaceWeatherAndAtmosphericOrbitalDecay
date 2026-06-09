# Space Weather And Atmospheric Orbital Decay
# 空间天气与大气轨道衰减

<div align="center">

<img src="https://imgur.com/2bk9Zad.jpg" alt="Banner"/>

[![License](https://img.shields.io/github/license/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay)](LICENSE)
[![Release](https://img.shields.io/github/v/release/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay)](https://github.com/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay/releases)

[English](README.md) | [中文](README_CN.md)

</div>

---

## 简介 | Introduction

**Space Weather And Atmospheric Orbital Decay (SWAOD)** 是一款为 **Kerbal Space Program (KSP)** 设计的模组，旨在引入真实的轨道衰减机制。

原版游戏中，飞船在真空中永远不会受到阻力影响。本模组改变了这一点：它模拟了高层大气对飞船的持续阻力，并与 **Kerbalism** 集成，模拟太阳风暴期间因大气膨胀导致的剧烈轨道衰减。

<div align="center">
    <img src="https://imgur.com/n6RjwUC.jpg" alt="UI Screenshot" width="600" />
</div>

## 功能特性 | Features

*   **自然轨道衰减**
    *   模拟高层大气/外逸层中未加载和加载飞船的大气阻力。
    *   有效清理轨道碎片，防止低轨道垃圾永久滞留。
    *   **真实物理模拟**：根据飞船的估算面质比（Area-to-Mass Ratio）和实时大气密度计算阻力。

*   **太阳风暴效应** (需要 Kerbalism)
    *   在太阳风暴期间大幅增加轨道衰减率，模拟因太阳活动导致的大气膨胀效应。
    *   **可配置性**：可在 `Settings.cfg` 中设置是否让无大气星球（如 Mun）也受太阳风暴影响（默认关闭）。

*   **实时监控 UI**
    *   提供控制面板监控在轨/次轨载具的状态。
    *   实时查看远/近地点轨道高度、衰减状态及**远/近地点的预计再入时间**。
    *   支持按状态筛选飞船列表。

*   **智能预警系统**
    *   当载具首次再入大气层和载具自动销毁开始时发送通知。

## 依赖项 | Dependencies

*   **Click Through Blocker**
    *   本模组 UI 所必需。

*   **Kerbalism** (可选)
    *   *推荐安装*：以获得完整的太阳风暴衰减体验。
    *   *如果不安装*：本模组仍可作为独立的“自然大气衰减”模组运行。
    *   模组会在运行时自动检测 Kerbalism，无需手动选择或替换不同版本的 DLL。

## 兼容性 | Compatibility

*   ✅ **星球包 (Planet Packs)**: 理论上兼容所有星球包。
*   ❌ **Principia**: 不支持。

## 安装说明 | Installation

1.  下载 [最新版本 (Latest Release)](https://github.com/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay/releases)。
2.  将 `GameData` 文件夹解压到您的 KSP 安装目录中：
    *   `Kerbal Space Program/GameData/SpaceWeatherAndAtmosphericOrbitalDecay/`
3.  确保已安装 **Click Through Blocker**（必需）。
4.  **Kerbalism** 为可选依赖：模组会在运行时自动检测。安装了 Kerbalism 时启用太阳风暴衰减；未安装时仍可使用自然大气衰减，无需任何额外操作。

## 使用指南 | Usage

### 快捷键
*   **打开/关闭 UI**：`Alt + Q`

### 界面功能
*   **监控列表**：列出所有在轨飞船及其状态。
*   **筛选器**：
    *   `Stable` (稳定)
    *   `Natural` (自然衰减)
    *   `Storm` (风暴衰减)
*   **配置面板**：点击 `Show Config` 可在游戏内调整UI比例、字体大小，查看Debug信息等。

### 注意事项
*   UI 中显示的再入时间为估算值。它与实际衰减逻辑使用同一套轨道平均模型，但长时间高倍速、偏心轨道以及载具质量变化仍可能造成差异。

## 配置 | Configuration

除了游戏内 UI，您还可以通过编辑以下文件进行高级配置：
`GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Config/Settings.cfg`

| 配置项 | 描述 | 默认值 |
| :--- | :--- | :--- |
| `stormDecayRate` | 太阳风暴期间的基础衰减率 | `1.5e-7` |
| `stormDistanceScaling` | 风暴强度是否随距离太阳远近而变化 | `true` |
| `applyStormDecayToNoAtmosphereBody` | 是否对无大气天体应用风暴衰减 | `false` |
| `naturalDecayEnabled` | 启用自然大气衰减 | `true` |
| `naturalDecayMultiplier` | 自然衰减力度倍率 | `1.0` |
| `naturalDecayAltitudeCutoff` | 自然衰减生效的最大高度倍率 (相对于大气层高度) | `10.0` |
| `exosphereFitStart` | 高层大气密度拟合窗口起点（大气高度比例） | `0.80` |
| `exosphereFitEnd` | 高层大气密度拟合窗口终点（大气高度比例） | `0.90` |
| `exosphereScaleHeightMin` | 外逸层外推标高下限（大气高度比例） | `0.03` |
| `exosphereScaleHeightMax` | 外逸层外推标高上限（大气高度比例） | `0.30` |
| `exosphereFitSamples` | 用于拟合高层大气 `ln(density)` 斜率的采样数 | `8` |
| `orbitAverageSamples` | 轨道平均阻力估算的采样数 | `24` |
| `warningEnabled` | 是否启用低轨道警告 | `true` |
| `warningThreshold` | 低轨道警告阈值 (Periapsis < 大气高度 * (1.0 + 阈值)) | `0.2` |
| `reentryDestroySeconds` | 未加载载具进入大气层后销毁的倒计时秒数 | `60.0` |

## 公共 API | Public API

其他模组可通过 `SpaceWeatherAndAtmosphericOrbitalDecay.OrbitalDecayApi` 读取 SWAOD 的衰减节奏估算：

| 方法 | 用途 |
| :--- | :--- |
| `TryEstimateStationKeepingCadence(Vessel, 目标Ap, 目标Pe, 容差%, out estimate)` | 飞行中已发射载具的估算；Kerbalism 报告太阳风暴时会包含风暴衰减。 |
| `TryEstimateStationKeepingCadenceForOrbit(CelestialBody, 目标Ap, 目标Pe, 容差%, 质量, out estimate)` | VAB/装配规划，无 `Vessel` 实例；仅按目标轨道自然衰减估算（不含风暴）。 |

两者均返回只读的 `StationKeepingEstimate`，包含到达容差带所需时间、衰减速率、是否风暴估算、容差掉高，以及恢复该掉高的预计 delta-v；不会修改载具轨道。[Orbital Keeper](https://github.com/Aebestach/OrbitalKeeper) 的寿命估算与 VAB 规划功能使用这些接口。

## 致谢 | Credits

*   感谢 **Gemini 3 Pro** 对本 Mod 开发的协助。
