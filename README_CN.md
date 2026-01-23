# Space Weather And Atmospheric Orbital Decay
# 空间天气与大气轨道衰减

<div align="center">

<img src="https://imgur.com/2bk9Zad.jpg" alt="Banner"/>

[![License](https://img.shields.io/github/license/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay)](LICENSE)
[![Release](https://img.shields.io/github/v/release/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay)](https://github.com/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay/releases)

[English](README.md) | [中文](README_CN.md)

</div>

---

## 📖 简介 | Introduction

**Space Weather And Atmospheric Orbital Decay (SWAOD)** 是一款为 **Kerbal Space Program (KSP)** 设计的模组，旨在引入真实的轨道衰减机制。

原版游戏中，飞船在真空中永远不会受到阻力影响。本模组改变了这一点：它模拟了高层大气对飞船的持续阻力，并与 **Kerbalism** 集成，模拟太阳风暴期间因大气膨胀导致的剧烈轨道衰减。

<div align="center">
    <img src="https://imgur.com/4oK8ftS.jpg" alt="UI Screenshot" width="600" />
</div>

## ✨ 功能特性 | Features

*   **🪐 自然轨道衰减**
    *   模拟高层大气/外逸层中未加载和加载飞船的大气阻力。
    *   有效清理轨道碎片，防止低轨道垃圾永久滞留。
    *   **真实物理模拟**：根据飞船的估算面质比（Area-to-Mass Ratio）和实时大气密度计算阻力。

*   **☀️ 太阳风暴效应** (需要 Kerbalism)
    *   在太阳风暴期间大幅增加轨道衰减率，模拟因太阳活动导致的大气膨胀效应。
    *   **可配置性**：可在 `Settings.cfg` 中设置是否让无大气星球（如 Mun）也受太阳风暴影响（默认关闭）。

*   **📊 实时监控 UI**
    *   提供控制面板监控所有追踪飞船的状态。
    *   实时查看轨道高度、衰减状态及**预计再入时间**。
    *   支持按状态筛选飞船列表。

*   **⚠️ 智能预警系统**
    *   当飞船进入衰减区、轨道过低或即将再入大气层时发送通知。

## 🛠️ 依赖项 | Dependencies

*   **Kerbalism** (可选)
    *   *推荐安装*：以获得完整的太阳风暴衰减体验。
    *   *如果不安装*：本模组仍可作为独立的“自然大气衰减”模组运行。

## 📥 安装说明 | Installation

1.  下载 [最新版本 (Latest Release)](https://github.com/Aebestach/SpaceWeatherAndAtmosphericOrbitalDecay/releases)。
2.  将 `GameData` 文件夹解压到您的 KSP 安装目录中：
    *   `Kerbal Space Program/GameData/SpaceWeatherAndAtmosphericOrbitalDecay/`
3.  **⚠️ 重要提示**：
    *   **使用 Kerbalism**：默认安装即可。
    *   **不使用 Kerbalism**：请使用 `Extra/Non-Kerbalism/` 目录下的 `.dll` 文件替换 `GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Plugin/` 中的文件。

## 🎮 使用指南 | Usage

### 快捷键
*   **打开/关闭 UI**：`Alt + F11`

### 界面功能
*   **监控列表**：列出所有在轨飞船及其状态。
*   **筛选器**：
    *   `Stable` (稳定)
    *   `Natural` (自然衰减)
    *   `Storm` (风暴衰减)
*   **配置面板**：点击 "Show Config" 可在游戏内调整UI比例、字体大小，查看Debug信息等。

## ⚙️ 配置 | Configuration

除了游戏内 UI，您还可以通过编辑以下文件进行高级配置：
`GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Config/Settings.cfg`

| 配置项 | 描述 | 默认值 |
| :--- | :--- | :--- |
| `stormDecayRate` | 太阳风暴期间的基础衰减率 | `1.5e-7` |
| `stormDistanceScaling` | 风暴强度是否随距离太阳远近而变化 | `true` |
| `applyStormDecayToNoAtmosphereBody` | 是否对无大气天体应用风暴衰减 | `false` |
| `naturalDecayEnabled` | 启用自然大气衰减 | `true` |
| `naturalDecayMultiplier` | 自然衰减力度倍率 | `0.5` |
| `naturalDecayAltitudeCutoff` | 自然衰减生效的最大高度倍率 (相对于大气层高度) | `10.0` |
| `warningEnabled` | 是否启用低轨道警告 | `true` |
| `warningThreshold` | 低轨道警告阈值 (Periapsis < 大气高度 * (1.0 + 阈值)) | `0.2` |

## 🤝 致谢 | Credits

*   感谢 **Gemini 3 Pro** 对本 Mod 开发的协助。
