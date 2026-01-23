# Space Weather And Atmospheric Orbital Decay (空间天气与大气轨道衰减)

## 简介
Space Weather And Atmospheric Orbital Decay (SWAOD) 是一款坎巴拉太空计划 (KSP) 模组，旨在引入轨道衰减机制。它模拟了飞船在未加载和加载时大气阻力对轨道的影响，并与 Kerbalism 模组集成，模拟太阳风暴期间加剧的轨道衰减效应。

## 功能特性
- **自然轨道衰减**：模拟高层大气/外逸层中未加载飞船的大气阻力（原版 KSP 通常忽略此处的阻力，导致碎片永久滞留）。
- **太阳风暴效应**：（需要 Kerbalism）在太阳风暴期间大幅增加轨道衰减率，模拟因太阳活动导致的大气膨胀效应（玩家也可根据需要在`Settings.cfg`中调整无大气星球会不会受到太阳风暴的轨道衰减影响）。
- **真实物理模拟**：根据飞船的估算面质比（Area-to-Mass Ratio）和大气密度计算阻力。
- **游戏内 UI**：提供控制面板监控所有追踪的飞船，查看其衰减状态及预计再入时间。

## 依赖项
- **Kerbalism** (可选：太阳风暴衰减功能需要此模组。如果不安装 Kerbalism，本模组仍可作为独立的大气轨道衰减模组使用。)

## 安装说明
1. 下载最新版本。
2. 将 `GameData` 文件夹解压到您的 KSP 安装目录中 (例如 `C:\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program\`)。
3. **重要提示**：
   - 如果您 **使用 Kerbalism**：默认安装即可。
   - 如果您 **不使用 Kerbalism**：请使用 `Extra/Non-Kerbalism/` 目录下的 `.dll` 文件替换 `GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Plugin/` 中的文件。

## 使用说明
- **打开 UI**：按下 `Mod + F11`（Windows 上通常是 `Alt + F11`，Linux 上是 `RightShift + F11`）打开控制窗口。
- **监控**：窗口列出了所有在轨飞船、高度及其当前的衰减状态。
- **筛选**：使用底部的按钮按“稳定”、“自然衰减”或“风暴衰减”筛选飞船列表。
- **警告**：当飞船进入衰减区、轨道过低或即将再入大气层时，您会收到通知。

## 配置
可以通过编辑 `GameData/SpaceWeatherAndAtmosphericOrbitalDecay/Config/Settings.cfg` 文件完成。
