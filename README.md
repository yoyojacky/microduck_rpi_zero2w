# microduck_rpi_zero2w

目前很火的项目，我想要迁移到树莓派 Zero 2W 上，使用的舵机是飞特的 HD1910，单独列一个仓库是为了方便记录和管理。

## 目标

把 Microduck 双足机器人，用以下方案完整复刻：

- **主控**：树莓派 Zero 2W
- **执行器**：15 颗飞特 HD-1910-C001
- **供电/IO 枢纽**：官方 RPI Robot HAT（已自制 ✅）
- **躯干 IMU**：imu_to_dxl 总线小板（已自制 ✅，总线 ID 200）
- **运行时**：kissqy HD-1910 整套（control.43，随包 10 份运动模型）
- 放弃视觉（相机/WebRTC/ToF），换取 Zero 2W 的低成本和小体积
- 全链路文档化：从舵机开盒到首次行走，每一步可复现

### 选型理由

| 决策 | 理由 |
|---|---|
| Zero 2W 而非官方 Radxa Zero 3W | 手头已有；50 Hz 控制环当年就是在 Zero 2W 上定的；不做视觉就不需要更强算力 |
| 飞特 HD-1910 而非官方 XL330 | 扭矩大近 3 倍（1.47 vs 0.52 N·m）；kissqy/replica 已全套真机验证 |
| 官方 RPI Robot HAT | 5–28V 输入直接吃电池、buck 出 5V 给 Pi，取代 UBEC；保留音频/关机检测的进阶空间 |
| imu_to_dxl 总线 IMU 小板 | 官方同架构：IMU 与 15 颗舵机同一次 SYNC READ 读回，时序严格一致；不占 I2C |
| 舵机总线走 USB（FE-URT-2） | Zero 2W 的 PL011 串口被蓝牙占用，蓝牙要留给手柄；HAT 的总线口留空 |
| kissqy 固件包而非官方运行时 | 官方只认 Dynamixel；kissqy 完成全部 HD-1910 适配且随包 10 份模型开箱即走 |

## 文档

| 文档 | 内容 |
|---|---|
| [Microduck-Zero2W-HD1910-构建Cookbook.md](Microduck-Zero2W-HD1910-构建Cookbook.md) | **主教程**：阶段零~九 + 附录，从物料清单到首次行走的完整 cookbook |
| [HD1910中位校准教程.md](HD1910中位校准教程.md) | 舵机中位校准独立精编版（cookbook 阶段六的展开） |

## 操作步骤（路线图）

按 cookbook 的阶段推进，每步有明确验收标准：

- [ ] **阶段一 · 舵机台架配置**：FD 上位机烧 ID（右腿10-14/左腿20-24/颈头嘴30-34）、开过流/过压保护、贴标签、导出寄存器底账
- [ ] **阶段二 · Zero 2W 系统**：Raspberry Pi OS Lite 64-bit、蓝牙手柄配置、zram
- [ ] **阶段三 · 运行时安装**：kissqy ARM64 固件包（control.43）装到 Pi OS、总线指向 ttyACM0、Windows 中控联机
- [ ] **阶段四 · 接线供电**：电池→HAT→5V 给 Pi、舵机总线三路分支、imu_to_dxl 小板烧飞特固件挂总线、检查单
- [ ] **阶段五 · 配置要点**：双份 robotd.toml 的规则、掉电保护、别手 sed 配置
- [ ] **阶段六 · 中位校准**：方向核对 → 官方折叠基准 15 颗联标（零点=直站=2048）→ 行程验证
- [ ] **阶段七 · 台架验收**：12 项清单（通信/时序/单位/方向/增益/过载恢复/总线 IMU/吊架站立）
- [ ] **阶段八 · 手柄与首站**：配对手柄 → 吊架 HOME → IMU 安装方向确认 → 随包模型玩耍
- [ ] **阶段九 ·（进阶）自训模型**：训练中控 R1.5.27 或 microduck_rl + HD-1910 M6 参数

## 参考仓库

| 仓库 | 用途 |
|---|---|
| [kissqy/Microduck-kissqy](https://github.com/kissqy/Microduck-kissqy) | HD-1910 整套主线：固件包、中控、随包模型（本项目部署目标） |
| [fanhao375/Microduck-kissqy](https://github.com/fanhao375/Microduck-kissqy) | fork：I2C IMU 备胎、舵机调参、UDP 手柄、掉线坐下 |
| [fanhao375/microduck-replica](https://github.com/fanhao375/microduck-replica) | 复刻总站：实测记录、imu_to_dxl 工程与固件、servo-web 调试台 |
| [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) | 官方运行时（上游） |
| [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) | RL 训练框架（阶段九） |
| [pollen-robotics/elec_RPI_Robot_HAT](https://github.com/pollen-robotics/elec_RPI_Robot_HAT) | HAT 硬件工程 |

## License

见 [LICENSE](LICENSE)。文档中引用的第三方项目内容，其权利归原项目所有（kissqy 原创部分为个人非商业许可，使用时注意）。
