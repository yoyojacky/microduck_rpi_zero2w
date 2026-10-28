# microduck_rpi_zero2w

目前很火的项目，我想要迁移到树莓派 Zero 2W 上，使用的舵机是飞特的 HD1910，单独列一个仓库是为了方便记录和管理。

## 目标

把 [Pollen Robotics 的 Microduck](https://github.com/pollen-robotics/microduck) 双足机器人，用**树莓派 Zero 2W** 作主控、**15 颗飞特 HD-1910-C001** 作执行器，完整复刻出来：

- 50 Hz 实时控制环 + ONNX 神经网络策略驱动
- 能站立、能行走、能被蓝牙手柄遥控
- 放弃视觉（相机/WebRTC/ToF），换取 Zero 2W 的低成本和小体积
- 全链路文档化：从舵机开盒到策略重训，每一步可复现

### 选型理由

| 决策 | 理由 |
|---|---|
| Zero 2W 而非官方 Radxa Zero 3W | 手头已有；50 Hz 控制环当年就是在 Zero 2W 上定的；不做视觉就不需要更强算力 |
| 飞特 HD-1910 而非官方 XL330 | 扭矩大近 3 倍（1.47 vs 0.52 N·m）、成本低；[microduck-replica](https://github.com/fanhao375/microduck-replica) 已真机验证全套适配 |
| 舵机总线走 USB（FE-URT-2） | Zero 2W 的 PL011 串口被蓝牙占用，蓝牙要留给手柄 |
| IMU 用 BNO085 直飞 I2C | 官方总线 IMU 小板需自写 STM32 固件，I2C 方案零固件成本 |

## 文档

| 文档 | 内容 |
|---|---|
| [Microduck-Zero2W-HD1910-构建Cookbook.md](Microduck-Zero2W-HD1910-构建Cookbook.md) | **主教程**：阶段零~九 + 附录，从物料清单到策略重训的完整 cookbook |
| [HD1910中位校准教程.md](HD1910中位校准教程.md) | 舵机中位校准独立精编版（cookbook 阶段六的展开） |

## 操作步骤（路线图）

按 cookbook 的阶段推进，每步有明确验收标准：

- [ ] **阶段一 · 舵机台架配置**：FD 上位机烧 ID（右腿10-14/左腿20-24/颈头嘴30-34）、开过流/过压保护、贴标签、导出寄存器底账
- [ ] **阶段二 · Zero 2W 系统**：Raspberry Pi OS Lite 64-bit、开 I2C、装 ONNX Runtime 1.28.0、BlueZ 手柄配置
- [ ] **阶段三 · 运行时部署**：官方 install.sh → 自签 dev 密钥 → 交叉编译 [feetech 分支](https://github.com/fanhao375/microduck/tree/feetech) → dev-push 上板
- [ ] **阶段四 · 接线供电**：电池→UBEC→Pi、舵机总线三路分支、BNO085 接 i2c-1、共地检查单
- [ ] **阶段五 · 配置 robotd.toml**：`protocol="feetech-sts"`、directions 全 −1、p_scale、关策略和音频
- [ ] **阶段六 · 中位校准**：方向核对 → 官方折叠基准 15 颗联标（零点=直站=2048）→ 行程验证
- [ ] **阶段七 · 台架验收**：12 项清单（通信/时序/单位/方向/增益/过载恢复/IMU/吊架站立）
- [ ] **阶段八 · 手柄与首站**：pad pair → 吊架 init 站立
- [ ] **阶段九 · 策略重训**：microduck_rl + HD-1910 M6 参数，训出能走的策略（官方 ONNX 在飞特上不能直接用）

## 参考仓库

| 仓库 | 用途 |
|---|---|
| [pollen-robotics/microduck](https://github.com/pollen-robotics/microduck) | 官方运行时（Rust） |
| [fanhao375/microduck-replica](https://github.com/fanhao375/microduck-replica) | 飞特 HD-1910 复刻主线，本项目的实测依据 |
| [fanhao375/microduck](https://github.com/fanhao375/microduck)（feetech 分支） | 含飞特协议支持的运行时，部署目标 |
| [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) | RL 训练框架 |
| [pollen-robotics/elec_RPI_Robot_HAT](https://github.com/pollen-robotics/elec_RPI_Robot_HAT) | 官方 HAT 硬件参考（本方案不打 HAT） |

## License

见 [LICENSE](LICENSE)。文档中引用的第三方项目内容，其权利归原项目所有。
