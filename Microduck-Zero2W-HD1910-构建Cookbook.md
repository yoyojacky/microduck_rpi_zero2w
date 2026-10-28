# Microduck 构建 Cookbook：树莓派 Zero 2W + 飞特 HD-1910 + 官方 HAT + 总线 IMU 小板

> 目标读者：手头有 15 颗飞特 HD-1910-C001、一块树莓派 Zero 2W、**自制了官方 RPI Robot HAT**、**自制了 imu_to_dxl 总线 IMU 小板**，想从零把 Microduck 做起来的人。
> 读者不需要读过四个仓库的源码——每一步都给出命令、预期输出和"为什么这么做"。
>
> **验证状态图例**：✅ = replica / kissqy 项目真机验证过；📋 = 从代码/文档推出、路径明确但本组合（Zero 2W）没人实测过，做的时候留意。
>
> 内容来源：`pollen-robotics/microduck`（官方运行时）、`kissqy/Microduck-kissqy` 与 `fanhao375/Microduck-kissqy`（HD-1910 整套主线）、`fanhao375/microduck-replica`（复刻总站，全部实测记录）、`pollen-robotics/microduck_rl`（训练）、`pollen-robotics/elec_RPI_Robot_HAT`（HAT 工程）。关键出处随文标注。

---

## 阶段零 · 先看清全局

### 0.1 你要造的是什么

Microduck 是一只 ~25 cm 高、~800 g 的双足鸭子机器人。它的本质是一个 **50 Hz 实时控制环**：

```
每 20 ms：
  从总线一次读出 15 颗舵机的位置/速度/电压/温度 + IMU 小板的姿态（ID 200）
  拼成 61 维观测向量 → 喂给 ONNX 神经网络策略 → 得到 14 维关节目标
  一次写回 15 颗舵机的目标位置
```

走路、坐下、捡东西、踢球、前滚翻都是训练出来的神经网络（策略）在驱动。kissqy 的 HD-1910 整套**随包带 10 份针对 HD-1910 训好的运动模型**，开箱即走；自己重训是进阶选项（阶段九）。

### 0.2 系统架构（你最终要接出来的样子）

```
                        ┌──────────────────────────────────┐
  蓝牙手柄 ──BLE──►    │ 树莓派 Zero 2W                    │
  (GameSir G50 Pro /   │  robotd（50Hz 控制环+策略+安全）   │
   PS4 / Switch Pro)   │  padd / btd / configd …           │
                        └───┬───────────────────▲──────────┘
                   USB OTG  │                   │ 40pin（5V 供电）
                ┌───────────▼──────┐   ┌────────┴───────────┐
                │ FE-URT-2 调试板   │   │ 官方 RPI Robot HAT  │
                │（USB↔TTL 转接）   │   │ 5-28V 输入 → 5V    │
                └───────────┬──────┘   │ 音频/ToF（Pi 上暂缓）│
                            │ S（1Mbps 单线半双工）           │
        ┌───────────────────┼─────────────────┐
   头颈支路            左腿支路            右腿支路
  30→31→32→33→34   20→21→22→23→24   10→11→12→13→14
                  （imu_to_dxl 小板 ID 200 挂任一路末端/中间）

  NP-F550 电池(2S, 6.6–8.4V)
       └──► 开关 ──► 保险丝 5A ──► HAT 电源输入（舵机电源口）
                                   ├─► HAT buck → 5V → 40pin → Zero 2W
                                   └─► 舵机总线 V（原压直通，18AWG）
       共地：电池负 · HAT · Pi · 舵机总线 G · URT-2 G
```

**为什么舵机吃电池原压**：HD-1910 额定 7.4 V，正好匹配 2S 锂电池（标称 7.4 / 满电 8.4 V）。HAT 的设计就是"舵机电源口即整机电源输入"（5–28 V），buck 只给主控出 5 V。

**为什么舵机总线走 USB（URT-2）而不是 HAT 的总线口**：HAT 上的 TTL 总线电路接的是 40pin 的 UART（GPIO 14/15），而 Zero 2W 唯一的全功能串口（PL011）默认被蓝牙占着，蓝牙要留给手柄。URT-2 插 USB 就是 `/dev/ttyACM0`，不抢串口。HAT 的总线口留空。

**为什么 IMU 用总线小板而不是 HAT 上的 IMU**：HAT 上的 IMU（C1 是 BMI088，D1 换成 LSM6DSV16X）在官方体系里是"头部 IMU"（head_imu，默认就关着，走路不用它）。**躯干主 IMU 必须是总线上 ID 200 的那块**——kissqy 的 robotd 只认它（或经 fork 配置的 I2C 模块）。imu_to_dxl 小板和 15 颗舵机同一次 SYNC READ 读回，时序严格一致。

### 0.3 物料清单

**机器人本体**

| 物料 | 规格要点 | 为什么 |
|---|---|---|
| 树莓派 Zero 2W | 配 2×20 排针（需焊接） | 主控。4×A53 + 512 MB，跑控制环和策略够用（50 Hz 控制环当年就是在 Zero 2W 上定的） |
| 飞特 HD-1910-C001 ×15 | 1.47 N·m @7.4V，4096 步/圈 | 执行器 |
| **官方 RPI Robot HAT**（自制 ✅） | 5–28V 输入、AP63205 buck 5V、音频 codec、关机检测（B1 起） | 供电与 IO 枢纽。取代 UBEC |
| **imu_to_dxl 总线 IMU 小板**（自制 ✅） | STM32G031F8P6 + LSM6DSV16X | 躯干主 IMU，总线 ID 200 |
| 飞特 FE-URT-2 调试板 | USB-C，带 TTL 口 | 一物两用：先在电脑上配舵机，配完挪进机器人当 USB 转接板 |
| NP-F550 电池 + 电池仓 | 2S 锂电 | 整机电源 |
| 保险丝座 + 5A 保险丝、船型开关 | 5×20mm 快熔；KCD1 ≥6A | 电源入口保护 |
| 18AWG 硅胶线 | 红黑各 1m | 电源与总线 V/G 主干。15 颗堵转能到十几安 |
| PH2.0 3P 端子线 ×5、杜邦线 | — | 总线分支与逻辑接线 |
| 高耐久 microSD ×2 | 32–64 GB，High Endurance | 一张用一张备。机器人硬断电多，普通卡容易坏 |
| 结构件 | 3D 打印，**飞特版 CAD**（replica 改过 8 个件适配 HD-1910） | 官方 XL330 的件装不上飞特 |
| 蓝牙手柄 | GameSir G50 Pro / PS4 / Switch Pro | 遥控（kissqy 实测用 G50 Pro） |
| ~~UBEC 降压模块~~ | 不需要了 | HAT 的 buck 已出 5V |
| *（备胎）BNO085 IMU 模块* | I2C，3.3V | 总线小板出问题时的替补，用 fanhao375 fork 的 `[trunk_imu] source="bno08x"` |

**台架工具**

| 工具 | 用途 |
|---|---|
| Windows PC | 飞特 FD 上位机（配 ID、中位校准）+ kissqy 机器人中控（标定/驾驶/固件维护） |
| CMSIS-DAP 调试器 + OpenOCD | 给 imu_to_dxl 小板烧固件（⚠️ 别用 Keil UV4 命令行烧，会假成功） |
| 舵机转接线 2.0→2.5 3P（或一根剪开的舵机线） | URT-2 的 TTL 口是 2.54mm，舵机原装线 2.0mm 插不上 |
| 可调直流电源（限流 3–5 A） | 台架阶段代替电池 |
| 万用表、直角尺、标签纸、吊架 | 量电、摆校准基准、标记舵机、首次上力保护 |
| 一台能交叉编译的电脑（Linux/macOS/WSL + Docker） | 只有要改代码/用 fork 功能时才需要；**用现成固件包则完全不需要编译** |

### 0.4 软件清单（先认清谁是谁）

| 软件 | 在哪跑 | 干什么 |
|---|---|---|
| 飞特 FD 上位机 | Windows PC | 舵机烧 ID、开保护、中位校准。`microduck-replica/tools/飞特/`（FD 1.9.8.5 + 驱动） |
| Raspberry Pi OS Lite 64-bit | Zero 2W | 操作系统 |
| **kissqy 已编译固件包** `Microduck-Robotd-FT6-R17.tar.gz` | Zero 2W | **主路径**：HD-1910 整套运行时（control.43），ARM64 免编译，内含 10 份运动模型 |
| kissqy 机器人中控 R17 V167 | Windows PC | 连接鸭子：3D 姿态、标定、手柄操控、固件升级 |
| *（进阶）* `fanhao375/Microduck-kissqy` fork | 交叉编译 → Zero 2W | 加 I2C IMU 模块（BNO085 备胎）、舵机调参、UDP 手柄、掉线坐下 |
| imu_to_dxl 固件 | 小板 | **replica `firmware-1.0.0`（飞特 FT-SCS 协议）**，有现成 hex；或 kissqy 的 FT6 固件 |
| servo-web 调试台 | Zero 2W（浏览器访问） | 舵机方向核对、姿态调试。`microduck-replica/tools/servo-web/` |
| *（进阶）* 训练中控 R1.5.27 / microduck_rl | 带 GPU 的电脑 | 自训新动作（阶段九） |

### 0.5 开始前必须知道的三件事

1. **运动模型开箱即用，但必须是 HD-1910 版的**。官方 pollen 的策略（XL330 动力学）在飞特上不能走；**kissqy 固件包随带的 10 份模型就是 HD-1910 的**（走/坐/捡/踢/滚翻 + 滑轮），本教程阶段八结束就能玩。自己重训是阶段九的进阶内容。
2. **Zero 2W 路线放弃视觉**，HAT 上的音频/ToF 也暂缓（Pi 上没有现成设备树配置，属进阶项目，附录 C）。控制、行走、手柄、标定不受影响。
3. **kissqy 的发行体系是围绕 Radxa Zero 3W 镜像建的**，Zero 2W 上要用它的 ARM64 固件包装到 Raspberry Pi OS 上——这一步是 📋 程度最高的环节（阶段三详述）。如果你想先 100% 走通再移植，可以先拿一块 Radxa Zero 3W 刷 kissqy 镜像跑通全套（纯 ✅），Pi 版作为二期。

---

## 阶段一 · 舵机台架配置（Windows PC + FD 上位机）✅

> 目标：15 颗 HD-1910 每颗烧好 ID、打开保护、贴上标签。
> 出处：`microduck-replica/调试记录.md`「舵机配置 · 拿到货一步一步做」，全部实测。
> 耗时：约 40 分钟（一颗约 2 分钟，加上第一次熟悉软件）。

**为什么先配舵机再碰主控**：15 颗舵机出厂 ID 全是 1，接在一条总线上会互相撞包，一颗都认不出来。必须一次接一颗、改 ID、贴标签。这一步在电脑上做，跟树莓派无关。

### 1.1 装 FD 软件和驱动

1. URT-2 插电脑 USB。设备管理器「端口 (COM 和 LPT)」里应出现 **USB-Enhanced-SERIAL CH343 (COMx)**，记下 COM 号。
   - *为什么*：URT-2 用 CH343 芯片，Win10/11 自带驱动。没出现才装 `microduck-replica/tools/飞特/CH340_URT.rar`（老款 URT-1 用）。
2. 解压 `microduck-replica/tools/飞特/FD1.9.8.5(250729).7z`，直接运行，免安装。

### 1.2 飞线（或转接线）

URT-2 左边两个白色 3 针口（丝印 `TTL-Bus`，针脚 `G V1 S`）比舵机的 2.0mm 线粗，插不上。剪一根舵机线（15 根牺牲一根）或买「舵机转接线 2.0 转 2.5 3P」。

飞线接法——**认脚号，别认颜色**：

```
舵机线                       URT-2
脚 1  Signal / TTL    →    S     （焊到白口 S 脚焊点，或线头镀锡插进 S 孔）
脚 2  Vcc             →    V1    （蓝色接线端子）
脚 3  GND             →    G     （蓝色接线端子）
```

- **中间那根一定是 V**，飞特和 Dynamixel 一致。V 接错才会烧。
- S 和 G 接反**不会烧**，只是搜不到，对调即可。
- 脚 1 在插头上有小三角/缺口标记，或看舵机插座旁丝印。

### 1.3 供电

**URT-2 上的拨码是 TTL 信号电平（3.3/5 V）选择，不是供电切换。拨 5 V**（HD-1910 认 2–5 V，余量大）。

| 方式 | 接法 | 什么时候用 |
|---|---|---|
| USB 5 V | 什么都不接，V1 自带 5 V | **只配 ID、改参数**（本阶段）。空载够 |
| 可调电源 → V1 端子 | + 进 V1、− 进 G | 15 颗一起挂的总线调试 |
| 可调电源直供舵机 | + 接舵机 V，− 接舵机 G 和 URT-2 的 G（共地），V1 空着 | 不想让电流过 URT-2 端子时（标称 6 A） |

⚠️ 三条军规：
- **别超 8.4 V**——HD-1910 电压上限，零余量。别在满电电池上做堵转测试。
- **上电顺序：先插 USB，再给舵机上电。** 反了总线电平会被压在 2.1 V，一颗都不应答（replica 卡了一小时才查到这个）。
- 本阶段只接一颗舵机，USB 5 V 就够。

### 1.4 搜舵机

```
1  FD 里选 1.1 记下的 COM 口
2  波特率选 1000000（HD-1910 出厂就是 1 Mbps）
3  点「打开」
4  确认只接了一颗舵机
5  点「搜索」→ 列表出现 ID 1，型号 HD-1910
```

搜不到按顺序查：驱动 → COM 口 → 波特率 → S 线接触（临时插孔最可疑）→ S/G 是否接反。

### 1.5 改参数（每颗都做）

切到「编程」页（EPROM）。**只改这几项，别的别动：**

| 参数 | 出厂 | 改成 | 为什么 |
|---|---|---|---|
| **ID** | 1 | 按下表 | 运行时按 ID 找关节 |
| **过流保护** | 关 | **开** | 装机后堵转是常事，不开就是烧舵机 |
| **过压保护** | 关 | **开** | 8.4 V 满电落在 4–10 V 窗口内，不会误触发 |
| 过热保护 | 开（80°C） | 不动 | 出厂已对 |
| 波特率 | 1 Mbps | 不动 | 出厂就是运行时要的 |
| 运行模式 | **模式 4 纯位置 PD** | 不动 | 强化学习落地要的就是它，出厂已对，千万别改成 0 |
| Return delay | 0 | 不动 | STS 系列这项不起作用 |
| 中位 | — | **现在不校** | 整机组装好后才校（阶段六）。FD 的中位校准只能把当前位置当 2048，现在校了也白校 |

改完点「保存」。

**ID 对照表**（来自上游 `duck-control/src/model.rs`，写死在代码里）：

| ID | 关节 | ID | 关节 | ID | 关节 |
|---|---|---|---|---|---|
| **10** | 右髋 yaw | **20** | 左髋 yaw | **30** | 颈俯仰 |
| **11** | 右髋 roll | **21** | 左髋 roll | **31** | 头俯仰 |
| **12** | 右髋 pitch | **22** | 左髋 pitch | **32** | 头 yaw |
| **13** | 右膝 | **23** | 左膝 | **33** | 头 roll |
| **14** | 右踝 | **24** | 左踝 | **34** | 嘴 |

**记法**：十位 = 部位（1 右腿 · 2 左腿 · 3 颈头），个位 = 从躯干往末端数（0 髋 yaw → 4 踝）。

### 1.6 验证（别省）

```
1  断电（拔 USB 或关电源）
2  重新上电
3  再点「搜索」→ 显示新 ID 才算写进去了
```

*为什么*：EPROM 参数要断电重启才确定生效。

### 1.7 贴标签，下一颗

标签写**关节名 + ID**（如「左膝 23」），别只写数字。拔掉，换下一颗，回 1.4。

### 1.8 顺手做：导出寄存器底账

挑一颗，在 FD「编程」页把寄存器 0–90 全部导出存档。这是这颗舵机的"出厂档案"，以后任何寄存器疑问（电流单位、相位位、保护参数）都拿它来对。

**阶段一验收**：断电重上后，15 颗各自能搜到自己的新 ID；每颗身上贴着「关节名 + ID」。

---

## 阶段二 · Zero 2W 系统准备 📋

> 目标：一块刷好 Raspberry Pi OS Lite 64-bit、网络就绪的 Zero 2W。
> kissqy 的镜像只有 Radxa 版，Pi 上从零装——好在他的固件是用户态程序，不绑内核。

### 2.1 刷系统（Raspberry Pi Imager）

1. 电脑上下载 Raspberry Pi Imager（raspberrypi.com/software）。
2. 设备选 **Raspberry Pi Zero 2 W**；系统选 **Raspberry Pi OS Lite (64-bit)**——必须 Lite（无桌面，省内存）且必须 64 位（固件包是 ARM64）。
3. 「编辑设置」做 headless 配置：
   - 主机名：`microduck`（路由器里好认）
   - 用户：`duck`，密码自定
   - WiFi：SSID、密码、国家码（CN）
   - 服务：**开启 SSH，密码登录**
4. 烧录，插卡。

### 2.2 首次开机、SSH

1. 插卡上电，首次会扩容分区并重启，**等 3–5 分钟**。
2. 路由器后台找 `microduck` 的 IP（或 `ssh duck@microduck.local`）。
3. `ssh duck@<IP>` 登录。

**多一条命**：WiFi 连不上就插显示器（mini-HDMI）+ USB 键盘，或串口控制台。先点亮再谈接线。

### 2.3 系统更新与基础包

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl i2c-tools
```

**先把时钟对好**：`timedatectl` 看 `System clock synchronized: yes`。板子没有 RTC 电池，时间不对会让 HTTPS/TLS 握手失败，报错还很隐晦。

### 2.4 蓝牙手柄的内核配置

```bash
sudo nano /etc/bluetooth/input.conf
```

写入（文件不存在就新建）：

```ini
[General]
UserspaceHID=false
```

*为什么*：这一行决定经典蓝牙手柄的报文由内核还是 bluetoothd 用户态处理。交给内核，padd 才能用标准 evdev 接口读手柄（官方 `setup-board.sh` 的同款配置）。改完重启生效。

### 2.5 串口与 I2C：本方案都不用动

- **串口**：舵机总线走 URT-2 的 USB（`/dev/ttyACM0`），GPIO 串口一个都不用，蓝牙保持默认完整留给手柄。
- **I2C**：主 IMU 是总线小板（ID 200），不需要 I2C。只有将来启用 BNO085 备胎（fork 的 `[trunk_imu] source="bno08x"`）时才需要 `sudo raspi-config` 开 I2C（设备名 `/dev/i2c-1`）。现在不开也行。

### 2.6（建议）开 zram

512 MB 内存紧张，zram 用 CPU 换内存：

```bash
sudo apt install -y zram-tools   # 默认配置即可
```

### 2.7 重启并验证

```bash
sudo reboot
```

回来后：

```bash
uname -m              # aarch64
timedatectl | grep -i synch   # yes
free -h               # 能看到 zram swap
```

**阶段二验收**：能 SSH、时钟同步、蓝牙配置就位。

---

## 阶段三 · 安装 kissqy HD-1910 运行时 📋（Zero 2W 上最关键的一步）

> 目标：把 kissqy 的 control.43 固件（HD-1910 整套：robotd + 模型 + 脚本 + systemd）装进 Raspberry Pi OS。
> **这是本教程唯一没有真机先例的阶段**——kissqy 只发 Radxa 镜像，但固件本身是 ARM64 用户态程序，Pi OS 同样是 Debian 系 aarch64，路径是通的。做的时候每一步都核对，遇到与 Radxa 绑定的环节（设备树 overlay、HAT 音频）跳过并记录。

### 3.1 拿到固件包

在 Zero 2W 上（或电脑下载后 scp 过去）：

```bash
cd ~
curl -fLO https://github.com/kissqy/Microduck-kissqy/raw/refs/heads/main/microduck-hd1910/firmware/console/updates/Microduck-Robotd-FT6-R17.tar.gz
tar tzf Microduck-Robotd-FT6-R17.tar.gz | head -30   # 先看一眼包结构
```

这个包是 kissqy 官方发布的"已有系统单独升级 ZERO 端服务"的 ARM64 安装包，与 V167 中控内含的 control.43 一致。包结构、安装与校验方式以其发布说明为准（仓库 `release-assets/README.md`，"独立固件安装与校验"一节）。

*为什么不用官方 pollen 的 install.sh*：官方运行时只认 Dynamixel XL330 舵机，你的 15 颗飞特说不了 Dynamixel 协议。kissqy 的固件是在官方 0.15.1 基础上完成的全部飞特适配（总线协议、标定、模型），**这才是 HD-1910 能跑的运行时**。

### 3.2 安装

按包内说明执行安装（通常是解包到 `/opt/robot/` 体系 + 安装 systemd 单元）。逐条注意：

1. **目录布局**：kissqy 体系的路径是 `/opt/robot/feetech-ft5-r5/`（程序与脚本）、`/etc/robot/feetech-ft5/`（`robotd.toml` 与 `robotd.saved.toml` 双份配置）。与官方 pollen 的 `/etc/robot/` 不冲突，可共存。
2. **systemd 单元**：安装后应有 `robotd.service`、`padd.service` 等。`systemctl status robotd` 能看到。
3. **跳过 Radxa 专属步骤**：安装脚本若涉及设备树 overlay（uart/i2c/audio 的 `.dtbo`）、`armbianEnv.txt`、extlinux——这些是 Radxa 的，Pi 上跳过，手工不做。总线我们用 USB（URT-2），不需要串口 overlay。
4. **关掉 Pi 上跑不动的服务**（若有）：`mediad`（相机）、`tofd`（ToF）——没接硬件还占 CPU，参考命令 `sudo systemctl disable --now mediad tofd`。

### 3.3 把总线指向 URT-2

kissqy 的配置在 `/etc/robot/feetech-ft5/robotd.toml`（改完同步 `robotd.saved.toml`，它是升级/重建配置时的来源）。找到串口配置项，改为：

```toml
port = "/dev/ttyACM0"
```

*为什么是 ttyACM0*：URT-2 的 CH343 被 Linux 识别为 USB ACM 串口。插好后 `dmesg | tail` 能看到 `ttyACM0`。如果枚举成 `ttyUSB0`（某些驱动），按实际填。

### 3.4 首次联机（Windows 中控）

1. Windows 电脑下载**机器人中控 R17 V167** ZIP，完整解压，运行 `start.cmd`。
2. 确保电脑和鸭子在同一局域网，中控连接鸭子 IP。
3. 中控能看到关节/IMU 数据，说明 robotd 已正常工作。

⚠️ **中控里「安装 / 升级固件」按钮的用法**：它会把固件升到中控随包版本——现在你的固件和中控同版（control.43），不需要点。将来若改用 fanhao375 fork（阶段三 3.6），**就别再点它**（会装回 kissqy 原版，覆盖 fork 功能）。

### 3.5 阶段的验收

- `systemctl status robotd`：active
- `journalctl -u robotd -b`：无反复重启
- 中控能连上、能看到数据（总线未接舵机时报"等总线"属正常，阶段四接线后消失）

### 3.6（进阶）fanhao375 fork：I2C IMU 备胎 / 舵机调参 / UDP 手柄

**现在可以跳过**——先用 kissqy 原版跑通。需要以下任一功能时再回来：

| fork 加的功能 | 什么时候想要 |
|---|---|
| `[trunk_imu] source="bno08x"` | 总线小板坏了/没做，用 BNO085 模块顶（接 Pi 的 i2c-1，地址 0x4A/0x4B） |
| 舵机调参（P/D/转矩限制） | 站姿抖动、跟官方动力学有差异时细调 |
| UDP 手柄、掉线坐下 | 用电脑手柄局域网约控；手柄断联自动坐下 |

要点（完整步骤读 fork 的 `IMU-MODULE.md`）：

```bash
# 编译机（Linux/WSL，Rust 1.99 + zig + cargo-zigbuild）
git clone -b hd1910-v167-trunk-imu-i2c https://github.com/fanhao375/Microduck-kissqy.git
cd Microduck-kissqy/microduck-hd1910/firmware
python3 prepare-V167.py
export DUCK_SYSROOT=~/.cache/duck-sysroot
sh scripts/cross-sysroot.sh     # 最后打印几行 export，照着执行
cargo board --locked -p robotd -p robotctl -p padd

# 拷 3 个二进制 + packaging 5 个脚本 + deploy/imu-module-install.sh 到板上 ~/replica/
# 板上（舵机电源开着、扶住鸭子）：
sudo bash ~/replica/imu-module-install.sh install ~/replica bno08x /dev/i2c-1 0x4A
# 用总线小板（bus 模式）则不带 bno08x 参数的细节以 IMU-MODULE.md 为准
```

安装脚本会卸力、备份、换文件、四步启动核对，任何一步失败自动撤回（`rollback` 手动回滚）。**装好 fork 后别再点中控的「安装 / 升级固件」**。

---

## 阶段四 · 接线与供电 ✅/📋

> 目标：电池、HAT、Pi、URT-2、舵机总线、IMU 小板全部接好。
> 原则：**每接一段量一次电压，确认极性再插。**

### 4.1 电源链路（有 HAT 版）

```
NP-F550 ──► 开关 ──► 保险丝 5A ──► HAT 电源输入（舵机电源口，5–28V）
                                     ├─► HAT 内部 buck ──► 5V ──► 40pin ──► Zero 2W
                                     └─► 舵机总线 V（电池原压直通）
共地：电池负极 · HAT · Pi · 舵机总线 G · URT-2 G
```

按顺序做，每步用万用表：

```
1  接开关、保险丝。量：开关闭合时保险丝两端导通
2  电池/可调电源进 HAT 电源输入。量 HAT 的 5V 输出（40pin pin 2/4）：5.0V ± 0.2
3  先别插 Pi，确认 5V 极性正确
4  HAT 对插到 Pi 的 40pin，上电，SSH 能进
5  再接舵机总线的 V
```

⚠️ **HAT 供电时，不要再从 Pi 的 USB PWR 口插充电器**——两路 5V 会顶牛（replica 调试记录原话："HAT 供电时不要同时用普通 USB 数据线接电脑引入第二路 5V"）。URT-2 插 OTG 口是从 Pi **取**电，不冲突。

*为什么没有 UBEC 了*：HAT 的 AP63205 buck 就是干这个的，而且官方设计就是"舵机电源口即整机电源输入"。不打 HAT 才需要 UBEC 飞线（备用方案见 `microduck-replica/docs/不打HAT.md`）。

### 4.2 舵机总线

```
URT-2 USB-C ──► Zero 2W 的 USB OTG 口（micro-USB OTG 转接）
URT-2 TTL 口 S ──► 舵机总线 S
URT-2 G      ──► 共地
舵机总线 V   ──► HAT 舵机电源（电池原压）
```

- **HAT 上的 TTL/485 舵机接口留空**：它接的是 40pin UART（GPIO 14/15），Pi 上这个串口被蓝牙占用（阶段二 2.5）。我们的总线走 USB。
- **电池原压别进 URT-2 的 V1 端子**：V1 与 USB 5V 有隔离，但不是功率通路，7.4 V 经它有反灌风险。舵机电源从 HAT/电池走 18AWG。
- URT-2 拨码在 5V 电平档。

上板验证：

```bash
dmesg | tail        # 插 URT-2 后应看到 ch343 ... ttyACM0
ls -l /dev/ttyACM0
```

**总线分支**（头颈/左腿/右腿三路，半双工并联，菊花链只是走线方便）：

```
URT-2 TTL ──┬──► 头颈  30 → 31 → 32 → 33 → 34
            ├──► 左腿  20 → 21 → 22 → 23 → 24
            └──► 右腿  10 → 11 → 12 → 13 → 14
            （imu_to_dxl 小板 ID 200 挂任一路的末端或中间）
```

**V 和 G 用 18AWG 单独走到三个支路**，S 随便走。⚠️ **三路舵机电流别全走 imu_to_dxl 小板**——它的座子虽并联，但 2 层板直通铜皮没按几安培设计，会烧。它只过信号和自身用电。

### 4.3 imu_to_dxl 小板：烧固件、挂总线 ✅

**烧固件**（做一次）：

```powershell
# Windows + CMSIS-DAP + OpenOCD（⚠️ 别用 Keil UV4 命令行，会假成功）
openocd -f interface/cmsis-dap.cfg -f target/stm32g0x.cfg -c "adapter speed 4000" `
  -c "program imu_to_dxl.hex verify reset exit"
```

固件选择：
- **replica `hardware/imu_to_dxl/firmware-1.0.0/`（飞特 FT-SCS 协议）**：有现成 hex，与 15 颗舵机同一条 SYNC READ 读回，被 20261023c 起的运行时自动识别（`imu_protocol="auto"`）。**推荐**。
- **kissqy 的 FT6 固件**：他的原版 robotd 认这个。配合 kissqy 原版固件包时用。

刷完用 servo-web 或 kissqy 中控验证：总线上能看到 ID 200，IMU 数据在动。

**硬性排位规则**：主程序 SYNC READ 的 ID 列表里，**200 必须排在舵机之后**（末位/中位），禁止首位。新版主程序自动这么排，手工工具调试时注意。

**安装方向**：小板装在躯干上的朝向决定 mount 参数。装好后必须做"安装方向确认"（阶段八 8.3），换模块或重装后要重做。

### 4.4（备胎）BNO085

总线小板出问题时的替补：接 Pi 的 i2c-1（pin 3 SDA / pin 5 SCL / 3.3V / GND），配合 fork 的 `[trunk_imu] source="bno08x"`（阶段三 3.6）。注意 **BNO085 切 `bno08x_fusion = "gyro"`**——实测芯片自带姿态走路时会跳变（"走两三步就摔"），陀螺积分姿态是 fork 里专门修的。平时不接，放着备用。

### 4.5 整机接线检查单（上电前最后过一遍）

- [ ] 保险丝装好、开关断开
- [ ] HAT 5V 输出 5.0±0.2V、极性正确，才对插 Pi
- [ ] 舵机总线 V 是电池原压（6.6–8.4V），**没有**经过降压
- [ ] 所有地共在一起：电池负、HAT、Pi、URT-2 G、总线 G
- [ ] URT-2 拨码在 5V 电平档；USB 插 OTG 口
- [ ] HAT 的 TTL/485 总线口空着
- [ ] imu_to_dxl 小板挂支路末端/中间，不过大电流；固件已烧
- [ ] 每颗舵机插头方向对（中间是 V）
- [ ] 台架阶段用可调电源（限流 3–5A，7.4V）代替电池

**阶段四验收**：`dmesg` 看到 ttyACM0；SSH 正常；总线 15 颗 + ID 200 在调试台/中控里全部可见。

---

## 阶段五 · 配置要点 ✅/📋

> kissqy 体系的配置与官方 pollen 不同：配置在 `/etc/robot/feetech-ft5/` 下，**双份**（`robotd.toml` + `robotd.saved.toml`，后者是升级/重建配置时的来源），改配置最好走中控或它自带的 configure 工具，别徒手 sed——同一节里同一个键写两行，TOML 就坏了，所有版本的 robotd 都会在启动第 1 步退出。

### 5.1 必须确认的项

| 项 | 值 | 为什么 |
|---|---|---|
| 总线端口 | `/dev/ttyACM0` | kissqy 默认是 Radxa 的 ttyS2，URT-2 要改（阶段三 3.3 已做） |
| IMU 来源 | 总线小板（`bus`，默认） | 你的 imu_to_dxl 小板就是它等的目标；**不写 `[trunk_imu]` 就是总线模式** |
| 标定 | 零点/方向/限位（阶段六） | **别把样机标定直接抄过来**——每只鸭子都要自己标（kissqy 原话） |

### 5.2 不要动的项

- **舵机 P/D/转矩**：V167 默认上力时写 P=5、D=0、转矩限制 1000（"治抖"版）。先用默认跑通；想调，用 fork 的 `service.sh servo-parameters`（阶段三 3.6），别在 FD 里把 EEPROM 改成奇怪的值。
- **音频**：Pi 上没有 aic3104 的设备树配置，保持关闭/忽略（附录 C）。

### 5.3 掉电保护

机器人运行时电池到 6.6 V 左右会触发保护性坐下/关机（安全设计）。**台架用可调电源时电压要 ≥ 7 V**，否则会莫名"坐下"。

---

## 阶段六 · 中位校准 ✅

> 目标：把"笔直站立"标定为每颗舵机的 2048。
> 出处：`microduck-replica/调试记录.md` 第 9 步。另有独立精编版：`HD1910中位校准教程.md`（本目录）。
> ⚠️ 这一步做错，后面策略全军覆没。别急。kissqy 中控里也有标定流程，与本节等效，选一个用。

### 6.1 先立概念

**零点 = 机器人笔直站立的姿态**：腿是直柱、脚平放、脖子竖直、头平视。对应 MJCF 模型所有关节 = 0，对应舵机读数 **2048**。

**站姿不是零点**：`DEFAULT_POSITION` 站姿是零点加偏移（髋 pitch ±26°、踝 ∓26°、颈/头 pitch +20°）。**别把站姿校成 2048。**

换算：4096 步 / 360°，**1° ≈ 11.4 步**。

### 6.2 装舵盘：先转 2048 再装

每颗舵机、每个关节：

```
1  舵机接 PC（URT-2），FD 里指令它转到 2048
2  断电，把舵盘按"关节在直位"的朝向装上（齿位对不准就装最近的齿）
3  整机装好后，用手把关节摆到"直"的位置扶着
4  FD 编程页 →「中位校准」→ 把当前位置定义为 2048
5  锁紧螺丝
```

### 6.3 整机联标：官方折叠（头颈摆正）基准

整机装完，直站靠手摆不准——用折叠基准，因为**膝盖折叠是零件顶住的接触点，每次摆都一致**。

工具：servo-web 调试台（0.15.0+）或 kissqy 中控的标定页。

**摆法（全程扭矩关闭，用手摆）：**

- 膝：折到**踝关节刚贴上大腿舵机**（零件顶住，每次都一样）
- 髋 pitch：直角尺摆到与躯干垂直
- 髋 yaw / roll、踝：0°
- 头颈：**摆正**（脖子竖直、头平视、嘴闭上）——不用官方 FOLD 的 57°，那不是接触点

**目标值**（方向全 −1 时）：

| 关节 | ID | 目标值 |
|---|---|---|
| 左髋 pitch | 22 | **1025** |
| 左膝 | 23 | **1025** |
| 右髋 pitch | 12 | **3071** |
| 右膝 | 13 | **3071** |
| 其余 11 颗 | — | **2048** |

**写入与检查：**

- 确认框逐颗列「现在 → 写成、差几度」，差 30° 以上逐颗再确认
- **两种 180° 要分清**：装舵盘转半圈（校准正好纠回，正常）vs **方向 ± 配反**（校进去零点就是错的）——先用手掰关节，看调试台/中控 3D 模型的转向跟实物一致再校
- 写入舵机 EEPROM，掉电不丢；零点依然是 2048，软件不加补偿
- **校完扭矩仍是关的**：手动展开机器人，确认无异常再开扭矩

*校准指令的底*：飞特 **0x0B 位置校准指令（带目标值）**，偏移舵机自算自存 EEPROM（寄存器 31，第 11 位符号）。**HD-1910（固件 3.46）不认老式"扭矩寄存器写 128"**，别抄网上 STS3215 的老教程。

### 6.4 行程验证

逐颗从 2048 慢慢拉到下表两端，看结构件碰不碰、线扯不扯。**碰到就记下那颗的实际角度——那才是这台机器的真实行程**。

软件限位（MJCF `robot_walk.xml`，2048 为零点）：

| ID | 关节 | 行程 | 舵机值 |
|---|---|---|---|
| 10 / 20 | 髋 yaw | 右 −30~+25° / 左 −25~+30° | 右 1707~2332 / 左 1764~2389 |
| 11 / 21 | 髋 roll | ±22° | 1798~2298 |
| 12 / 22 | 髋 pitch | ±90° | 1024~3072 |
| 13 / 23 | 膝 | ±90° | 1024~3072 |
| 14 / 24 | 踝 | ±90° | 1024~3072 |
| 30 | 颈 pitch | −90~+60° | 1024~2731 |
| 31 | 头 pitch | ±90° | 1024~3072 |
| 32 | 头 yaw | ±170° | 114~3982 |
| 33 | 头 roll | ±25° | 1764~2332 |
| 34 | 嘴 | −5°（闭）~+30°（开） | 1991~2389 |

### 6.5 方向表

15 颗方向全 −1（HD-1910 与 XL330 转向相反，replica 实测）。servo-web 调试台有「复制 robotd 方向表」按钮可直接生成配置行。

**阶段六验收**：手摆直站时各关节读数在 2048 ± 30 步（±2.6°）以内；每颗拉过行程两端并记录实际限位；断电重启后零位保持。

---

## 阶段七 · 台架验收清单 ✅

> 出处：飞特适配架构文档 §5 的 12 项。**鸭子全程吊在吊架上，先低扭矩。**

| # | 验什么 | 怎么验 | 通过标准 |
|---|---|---|---|
| 0 | 寄存器底账 | FD 导出一颗的 0–90（阶段一 1.8 已做） | 表已存档 |
| 1 | 物理层通 | 调试台/中控 ping ID 10–34 | 15 颗全应答 |
| 1b | 转接板有无回显 | 逻辑分析仪或裸 `cat /dev/ttyACM0` 看发出的指令是否原样回到 RX | 无回显 |
| 2 | 协议读通 | sync_read 15 颗位置 | 一次读全回，0 校验错 |
| 3 | 读写往返 | 写 2048±200，读回 | 误差 ≤ 3 步 |
| 4 | tick 时序 | 30000 次 sync_read + sync_write | 每次 < 6 ms，超时/校验错为 0 |
| 4b | sync_read 应答顺序 | ID 列表最前放不存在的 ID；**确认 200 排舵机后** | 记录行为 |
| 5 | 单位 | 匀速转一圈计时对速度寄存器；万用表对电压 | 速度换算误差 < 5% |
| 6 | 方向 | 每颗 +30°，看转向（阶段六已做） | directions 表已生成 |
| 7 | 增益 | 默认 P5/D0/1000 先跑；抖再调 | 站姿不持续振荡 |
| 8 | 过载恢复 | 手堵一颗触发保护 | 卸力/重启舵机后恢复 |
| 9 | robotd 全链路 | 中控连接 | 关节数据 50 Hz 刷新 |
| 10 | **总线 IMU（ID 200）** | 翻转机身，看姿态数据 | IMU ready，gravity 跟着变；三姿态对（直立/前倾90°/侧躺） |
| 11 | 吊架站立 | HOME | 2 s 站到 DEFAULT_POSITION，不抖 |

---

## 阶段八 · 手柄、安装方向确认与首次站立 ✅

### 8.1 配对手柄

按中控的手柄页面指引配对（GameSir G50 Pro 是 kissqy 实测款；PS4/Switch Pro 类也行）。阶段二 2.4 写的 `UserspaceHID=false` 是这步的前提。

### 8.2 首次 HOME（吊架）

```
1  机器人吊在吊架上，四脚离地
2  中控或命令行执行 HOME
3  观察：2 秒内应平滑插值站到 DEFAULT_POSITION
4  不抖、不抽、无报错 → 保持 10 秒
5  异常 → 立刻卸力（或断电），查方向表和零位
```

### 8.3 IMU 安装方向确认（必须做）

**没确认过安装方向的 IMU，运动模式下拒绝上力**。kissqy 中控有「保存安装确认」（总线 IMU 用它就行）；用 fork + I2C 模块时走 SSH 的 `verify-imu`（模块要 20 秒，中控 15 秒超时会失败）：

```bash
sudo bash /opt/robot/feetech-ft5-r5/service.sh disable      # 调试模式，会卸力
sudo bash /opt/robot/feetech-ft5-r5/service.sh bus verify-imu --confirmed
sudo bash /opt/robot/feetech-ft5-r5/service.sh enable       # 回运动模式
```

过程：鸭子**直立**（离竖直 < 5°，用垫块/支架撑住，手扶很难做到），再在 20 秒内**往前低头 15°–30° 并保持**，等命令返回。**换模块、拆装过，都要重新确认。**

### 8.4 玩起来

到这里你有一只能站、能被手柄遥控走/坐/捡/踢/滚翻的鸭子（V167 随包 10 份模型）。**先慢速、近距离、软地面**，逐个小动作试。

---

## 阶段九 ·（进阶）自训模型 📋

> **开箱不需要这一步**——V167 的 10 份模型就是 HD-1910 的。想训自己的动作（新步态、新 trick）才走这里。

两条路：

1. **kissqy 训练中控 R1.5.27**（推荐）：Windows 解压 `start-training.cmd`，可视化训练任务、仿真评估、ONNX 导出，与机器人中控一套体系。
2. **microduck_rl / replica training**：`uv run train <TASK_ID>`，先跑 5 迭代 smoke test；执行器用 HD-1910 的 BAM M6 参数（`microduck-replica/docs/HD-1910-M6参数来源.md`、训练前数据清单）；61 维入 / 14 维出契约不动；导出时归一化器必须烘焙进 ONNX。

*一个隐蔽坑*：观测里 20–34 维是关节速度——速度单位换算错一个常数，策略不报错只会"走得难看"。所以阶段七第 5 项（单位验收）必须先过。

---

## 附录 A · 故障排查表

| 现象 | 最可能的原因 | 怎么办 |
|---|---|---|
| 一直"等总线" | 总线物理不通 / port 没改 / URT-2 没认出 / **总线上没有 IMU 小板**（kissqy 原版必须读到 ID 200） | `ls /dev/ttyACM0`；`dmesg`；共地；先插 USB 再上舵机电；确认小板已烧固件且挂总线 |
| missing N（部分） | 那颗 ID 烧错 / 支路断线 | 调试台逐支路 ping；重搜 ID |
| 每笔写都超时 | 舵机应答级别被改成 0 | FD 编程页查应答级别 = 1 |
| 能通信但位置指令没反应 | 运行模式不是 4 | FD 查模式 = 4 |
| 某颗不跟随 | 过载/过热保护锁存 | 卸力→重新 HOME；或舵机断电重启 |
| IMU 数据不动 | 小板固件没烧上（UV4 假成功）/ 排位错 / 协议不对 | OpenOCD 重烧；确认 200 在舵机后；`imu_protocol` 与固件匹配 |
| 运动模式拒绝上力 | IMU 安装方向没确认 | 做 8.3 的 verify-imu |
| 站着莫名坐下/关机 | 电池到 6.6V 保护 / 台架电源 < 7V | 充电；台架电源调高 |
| Pi 莫名重启 | 舵机启动压降 | 查 HAT 供电；电池电量；别和 USB 供电混插 |
| robotd 启动第 1 步就退出 | robotd.toml 里同键写了两行 | 用中控/configure 工具修，别手 sed |
| 点了中控「安装/升级固件」后 fork 功能没了 | 被装回 kissqy 原版 | 重跑 fork 的 imu-module-install.sh |
| 换模块/拆装后姿态不对 | 安装方向失效 | 重做 8.3 |

## 附录 B · 常用命令速查

```bash
# 状态
systemctl status robotd
journalctl -u robotd -b --no-pager

# 调试台（microduck-replica/tools/servo-web）
python server.py   # 浏览器 http://<Pi的IP>:8000

# fork 体系（装了 fanhao375 fork 才有）
sudo bash /opt/robot/feetech-ft5-r5/service.sh report
sudo bash /opt/robot/feetech-ft5-r5/service.sh reconnect   # 卸力+重启 robotd+拉起 padd（别直接 systemctl restart robotd——不卸力）
sudo bash ~/replica/imu-module-install.sh check            # 看装的是哪一版

# 舵机
robotctl robot reboot-motors    # 若该体系提供；或 FD 里复位
```

## 附录 C · 已知限制与声明

1. **Zero 2W 无视觉**：不含相机/WebRTC/ToF（512 MB 内存风险项）。要视觉换 Radxa Zero 3W 走 kissqy 镜像（纯 ✅）。
2. **HAT 上的音频/ToF 在 Pi 上暂缓**：官方只写了 Radxa 的设备树 overlay。音频 codec（TLV320AIC3104）在 Linux 有主线驱动，写个 Pi overlay 是可行的进阶项目；HAT 的关机检测（B1 起）在 Pi 上可接 `dtoverlay=gpio-shutdown`（具体信号脚以 HAT 原理图为准）。两者都不影响走路。
3. **阶段三（固件包装到 Pi OS）是全教程 📋 程度最高的环节**：固件为 Radxa 镜像打包，Pi 上照装时跳过 overlay/内核相关步骤。遇到与包内说明冲突，以包内说明为准并记录差异。
4. **标定是逐机的**：任何"样机标定"都不能直接抄到另一只鸭子上（kissqy 原话）。
5. **HAT 版本差异**：C1 载 BMI088、D1 载 LSM6DSV16X（都是 head_imu，默认关，走路不用）。核对你手上的版本，D1 的 I2C 地址占用（0x6A）与外挂 LSM6DSV16X 模块（须 0x6B）避开即可。

---

## 附：本教程信息出处

| 内容 | 出处 |
|---|---|
| kissqy 体系 / 固件包 / 中控 / 随包模型 | `kissqy/Microduck-kissqy` README、release-assets |
| IMU 模块 / 调参 / UDP 手柄 / 安装脚本 | `fanhao375/Microduck-kissqy` IMU-MODULE.md |
| 总线 IMU 自动识别 / 固件 1.0.0 | `microduck-replica/docs/HD1910-20261023-总线IMU自动识别.md`、`hardware/imu_to_dxl/firmware-1.0.0/` |
| 舵机 FD 配置 / ID 表 / 中位校准 | `microduck-replica/调试记录.md`（第 1–9 步） |
| 接线 / 电源 / 不打 HAT 备用方案 | `microduck-replica/docs/不打HAT.md` |
| HAT 规格 | `pollen-robotics/elec_RPI_Robot_HAT` README 与原理图 |
| 训练 / 参数 | `microduck-replica/docs/HD-1910训练前数据清单.md`、`HD-1910-M6参数来源.md` |
| 台架验收 12 项 | `microduck-replica/software/飞特适配架构.md` §5 |
