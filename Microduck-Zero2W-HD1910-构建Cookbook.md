# Microduck 构建 Cookbook：树莓派 Zero 2W + 飞特 HD-1910

> 目标读者：手头有 15 颗飞特 HD-1910-C001 和一块树莓派 Zero 2W，想从零把 Microduck 做起来的人。
> 读者不需要读过四个仓库的源码——每一步都给出命令、预期输出和"为什么这么做"。
>
> **验证状态图例**：✅ = replica 项目真机验证过；📋 = 从代码/文档推出、路径明确但本组合（Zero 2W）没人实测过，做的时候留意。
>
> 内容来源：`pollen-robotics/microduck`（官方运行时）、`fanhao375/microduck-replica`（飞特复刻，全部实测记录）、`pollen-robotics/microduck_rl`（训练）。关键出处随文标注，方便回查。

---

## 阶段零 · 先看清全局

### 0.1 你要造的是什么

Microduck 是一只 ~25 cm 高、~800 g 的双足鸭子机器人。它的本质是一个 **50 Hz 实时控制环**：

```
每 20 ms：
  从总线一次读出 15 颗舵机的位置/速度/电压/温度 + IMU 姿态
  拼成 61 维观测向量 → 喂给 ONNX 神经网络策略 → 得到 14 维关节目标
  一次写回 15 颗舵机的目标位置
```

走路、站立、前滚翻都是训练出来的神经网络（策略）在驱动，不是手工编程的动作。所以这套构建分两半：**把硬件和通信跑通**（阶段一~八）和**让策略学会用你的身体走路**（阶段九）。

### 0.2 系统架构（你最终要接出来的样子）

```
                        ┌──────────────────────────────────┐
  蓝牙手柄 ──BLE──►    │ 树莓派 Zero 2W                    │
  (PS4/Switch Pro 类)   │  ├ robotd（50Hz 控制环+策略+安全）│
                        │  ├ btd/padd（蓝牙手柄）          │
                        │  ├ configd（WiFi）/ updaterd     │
                        │  └ BNO085 IMU ◄── I2C (pin 3/5)  │
                        └───────┬──────────────────────────┘
                                │ USB
                        ┌───────▼──────────┐
                        │ FE-URT-2 调试板   │  ← 配舵机用的那块，配完直接挪进机器人
                        └───────┬──────────┘
                                │ 1Mbps 单线半双工 TTL（S 线）
              ┌─────────────────┼─────────────────┐
        头颈支路            左腿支路            右腿支路
      30→31→32→33→34    20→21→22→23→24    10→11→12→13→14

  NP-F550 电池(2S, 6.6–8.4V)
       ├──► 开关 ──► 保险丝5A ──┬──► 舵机总线 V（原压直供，18AWG）
       │                       └──► UBEC 5V/3A ──► Zero 2W (pin 2/4)
       └──► 共地：电池负极 · UBEC 输出地 · Pi pin 6 · 舵机总线 G
```

**为什么舵机吃电池原压**：HD-1910 额定 7.4 V，正好匹配 2S 锂电池（标称 7.4 V / 满电 8.4 V）。官方 HAT 也只给主控降压，舵机同样直挂电池。多一级降压只会带来发热和压降。

**为什么舵机总线走 USB（URT-2）而不是 Pi 的 GPIO 串口**：Zero 2W 唯一的全功能串口（PL011）默认被蓝牙占着，而蓝牙要留给手柄——手柄是这台机器人唯一的遥控器。URT-2 插 USB 就是 `/dev/ttyACM0`，一根线解决，还不占任何 GPIO。✅（replica 在 Radxa 上就是这么跑的）

### 0.3 物料清单

**机器人本体**

| 物料 | 规格要点 | 为什么 |
|---|---|---|
| 树莓派 Zero 2W | 配 2×20 排针（需焊接） | 主控。4×A53 + 512 MB，跑控制环和策略够用（50 Hz 控制环当年就是在 Zero 2W 上定的） |
| 飞特 HD-1910-C001 ×15 | 1.47 N·m @7.4V，4096 步/圈 | 执行器 |
| 飞特 FE-URT-2 调试板 | USB-C，带 TTL 口 | 一物两用：先在电脑上配舵机，配完挪进机器人当 USB 转接板 |
| BNO085 IMU 模块 | I2C 接口，3.3 V | 姿态反馈，策略观测的核心来源。官方方案 IMU 挂舵机总线，飞特方案直飞主控 I2C |
| NP-F550 电池 + 电池仓 | 2S 锂电 | 整机电源 |
| UBEC 降压模块 | 5V/3A，**输入下限 ≤6 V**，同步降压 | 给 Zero 2W 供电。⚠️ 很多标"2-6S"的实际 7 V 起，电池快没电时先掉压重启；别买 LM2596 |
| 保险丝座 + 5A 保险丝 | 5×20mm 快熔 | 舵机短路时保电池和线 |
| 船型开关 | KCD1，≥6A | 总电源开关 |
| 18AWG 硅胶线 | 红黑各 1m | 电源和总线 V/G 主干。15 颗堵转能到十几安，细线会烧 |
| PH2.0 3P 端子线 ×5 | 2.0mm 间距，单头 | 总线分支 |
| 杜邦线 母对母 | 20cm | I2C 和逻辑接线 |
| 470µF/16V 电解电容 | 105°C | 并在 UBEC 输入端，吸收舵机启动压降 |
| 高耐久 microSD ×2 | 32–64 GB，High Endurance | 一张用一张备。机器人硬断电多，普通卡容易坏 |
| 结构件 | 3D 打印，**飞特版 CAD**（replica 仓库改过 8 个件适配 HD-1910 尺寸） | 官方 XL330 的件装不上飞特 |
| 蓝牙手柄 | PS4 / Switch Pro 兼容 | 遥控 |

**台架工具**

| 工具 | 用途 |
|---|---|
| Windows PC | 跑飞特 FD 上位机（配 ID、中位校准） |
| 舵机转接线 2.0→2.5 3P（或一根剪开的舵机线） | URT-2 的 TTL 口是 2.54mm，舵机原装线是 2.0mm，**插不上**，必须转接或飞线 |
| 可调直流电源（能限流到 3–5 A） | 台架阶段代替电池，安全 |
| 万用表 | 每接一段电都要量 |
| 直角尺 | 中位校准摆髋 pitch |
| 标签纸 | 15 颗舵机长得一模一样 |
| 吊架（或把机器人吊起来的任何架子） | 首次上力必须吊着，防止自残 |
| 一台能交叉编译的电脑（Linux/macOS，或 Docker） | Zero 2W 的 512 MB 内存编不动 Rust，编译都在别的机器上做 |

### 0.4 软件清单（先认清谁是谁）

| 软件 | 在哪跑 | 干什么 |
|---|---|---|
| 飞特 FD 上位机 | Windows PC | 舵机烧 ID、开保护、中位校准。仓库里有：`microduck-replica/tools/飞特/`（FD 1.9.8.5 + 驱动） |
| Raspberry Pi OS Lite 64-bit | Zero 2W | 操作系统 |
| 官方 install.sh | Zero 2W | 装官方运行时（守护进程 + 更新系统） |
| `fanhao375/microduck` **feetech 分支** | 交叉编译 → Zero 2W | 含飞特协议支持的运行时，**本教程的核心** |
| servo-web 调试台 | Zero 2W（浏览器访问） | 舵机方向核对、姿态调试。`microduck-replica/tools/servo-web/` |
| microduck_rl | 训练机（带 GPU） | 重训走路策略（阶段九） |

### 0.5 开始前必须知道的三件事

1. **官方 ONNX 策略不能直接用在飞特上。** 官方策略是在 XL330（0.52 N·m）上训的，HD-1910 是 1.47 N·m、320:1 减速比，动力学完全不同。本教程阶段八结束时你的鸭子能**站立**（站立主要靠控制环），但**走路要等阶段九重训策略**。
2. **Zero 2W 路线放弃视觉。** 相机/WebRTC/ToF 这些官方功能在 512 MB 内存上是风险项，本教程全部关闭。想要视觉请换 Radxa Zero 3W（replica 主线，有现成镜像）。
3. **教程里 Zero 2W 特有的部分是 📋 推断**（系统配置、I2C 设备名、性能表现），飞特相关的部分全部 ✅ 验证过（在 Radxa 上）。遇到不一致，以你板子上的实测为准——并且欢迎回来改这份文档。

---

## 阶段一 · 舵机台架配置（Windows PC + FD 上位机）✅

> 目标：15 颗 HD-1910 每颗烧好 ID、打开保护、贴上标签。
> 出处：`microduck-replica/调试记录.md`「舵机配置 · 拿到货一步一步做」，全部实测。
> 耗时：约 40 分钟（一颗约 2 分钟，加上第一次熟悉软件）。

**为什么先配舵机再碰主控**：15 颗舵机出厂 ID 全是 1，接在一条总线上会互相撞包，一颗都认不出来。必须一次接一颗、改 ID、贴标签。这一步在电脑上做，用 FD 上位机，跟树莓派无关。

### 1.1 装 FD 软件和驱动

1. URT-2 插电脑 USB。打开设备管理器，「端口 (COM 和 LPT)」里应出现 **USB-Enhanced-SERIAL CH343 (COMx)**，记下 COM 号。
   - *为什么*：URT-2 用的是 CH343 芯片，Win10/11 自带驱动。没出现才装 `microduck-replica/tools/飞特/CH340_URT.rar`（那是老款 URT-1 用的）。
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

**URT-2 上的拨码开关是 TTL 信号电平（3.3/5 V）选择，不是供电切换。拨 5 V**（HD-1910 认 2–5 V，余量大）。

三种供电方式：

| 方式 | 接法 | 什么时候用 |
|---|---|---|
| USB 5 V | 什么都不接，V1 自带 5 V | **只配 ID、改参数**（本阶段）。空载够用 |
| 可调电源 → V1 端子 | + 进 V1、− 进 G | 15 颗一起挂的总线调试 |
| 可调电源直供舵机 | + 接舵机 V，− 接舵机 G 和 URT-2 的 G（共地），V1 空着 | 不想让电流过 URT-2 端子时（标称 6 A） |

⚠️ 三条军规：
- **别超 8.4 V**——那是 HD-1910 的电压上限，零余量。别在满电电池上做堵转测试。
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

搜不到，按顺序查：驱动 → COM 口 → 波特率是不是 1000000 → S 线接触（临时插孔最可疑）→ S/G 是否接反。

### 1.5 改参数（每颗都做）

切到「编程」页（EPROM）。**只改这几项，别的别动：**

| 参数 | 出厂 | 改成 | 为什么 |
|---|---|---|---|
| **ID** | 1 | 按下表 | 运行时按 ID 找关节，烧错关节就安反了 |
| **过流保护** | 关 | **开** | 装机后堵转是常事，不开就是烧舵机 |
| **过压保护** | 关 | **开** | 8.4 V 满电落在 4–10 V 窗口内，不会误触发 |
| 过热保护 | 开（80°C） | 不动 | 出厂已对 |
| 波特率 | 1 Mbps | 不动 | 出厂就是运行时要的 |
| 运行模式 | **模式 4 纯位置 PD** | 不动 | 强化学习落地要的就是这个模式，出厂已对，千万别改成 0 |
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

*为什么*：EPROM 参数要断电重启才确定生效。这步省下的话，可能 15 颗贴完标签才发现没写进去。

### 1.7 贴标签，下一颗

标签写**关节名 + ID**（如「左膝 23」），别只写数字。拔掉，换下一颗，回 1.4。

### 1.8 顺手做：导出寄存器底账

挑一颗，在 FD「编程」页把寄存器 0–90 全部导出存档。

*为什么*：这是这颗舵机的"出厂档案"。以后任何寄存器语义疑问（电流单位、相位位、保护参数）都拿它来对，也是运行时 `check_registers` 断言的核对依据。replica 的底账存在 `docs/飞特资料/`。

**阶段一验收**：断电重上后，15 颗各自能搜到自己的新 ID；每颗身上贴着「关节名 + ID」。

---

## 阶段二 · Zero 2W 系统准备 📋

> 目标：一块刷好 Raspberry Pi OS Lite 64-bit、开了 I2C、装好 ONNX Runtime 的 Zero 2W。
> 官方部署脚本（`setup-board.sh`）整套绑定 Radxa 的设备树，**不能在 Pi 上跑**——但它做的事里只有三件跟 Pi 有关，本阶段手工做掉。

### 2.1 刷系统（Raspberry Pi Imager）

1. 电脑上下载 Raspberry Pi Imager（raspberrypi.com/software）。
2. 设备选 **Raspberry Pi Zero 2 W**；系统选 **Raspberry Pi OS Lite (64-bit)**——必须 Lite（无桌面，省内存）且必须 64 位（官方运行时只发布 aarch64，`install.sh` 会检查 `uname -m == aarch64`）。
3. 点「编辑设置」做 headless 配置：
   - 主机名：`microduck`（路由器里好认）
   - 用户：`duck`，密码自定
   - WiFi：填你家的 SSID 和密码，国家码选对（CN）
   - 服务：**开启 SSH，密码登录**
4. 烧录，插卡。

*为什么不用老办法（boot 分区放 ssh 空文件）*：新版 Imager 的自定义设置一次把用户/WiFi/SSH 全做完，那是官方推荐路径。

### 2.2 首次开机、SSH

1. 插卡上电。Zero 2W 首次开机会扩容分区并重启，**等 3–5 分钟**。
2. 路由器后台找到 `microduck` 的 IP（或试 `ssh duck@microduck.local`，mDNS 通的话）。
3. `ssh duck@<IP>`，用 2.1 设的密码登录。

**多一条命**：如果 WiFi 连不上，用 USB 线接 Pi 的 **USB OTG 口**（靠中间那个 micro-USB）到电脑，配 USB Gadget 网卡或直接插显示器+键盘（mini-HDMI）。先点亮再谈接线。

### 2.3 系统更新与基础包

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y git curl i2c-tools minisign
```

- `i2c-tools`：`i2cdetect`，阶段四验 BNO085 用。
- `minisign`：阶段三给自编译的包签名用（官方更新系统验签就是 minisign 格式）。

**把时钟先对好**：`timedatectl` 看 `System clock synchronized: yes`。板子没有 RTC 电池，时间不对会让 HTTPS/TLS 握手失败，报错还特别隐晦（官方 install.sh 专门为此等 2 分钟时钟）。

### 2.4 开启 I2C（为 BNO085）

```bash
sudo raspi-config
# Interface Options → I2C → Yes
```

或等价地，`/boot/firmware/config.txt` 里确认有 `dtparam=i2c_arm=on`，然后重启。

验证：

```bash
ls /dev/i2c-1     # 存在
```

*为什么*：BNO085 要走 Pi 的 I2C-1 总线（pin 3 SDA / pin 5 SCL）。官方代码默认的 I2C 设备名是 Radxa 的 `i2c-3`，阶段五配置时会改成 `i2c-1`。

### 2.5 串口：什么都不用动（解释一下为什么）

Zero 2W 有两个 UART：PL011（好）默认给蓝牙，mini-UART（弱）空着。你可能看过老教程让改 `config.txt` 换串口——**本方案不需要**：舵机总线走 URT-2 的 USB（`/dev/ttyACM0`），GPIO 串口一个都不用。蓝牙保持默认配置，完整留给手柄。

### 2.6 安装 ONNX Runtime 1.28.0

策略推理靠 ONNX Runtime。官方 `setup-board.sh` 里这部分是平台无关的，手工照搬：

```bash
cd /tmp
curl -fLO https://github.com/microsoft/onnxruntime/releases/download/v1.28.0/onnxruntime-linux-aarch64-1.28.0.tgz
tar xzf onnxruntime-linux-aarch64-1.28.0.tgz
sudo cp -a onnxruntime-linux-aarch64-1.28.0/lib/libonnxruntime.so* /usr/local/lib/
sudo ldconfig
```

验证：

```bash
ls -l /usr/local/lib/libonnxruntime.so    # 应是指向 libonnxruntime.so.1.28.0 的软链
```

*为什么是这个版本*：运行时用的 Rust `ort` crate 硬性要求 ≥ 1.23.x，低了会直接 panic 杀死控制线程；1.28.0 是官方当前钉的版本。robotd 是 **dlopen** 加载它的（不是编译期链接），所以缺库的表现是 robotd 启动后报 health 而不是编译错误；`ORT_DYLIB_PATH` 环境变量可改查找路径。

### 2.7 蓝牙手柄的内核配置

```bash
sudo nano /etc/bluetooth/input.conf
```

写入（文件不存在就新建）：

```ini
[General]
UserspaceHID=false
```

*为什么*：这一行决定经典蓝牙（BR/EDR）手柄的报文由内核还是 bluetoothd 用户态处理。官方 `setup-board.sh` 的 `configure_classic_hid` 做的就是这件事；交给内核，padd 才能用标准 evdev 接口读手柄。改完要重启生效（本阶段结束时统一重启）。

### 2.8（建议）开 zram

512 MB 内存紧张，zram 用 CPU 换内存：

```bash
sudo apt install -y zram-tools
# 默认配置即可（压缩一半内存做 swap）
```

### 2.9 重启并验证

```bash
sudo reboot
```

回来后：

```bash
uname -m              # aarch64
ls /dev/i2c-1         # 存在
timedatectl | grep -i synch   # yes
free -h               # 能看到 zram swap
```

**阶段二验收**：能 SSH、I2C 设备出现、ONNX Runtime 就位、时钟同步。

---

## 阶段三 · 安装机器人运行时 ✅/📋

> 目标：feetech 分支的运行时以"正规军"方式装进 Zero 2W——systemd 管理、开机自启、带签名更新和自动回滚。
> 路线：先跑官方 `install.sh` 装官方版（证明安装通道通），再用自签密钥把自编译的 feetech 版推上去替换。
>
> *为什么不直接 scp 个二进制跑*：官方运行时是一整套（7 个守护进程 + systemd unit + 更新器 + 健康门 + 回滚）。手工二进制能跑通总线，但没有更新通道，以后每改一行代码都要手工折腾。一次把正规通道建好，之后 `scripts/dev-push.sh` 一条命令就能"编译→签名→安装→健康检查→起不来自动回滚"。

### 3.1 跑官方 install.sh（在 Zero 2W 上）📋

```bash
curl -fsSL https://raw.githubusercontent.com/pollen-robotics/microduck/main/scripts/install.sh -o /tmp/install.sh
sudo sh /tmp/install.sh
```

*为什么官方脚本能在 Pi 上用*：它的目标写的是"64-bit Debian userland on aarch64"，并不检查板型——绑 Radxa 的只有 `setup-board.sh`（设备树 overlay），那个我们不跑。install.sh 做的事：下载 bootstrap `updaterd` → 用它验签安装第一个正式 release 到 `/opt/robot/daemon/` → 创建 `robot` 组和 btd/padd 服务账号 → 装 systemd units → 把 `robotctl` 软链到 PATH。

⚠️ 国内网络注意：脚本要访问 `raw.githubusercontent.com` 和 `api.github.com`。不通的话给终端配代理（`export https_proxy=http://<电脑IP>:7890`）再跑。

跑完后：

```bash
newgrp robot          # install.sh 把你加进了 robot 组；当前 shell 还没继承，newgrp 刷新（或重新登录）
robotctl version      # 能看到守护进程版本
robotctl health       # 大概率报 bus 红——总线还没接，属预期，它在不停重试
```

*为什么先装官方版*：这一步把 systemd、用户组、更新器全部就位，并用一个签名完好的官方 release 证明"安装→启动→健康检查"链路是通的。之后换 feetech 版时如果出问题，你就知道问题在代码而不在部署通道。

### 3.2 关掉 Zero 2W 跑不动的守护进程 📋

官方 release 会带上全部守护进程。Zero 2W 没有相机、没有 ToF、内存只有 512 MB，关掉三个：

```bash
sudo systemctl disable --now mediad.service tofd.service nfcd.service
```

- `mediad`：相机/WebRTC。没相机时它会不停崩溃重启，GStreamer 扫描能占满一个核（replica 在 Radxa 上实测把主板顶到 89°C）。
- `tofd`：ToF 深度传感器。没接。
- `nfcd`：NFC 配对。没接。

*为什么用 disable 而不是装最小集*：release 是整包签名的，不能挑件装；但 systemd 层关掉即可，升级时会保持 disable 状态。robotd/configd/btd/padd/updaterd 全留着。

### 3.3 自签开发密钥（一次性）

官方更新器只装 `/etc/robot/trusted_keys/` 里的公钥签过的包。Pollen 团队的私钥拿不到，所以**自己生成一对**，公钥放板子、私钥留在编译电脑上。

在 **Zero 2W** 上（或编译机上先生成再分发）：

```bash
mkdir -p ~/.duck-keys
minisign -G -p ~/.duck-keys/replica.dev.pub -s ~/.duck-keys/replica.dev.key
# 设个密码（或留空）
```

公钥装上板子，并允许 dev 签名：

```bash
sudo cp ~/.duck-keys/replica.dev.pub /etc/robot/trusted_keys/replica.dev.pub
sudo chmod 644 /etc/robot/trusted_keys/replica.dev.pub
sudo sed -i 's/^allow_dev_keys.*/allow_dev_keys        = true/' /etc/robot/updater.toml
sudo systemctl restart updaterd
```

三个细节，每个都有坑（官方 `install.sh` 注释里写明了）：
- 公钥文件名**必须以 `.dev.pub` 结尾**——更新器按文件名分类 dev key 和 release key，改个名字就会被当成正式签名密钥，等于自签的包被当成"评审过的正式版"信任。
- `allow_dev_keys = true` 和公钥**缺一不可**——只做一半会得到一个看起来像"签名损坏"的报错。
- 这台板子从此会安装你用这把私钥签的任何包，**别把私钥提交进任何仓库**。

把私钥安全地拷到编译电脑上（`scp duck@<IP>:~/.duck-keys/replica.dev.key ~/.duck-keys/`）。

### 3.4 在编译电脑上准备 feetech 分支

Zero 2W 的 512 MB 内存编不动这个 23-crate 的 Rust workspace，**编译永远在其他机器上做**。编译机需要：Linux 或 macOS、Rust 1.89+（不超过 1.94.1）、以及交叉编译工具。

```bash
git clone -b feetech https://github.com/fanhao375/microduck.git
cd microduck

# 系统依赖（Linux 编译机；macOS 不需要）
sudo apt-get install -y libudev-dev libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev

# 交叉编译工具链（二选一）
cargo install cargo-zigbuild --locked   # 方式一：zig 链接器，另需装 zig
# 或者什么都不装，后面 dev-push.sh 加 --docker 用容器编
```

先本地跑测试确认代码健康（不需要硬件）：

```bash
cargo test --workspace
```

feetech 分支的测试包含飞特协议的位置换算、方向镜像、第 15 位符号、增益映射等 11 个专项测试，全绿再继续。

### 3.5 一键推送：dev-push.sh

feetech 分支里带着官方的 `scripts/dev-push.sh`。在编译电脑上：

```bash
export DUCK_DEV_SECRET_KEY=~/.duck-keys/replica.dev.key
export DUCK_BOARD_USER=duck          # 默认是 microduck，Pi 上我们的用户是 duck

scripts/dev-push.sh duck@<Zero2W的IP>
# 或走 Docker：scripts/dev-push.sh --docker duck@<IP>
```

它做的事：zigbuild 交叉编译整个 workspace → 打成和正式 release 相同结构的包 → 用你的私钥签名 → scp 到板子的 `~/duck-sideload` → 通过 `robotctl update apply --from` 走**正常更新通道**安装（带健康门：起不来自动回滚到 3.1 装的官方版）。

完成后板上验证：

```bash
robotctl version    # 应显示 0.x.x-dev.local....gb214486 之类的 dev 版本号
```

*为什么这一步值得这么做*：从现在起，"改代码 → 上板"就是一条命令，且有自动回滚兜底。这是官方团队的日常开发循环，replica 全套适配都是这么上板的。

**阶段三验收**：`robotctl version` 显示 dev 版本；`systemctl status robotd` active；`robotctl health` 里 bus 红（总线还没接）但进程活着。

---

## 阶段四 · 接线与供电 ✅/📋

> 目标：电池、UBEC、Pi、URT-2、舵机总线、IMU 全部接好。
> 原则：**每接一段量一次电压，确认极性再插。**

### 4.1 电源链路

```
NP-F550 ──► 开关 ──► 保险丝 5A ──┬──► UBEC 输入（并 470µF 电容）
                                 │         │
                                 │         └──► 5V 输出 ──► Pi pin 2（或 pin 4）
                                 └──► 舵机总线 V（原压，18AWG）
共地：电池负极 · UBEC 输出地 · Pi pin 6 · 舵机总线 G · URT-2 G
```

按这个顺序做，每步用万用表：

```
1  接开关、保险丝。量：开关闭合时保险丝两端导通
2  接 UBEC，先别接 Pi。量输出：5.0V ± 0.2，极性对
3  ⚠️ 确认极性再插 Pi —— pin 2/4 的 5V 输入没有保护，接反烧板
4  Pi 拔掉 USB 供电线，靠 pin 2 的 5V 点亮，SSH 能进
5  再接舵机总线的 V
```

*为什么 UBEC 输入并 470µF 电容*：15 颗舵机同时启动的瞬间电池电压会跌，电容吸收这个压降，防止 Pi 复位。走路时如果 Pi 莫名重启，第一怀疑对象就是它（加大电容或查 UBEC）。

### 4.2 舵机总线

URT-2 的角色从"调试板"切换为"机上转接板"：

```
URT-2 USB-C ──► Zero 2W 的 USB OTG 口（用 OTG 线/转接头）
URT-2 TTL 口 S ──► 舵机总线 S
URT-2 G      ──► 舵机总线 G（共地）
舵机总线 V   ──► 电池原压（4.1 接好的）
```

⚠️ **电池原压直接给舵机，别进 URT-2 的 V1 端子**——V1 与 USB 5V 有隔离设计，但这不是功率通路，7.4 V 经它有反灌进 Pi 的风险。舵机电源从电池/保险丝直接走 18AWG。

上板验证 CH343 被识别：

```bash
dmesg | tail        # 插 URT-2 后应看到 usb ... ch343 ... ttyACM0
ls -l /dev/ttyACM0  # 存在
```

**总线分支**：头颈、左腿、右腿在躯干分三路。半双工单线总线是**并联**，星形树形菊花链都行，菊花链只是走线方便：

```
URT-2 TTL ──┬──► 头颈  30 → 31 → 32 → 33 → 34
            ├──► 左腿  20 → 21 → 22 → 23 → 24
            └──► 右腿  10 → 11 → 12 → 13 → 14
```

分法三选一：URT-2 自带两个 TTL 口各出一路、第三路从某颗舵机再分；买「舵机总线分线板 3P」一进四出（约 ¥5）；或三根线并焊。**V 和 G 用 18AWG 单独走到三个支路**，S 随便走。

### 4.3 BNO085 IMU 接 I2C 📋

```
BNO085 模块      Zero 2W
VIN/3V3   ──►   pin 17 (3.3V)     （pin 1 也可以；别接 5V）
GND       ──►   pin 6 (GND)
SDA       ──►   pin 3 (GPIO2/SDA1)
SCL       ──►   pin 5 (GPIO3/SCL1)
```

Pi 的 I2C-1 板载 1.8kΩ 上拉，模块自己一般也带上拉，不用外加。

上电验证：

```bash
sudo i2cdetect -y 1
# 应在 0x4A（或 0x4B，取决于模块 SA0 引脚）看到一个占用
```

*为什么 IMU 不挂舵机总线*：官方 XL330 方案的 IMU 是一块 `imu_to_dxl` 小板挂在总线上当 ID 200，和舵机同一次 sync_read 读回。飞特版本的小板固件是另一个项目（STM32，两三周工作量），replica 选择的是**方案 B：BNO085 模块直飞主控 I2C**，运行时开一个 100 Hz 线程读它，控制环每 tick 取最新一帧。官方原型机在 Zero 2W 时代就是 I2C 接 BNO055 跑过来的，有先例。

⚠️ 已知风险：BNO085 重度使用 clock stretching（拉低 SCL 等数据），个别 SoC 的 I2C 控制器对此不友好。**接上后先跑 10 分钟连续读取看丢帧**；实在不稳，BNO085 可以改 UART-RVC 模式（100 Hz 固定输出），但那就得占用 Pi 的串口，跟蓝牙抢，不推荐。先做 I2C，大概率能行。

### 4.4 整机接线检查单（上电前最后过一遍）

- [ ] 保险丝装好、开关断开
- [ ] UBEC 输出 5.0±0.2V、极性正确，才接 Pi
- [ ] 舵机总线 V 是电池原压（6.6–8.4V），**没有**经过 UBEC
- [ ] 所有地共在一起：电池负、UBEC 地、Pi pin 6、URT-2 G、总线 G
- [ ] URT-2 拨码在 5V 电平档
- [ ] BNO085 接的是 3.3V 不是 5V
- [ ] 每颗舵机插头方向对（中间是 V）
- [ ] 台架阶段用可调电源（限流 3–5A，电压 7.4V）代替电池

**阶段四验收**：`dmesg` 看到 ttyACM0；`i2cdetect -y 1` 看到 0x4A；`ls /dev/i2c-1` 存在；Pi 靠 UBEC 供电能 SSH。

---

## 阶段五 · 配置 robotd.toml ✅

> 目标：告诉运行时"舵机说飞特协议、总线在 ttyACM0、15 颗方向全反、IMU 在 I2C"。
> 配置文件在 `/etc/robot/robotd.toml`。它故意设计成"每台机器人一份、升级不覆盖"——你写的每个值都会一直留着，所以**只写需要改的，其余保持注释**（注释掉的值 = 跟随内建默认值，能随版本升级）。

用 `sudo nano /etc/robot/robotd.toml`，把下面几段改/加进去：

```toml
[bus]
# URT-2 插 USB 后的设备名。官方默认 /dev/ttyS2 是 Radxa 的接法。
port = "/dev/ttyACM0"

# 舵机说什么协议。这是 feetech 分支加的开关，默认 dynamixel2 时行为跟官方一模一样。
protocol = "feetech-sts"

# 飞特专用参数
[bus.feetech]
# 增益换算系数：别处说的 kP（XL330 量纲）× p_scale = 飞特的一字节 P。
# 0.16 是分支里的示例起点，阶段七台架标定后改成你的实测值。
p_scale = 0.16

# 每颗舵机的方向，+1/-1，顺序 = 左腿5颗、颈头嘴5颗、右腿5颗（JOINT_IDS 顺序）。
# HD-1910 读数增大的方向跟 XL330 相反，replica 实测 15 颗全是 -1。
directions = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]

# 躯干 IMU 来源（飞特总线上没有官方那块 IMU 小板）。
# 键名以你 checkout 的分支为准：kissqy 整套用 [trunk_imu] source = "bno08x"；
# 不确定就跑 robotctl configure 看它列出哪些键。
[trunk_imu]
source = "bno08x"
# 如果分支里有 i2c 设备路径的配置键，填 /dev/i2c-1（Radxa 默认是 i2c-3，Pi 是 i2c-1）。
```

再改两段跟 Zero 2W 相关的：

```toml
[policy]
# 阶段七之前先关掉策略加载：没有为 HD-1910 重训的策略，官方 ONNX 在飞特上不能走。
# 关掉后控制环照常跑、health 照常绿，只是不驱动动作——这正是调试期要的状态。
enabled = false

[audio]
# 没有 HAT 就没有 aic3104 音频芯片，关掉防止无意义的报错。
# （官方说明：没 codec 的板子走路完全一样，只是不出声。）
enabled = false
```

生效：

```bash
sudo systemctl restart robotd
robotctl health
```

**期望**：bus 一项从红变绿（15 颗全在线时）；loop 显示 50 Hz。如果 bus 还是红，看 `journalctl -u robotd -b --no-pager`，对照附录 A 排障表。

几个配置背后的"为什么"：

- **directions 为什么是 -1**：飞特的计数方向和 Dynamixel 相反。这个表是运行时读写时统一乘的符号，配错的表现是"命令它 +30° 它走 −30°"——阶段六会逐颗核对。
- **p_scale 为什么只是个起点**：XL330 的 P 是 0–16383 的 u16，飞特的 P 是 0–254 的 u8 而且在 EEPROM 区，物理量纲不同，没有理论换算值，只能台架实测。配不准的后果不是不动，是步态跟训练时不一样。
- **为什么策略先关**：[policy] enabled=false 时控制环空转但健康——这是官方设计的台架安全模式（"a legitimate configuration, not a degraded one"），装拆、断电循环时机器人不会乱动。
- **`battery_empty_shutdown` 默认开着**：电池 EMA 到 6.6 V 会自动坐下关机。台架用可调电源时**电压要 ≥ 7 V**（HD-1910 有 6 V 档，低于 6.6 V 就触发关机流程），或者调试期显式关掉它。

**阶段五验收**：`robotctl health` 的 bus 绿、loop 50 Hz 0 missed。

---

## 阶段六 · 方向核对与中位校准 ✅

> 目标：①逐颗确认舵机转向（directions 表）②把"笔直站立"标定为每颗舵机的 2048。
> 出处：`microduck-replica/调试记录.md` 第 9 步。另有独立精编版：`HD1910中位校准教程.md`（本目录）。
> ⚠️ 这一步做错，后面策略全军覆没。别急。

### 6.1 先立概念

**零点 = 机器人笔直站立的姿态**：腿是直柱、脚平放、脖子竖直、头平视。这对应 MJCF 模型所有关节 = 0，对应舵机读数 **2048**。

**站姿不是零点**：官方 `DEFAULT_POSITION` 站姿是零点加偏移（髋 pitch ±26°、踝 ∓26°、颈/头 pitch +20°）。**别把站姿校成 2048。**

换算：4096 步 / 360°，**1° ≈ 11.4 步**。

### 6.2 装舵盘：先转 2048 再装

每颗舵机、每个关节：

```
1  舵机接 PC（URT-2），FD 里指令它转到 2048
2  断电，把舵盘按"关节在直位"的朝向装上
   —— 齿位对不准就装最近的齿，误差留给下一步吸收
3  整机装好后，用手把关节摆到"直"的位置扶着
4  FD 编程页 →「中位校准」→ 把当前位置定义为 2048
5  锁紧螺丝
```

*为什么先转 2048 再装舵盘*：让机械零位和电气零位天然对齐，剩下的只是齿位误差（半齿 ≈ 几分之一圈），校准指令吸收掉。

### 6.3 整机联标：官方折叠（头颈摆正）基准

整机装完，直站靠手摆不准——用折叠基准，因为**膝盖折叠是零件顶住的接触点，每次摆都一致**，比直站可重复得多。

工具：servo-web 网页调试台（0.15.0+）。在 Zero 2W 上：

```bash
cd microduck-replica/tools/servo-web
pip install -r requirements.txt   # 或按 README
python server.py
# 浏览器打开 http://<Pi的IP>:8000
```

**摆法（全程扭矩关闭，用手摆）：**

- 膝：折到**踝关节刚贴上大腿舵机**（零件顶住，每次都一样）
- 髋 pitch：直角尺摆到与躯干垂直
- 髋 yaw / roll、踝：0°
- 头颈：**摆正**（脖子竖直、头平视、嘴闭上）——不用官方 FOLD 的 57°，那不是接触点、摆不准

**目标值**（方向全 −1 时）：

| 关节 | ID | 目标值 |
|---|---|---|
| 左髋 pitch | 22 | **1025** |
| 左膝 | 23 | **1025** |
| 右髋 pitch | 12 | **3071** |
| 右膝 | 13 | **3071** |
| 其余 11 颗 | — | **2048** |

**写入**：调试台确认框会逐颗列「现在 → 写成、差几度」。

- 差 30° 以上逐颗再确认——但**先分清两种 180°**：
  - 装舵盘转了半圈 → 差 180°，校准正好纠回来，正常；
  - **方向 ± 配反了** → 也差 180°，校进去零点就是错的。**先用手掰关节，看调试台 3D 模型的转向跟实物一致**，不一致先在页面翻那颗的方向（调 directions），再校。
- 写入后进舵机 EEPROM，掉电不丢；零点依然是 2048，软件不加任何补偿。
- **校完扭矩仍是关的**：手动展开机器人，确认无异常后再开扭矩（调试台 0.15.0 起开扭矩前自动先把目标对准当前位置，不会跳）。

*校准指令的底*：飞特 **0x0B 位置校准指令（带目标值）**——"你现在这个位置应读作 X"，偏移舵机自算自存 EEPROM（寄存器 31，第 11 位符号）。**HD-1910（固件 3.46）不认老式"扭矩寄存器写 128"**（实测不生效），别抄网上 STS3215 的老教程。

### 6.4 行程验证

逐颗从 2048 慢慢拉到下表两端，看结构件碰不碰、线扯不扯。**碰到就记下那颗的实际角度——那才是这台机器的真实行程**（比软件限位小是正常的）。

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

### 6.5 把 directions 表写回配置

servo-web 的调试台有「复制 robotd 方向表」按钮，直接生成 `directions = [...]` 那一行。粘进 `/etc/robot/robotd.toml` 的 `[bus.feetech]`，重启 robotd。

**阶段六验收**：手摆直站时，调试台/robotctl monitor 显示各关节在 2048 ± 30 步（±2.6°）以内；每颗拉过行程两端并记录实际限位；断电重启后零位保持。

---

## 阶段七 · 台架验收清单 ✅

> 出处：飞特适配架构文档 §5 的 12 项。做完这些才算"适配完成"。
> 0–8 项用 servo-web 调试台做；9 起用 robotd。**鸭子全程吊在吊架上，扭矩限制先降到 30%。**

| # | 验什么 | 怎么验 | 通过标准 |
|---|---|---|---|
| 0 | 寄存器底账 | FD 导出一颗的 0–90（阶段一 1.8 已做） | 表已存档，关键项对得上 |
| 1 | 物理层通 | Python/调试台 ping ID 10–34 | 15 颗全应答 |
| 1b | 转接板有无回显 | 逻辑分析仪或裸 `cat /dev/ttyACM0` 看发出的指令是否原样回到 RX | 无回显；有则后端要按发送长度丢弃 |
| 2 | rustypot v1 能用 | 调试台 sync_read 15 颗位置 | 一次读全回，0 校验错 |
| 3 | 读写往返 | 写 2048±200，读回 | 误差 ≤ 3 步 |
| 4 | tick 时序 | 30000 次 sync_read + sync_write | 每次 < 6 ms，超时/校验错为 0 |
| 4b | sync_read 应答顺序 | ID 列表最前放一个不存在的 ID | 记录 15 颗行为（等它超时/照答/撞包） |
| 5 | 单位 | 匀速转一圈计时对速度寄存器；万用表对电压 | 速度换算误差 < 5% |
| 6 | 方向 | 每颗 +30°，看转向（阶段六已做） | directions 表已生成 |
| 7 | 增益标定 | 阶跃响应对比，调 p_scale | 响应接近预期，定下 p_scale |
| 8 | 过载恢复 | 手堵一颗触发保护（状态寄存器 BIT5） | `robotctl robot reboot-motors` 后 1s ping 通、状态清零、ID 保留 |
| 9 | feetech 协议全链路 | `[bus] protocol="feetech-sts"` 已配，`robotctl health` | bus 绿、loop 50 Hz 0 missed |
| 10 | IMU 源 | BNO085 接上，翻转机身 | `imu ready`，gravity 向量跟着变 |
| 11 | 吊架站立 | `robotctl robot init` | 2 s 站到 DEFAULT_POSITION，不抖 |

**第 7 项增益标定的做法**：同一颗关节、同样幅度的阶跃指令，从 `p_scale = 0.16` 起步，观察响应快慢和超调，逐步调到响应干脆不振荡。飞特的 P 是"锁着写"（EEPROM lock=1 时写入生效但掉电不保存，正好等价 XL330 的 RAM 语义），所以运行中切增益不磨 EEPROM。**D/I 别动**——飞特出厂 D=32 是厂家整定的一部分，320:1 减速比去掉 D 可能振荡。

**第 10 项 IMU 验收的三个姿态**：直立（gravity ≈ [0,0,-1]）、前倾 90°、侧躺。三个姿态读数都对，安装方向（mount 四元数）才算对。若系统性偏一个方向，是安装方向配置问题，不是校准能解决的。

---

## 阶段八 · 手柄配对与首次站立 ✅

### 8.1 配对手柄

```bash
sudo robotctl pad pair
# 按提示把手柄置于配对模式（PS4：Share+PS 键长按；Switch Pro：配对键长按）
```

padd 守护进程会接管手柄。`robotctl monitor` 能看到手柄输入。

*为什么蓝牙不用额外配置*：Zero 2W 的蓝牙走 PL011，我们没动它（阶段二 2.5）；阶段二 2.7 写的 `UserspaceHID=false` 让经典手柄报文走内核 evdev，padd 直接读。

### 8.2 首次 init（吊架）

```
1  机器人吊在吊架上，四脚离地
2  确认扭矩限制已降到 30%（FD 里设，或调试台）
3  robotctl robot init
4  观察：2 秒内应平滑插值站到 DEFAULT_POSITION
5  不抖、不抽、无报错 → 保持 10 秒，看机身倾斜是否收敛
6  异常 → 立刻 robotctl robot disable（或直接断电），查方向表和零位
```

`init` 做的事：`set_torque(true)` → 写增益 → 2 秒线性插值到站姿。**进程启动时从不自己碰扭矩**——只有人按了 init 它才动，这是安全设计。

### 8.3 关于走路

到这里你有一个能站、能被手柄指挥姿态的鸭子。**走路需要阶段九的重训策略**——官方策略在 HD-1910 上不能直接跑。replica 用同样的流程训出了走/站/前滚翻策略并实机验证，路是通的，但要花一两个晚上训练。

---

## 阶段九 · 策略重训指路 ✅

> 完整训练框架在 `microduck_rl` 仓库（MuJoCo + PPO），观测/动作契约（61 维入 / 14 维出）与运行时严格对应。
> 这里给路径和关键参数出处，详细训练文档看 `microduck-replica/software/training/` 和 `docs/HD-1910训练前数据清单.md`。

### 9.1 训练前必须换的参数

**执行器模型**：训练用的 BAM（电压-力矩）执行器模型要换成 HD-1910 的实测参数。两套现成参数（陆吾版 / 飞特官方版）在 `microduck-replica/docs/HD-1910-M6参数来源.md`，训练前要填的数据清单（已填好的样板）在 `docs/HD-1910训练前数据清单.md`。

**训练环境**：`microduck-replica/software/training/` 里有 replica 改好的 HD-1910 版 mjlab 训练环境（含 `robot/hd1910/` 的 MJCF 模型和 `1910_m6.json` 参数文件）。

### 9.2 训练流程（在带 GPU 的机器上）

```bash
cd microduck-replica/software/training    # 或 microduck_rl
uv sync

# 先跑冒烟测试（64 环境 5 迭代，几分钟）——抓住 95% 的配置错误
uv run train <TASK_ID> --env.scene.num-envs 64 --agent.max_iterations 5

# 正式训练（步态/课程类任务 4000–6000 迭代）
uv run train <TASK_ID> --env.scene.num-envs 4096

# 导出 ONNX（归一化器必须烘焙进去，scripts/export.py 自动做）
uv run scripts/export.py <TASK_ID> --wandb-run-path <entity/project/run_id>

# 部署前先在仿真里彩排
uv run scripts/infer_policy.py --walking out.onnx
```

### 9.3 上机

```bash
scp out.onnx duck@<IP>:~/
ssh duck@<IP>
sudo robotctl policy load walk ~/out.onnx
# robotd.toml 里 [policy] enabled 改回 true
sudo systemctl restart robotd
```

*契约要点*：策略必须 61 维入 / 14 维出，robotd 加载时检查；观测里 20–34 维是关节速度——**速度单位换算错一个常数，策略不报错，只会"走得难看"**，所以阶段七第 5 项（单位验收）必须先过。

---

## 附录 A · 故障排查表

`robotctl health` 各项红的第一嫌疑：

| 现象 | 最可能的原因 | 怎么办 |
|---|---|---|
| bus 红，missing 15 | 总线物理不通 / port 配错 / URT-2 没认出 | `ls /dev/ttyACM0`；`dmesg`；共地；URT-2 上电顺序（先 USB 后舵机电） |
| bus 红，missing N（部分） | 那颗 ID 烧错 / 线接触不良 / 该支路断路 | 调试台逐支路 ping；重新搜 ID |
| 每笔写都超时 30ms | 舵机应答级别被改成 0 | FD 编程页检查应答级别 = 1 |
| 能通信但位置指令没反应 / 行为怪 | 运行模式不是 4 | FD 检查模式 = 4（纯位置 PD） |
| 开机报 corrected 但下次还报 | EEPROM 锁没解开就写了参数 | FD 里解锁后重写 |
| 某颗位置不跟随、状态有位 | 过载/过热保护锁存 | `robotctl robot reboot-motors`（飞特 0x08 重启） |
| init 花 1 秒以上还报错 | 应答级别 0，每笔写超时 | 同上第二行 |
| imu stale / not ready | I2C 线松 / 设备名错 / mount 方向错 | `i2cdetect -y 1` 看 0x4A；检查配置里 i2c-1；重做三姿态验收 |
| 走着走着突然坐下关机 | 电池到 6.6V 触发 battery_empty_shutdown | 充电；台架电源要 ≥ 7V |
| Pi 莫名重启 | 舵机启动压降 / UBEC 掉压 | 加大输入电容；查 UBEC 输入下限 |
| 温度高、CPU 降频 | mediad 没关 / 散热不够 | `systemctl disable --now mediad`；加散热 |
| robotctl 命令 Permission denied | 当前用户不在 robot 组 | `newgrp robot` 或重新登录 |

## 附录 B · 常用命令速查

```bash
# 状态
robotctl health                    # 整机健康（第一要看）
robotctl monitor                   # 实时：关节/IMU/重力/手柄
robotctl version                   # 运行版本
journalctl -u robotd -b --no-pager # robotd 本次启动以来日志

# 控制（都需要 sudo 或 root）
robotctl robot init                # 上扭矩，2 秒站到站姿
robotctl robot disable             # 下扭矩
robotctl robot reboot-motors       # 重启舵机清保护锁存

# 配置与更新
robotctl configure                 # 交互式改 robotd.toml（知道哪些键要重启哪个服务）
sudo systemctl restart robotd      # 改完配置重启
scripts/dev-push.sh duck@<IP>      # 编译机：改代码一键上板

# 舵机台架（调试台 http://<IP>:8000）
# 扫描 / 拖滑块 / 姿态序列 / 中位校准 / 方向翻转 / 导出寄存器
```

## 附录 C · 已知限制与声明

1. **Zero 2W 无视觉**：本方案不含相机/WebRTC/ToF/NPU 检测（512 MB 内存风险项）。要视觉换 Radxa Zero 3W 走 replica 主线（有现成镜像）。
2. **官方策略不可直接用**：XL330 动力学 ≠ HD-1910 动力学，走路必须重训（阶段九）。
3. **板型声明**：`[board] version` 只有 `zero3`/`beta` 两个合法值，Pi 不在其中。留默认即可；若 `robotctl health` 对板型有提示，属预期（这块板不在官方支持列表），不影响控制环。
4. **HD-1910 固件差异**：0x08 重启、0x0B 校准在固件 3.46 上验证过；出厂固件版本号（寄存器 0/1）建议存档，买新批次舵机时核对。
5. **本教程 Zero 2W 特定步骤为 📋 推断**：I2C 稳定性、内存表现、btd/padd 在 Pi OS 上的行为未经整机实测——官方原型当年在 Zero 2W 上用 BNO055 + 自研运行时跑通过 50Hz 控制环，方向可行；落地遇到不一致，以实测为准并欢迎修订本文档。

---

## 附：本教程信息出处

| 内容 | 出处 |
|---|---|
| 系统架构 / RobotIo 接缝 / 适配原理 | `microduck-replica/software/飞特适配架构.md` |
| 舵机配置 FD 步骤 / ID 表 / 中位校准 | `microduck-replica/调试记录.md`（第 1–9 步、2026-09-17/28 节） |
| 接线 / UBEC / 总线分支 | `microduck-replica/docs/不打HAT.md` |
| robotd.toml 全部配置项语义 | `microduck/deploy/robotd.toml`（官方注释）、fanhao375/microduck feetech 分支 |
| install.sh / setup-board.sh 行为 | `microduck/scripts/` 源码 |
| dev-push / 签名 / 更新通道 | `microduck/docs/robot/dev-push.md`、`scripts/install.sh` |
| 训练流程 / 参数 | `microduck-replica/docs/HD-1910训练前数据清单.md`、`HD-1910-M6参数来源.md`、`microduck_rl/AGENTS.md` |
| 台架验收 12 项 | `microduck-replica/software/飞特适配架构.md` §5 |
