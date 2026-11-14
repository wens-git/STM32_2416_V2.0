# 2416_F407 在线示教 · 调控轴 上位机

聚焦"**在线示教 + 多轴 JOG 调试**"的上位机。基于 PySide6 + pyserial，通过 Modbus RTU (115200-8N1) 与 STM32F407 2416 板卡通信。板卡提供两个独立串口通道，接到任一口即可：

- **RS485**（USART3，PC10/PC11，方向脚 PB14）：固定 Modbus RTU，推荐用于调试。
- **RS232**（USART1，PA9/PA10）：默认 Modbus RTU；当固件寄存器 `220` 写 1 时该口切换为自由口，此时调试上位机需改接 RS485。

两端口共用同一套寄存器；`207` 从站地址与 `212`/`213` 帧统计按端口独立。

当前协议按 8 轴容量设计：

- 固件版本握手值 `0x2416`
- 轴容量 `8`（4 硬件 PWM + 4 TIM5 软件脉冲）
- 启动默认注册轴数 `8`（`AXIS_APP_AUTO_REGISTER=1`），连接后按 `REG201` 动态显示
- 软件轴和硬件 PWM 轴配置范围均为 `100 Hz..100 kHz`；软件轴高频并发需按实际负载降额
- 点位名称支持 UTF-16，并与 Flash 持久化联通

---

## 1. 文件结构

```
HostTool/
├── teach_axis.py          # 入口 + 主窗口 (连接 / 轴导航 / 示教点 / I/O)
├── comm_worker.py         # QThread 通信线程 (轮询 + 命令队列 + 心跳)
├── modbus_master.py       # Modbus RTU 主站 + 2416_F407 协议封装
├── modbus_tcp_client.py   # 可选 TCP 客户端
├── test_modbus_master.py  # 协议回归测试
├── test_modbus_tcp_client.py
├── requirements.txt
└── README.md
```

---

## 2. 安装与运行

```powershell
# 1) Python 3.9+
python --version

# 2) 安装依赖
cd HostTool
pip install -r requirements.txt

# 3) 启动
python teach_axis.py

# 4) 回归测试
python -m unittest discover -s . -p "test_*.py" -v
```

---

## 3. 关键功能

| 模块 | 功能 |
| --- | --- |
| 顶部连接栏 | 端口扫描 / 波特率 / 从站地址 / 一键连接断开 / 大号急停 |
| 轴导航 + 详情 (×8) | 顶部选择卡切换当前轴；实时位置、状态徽章、按住 JOG (±)、步进 JOG、绝对/相对定位、回原、当前置零、设位置 |
| 示教点面板 | 记录当前已注册轴位置 → 命名 → 一键回放 → 删除 / 清空 / 保存 JSON / 加载 JSON / 导出 CSV；0 轴时记录/移动禁用 |
| 定位点面板 | 每轴 8 个点位的名称 / 绝对坐标 / 运行速度（Hz）读写，先写 RAM 再“保存到 Flash”掉电保持；`Axis_MoveToPoint()` 直接使用该点保存的速度 |
| I/O 监控 | 24 路 X 输入实时点亮、16 路 Y 输出在调试模式双击切换；轴复用输出仅允许在对应轴空闲时测试 |
| 轴配置 | 只读显示固件模板导出的轴绑定、传感器绑定和本地轴名称；可读写每轴可选软限位（默认关闭、掉电保持） |
| 系统 / 诊断 | 回原参数写入、通信看门狗超时设置、固件帧/错误统计 |

### 通信线程亮点

- 命令优先于轮询，**按住 JOG 立即下发**，松开立即停止
- 心跳默认 `800 ms` 自动写 `REG_HEARTBEAT(214)`，并按设备 `COMM_WDT_MS` 自动收紧
- 连续通信失败会触发 UI 断线归位
- 0 轴状态不会发起轴状态轮询，避免空访问

---

## 4. 寄存器映射

```
Per-Axis (10 regs):  Axis0@320 ... Axis7@390
   +0  POS_LO   R    +5  SPEED       RW
   +1  POS_HI   R    +6  TARGET_LO   RW
   +2  STATE    R    +7  TARGET_HI   RW
   +3  HOMED    R    +8  CMD         W
   +4  保留     R    +9  SPEED_HI    RW

Axis Cfg Ext (9 regs): Cfg0@1400 ... Cfg7@1463 只读
   pulse_mode / pwm_ch / soft_ch / pulse_y / dir_y /
   home / lim+ / lim- / flags

System @200..220:
   200 VERSION      0x2416
   201 AXIS_COUNT   当前注册轴数 (默认 8)
   202..205         回原参数
   206 E_STOP
   207 SLAVE_ADDR
   208 SUBDIV
   209 JOG_STEP
   210..214         通信看门狗 / 帧统计 / 心跳
   215 OP_MODE
   216 AXIS_CAPACITY  8
   217 SAVE_POINTS    写 0xA55A 保存（10s 专用超时，不自动重试）
   220 RS232_PROTO    RW  0=Modbus / 1=自由口（掉电保持，建议经 RS485 写入）

Points @400..527:   每轴 8 点坐标 (lo/hi int32)
Names  @600..1367:  每轴 8 点名称 (UTF-16)
SoftLim @1500..1563: 每轴 8 寄存器, 仅调试模式可写
   +0 使能     +1,+2 负向 lo,hi
   +3,+4 正向 lo,hi   +5 有效(读)
   +7 写 0xA55A 立即保存到 Flash
PointSpd @1600..1727: 每轴 8 点速度 (lo/hi uint32 Hz, 100..100000)
I/O    @300..302:   DI_LO / DI_HI / DO_STATE
线圈   0..15:       Y0..Y17
```

---

## 5. 操作流程

1. 烧录固件并确认 `LED_RUN` 周期闪烁
2. 接 USB 转串口（RS-485 或 RS-232 均可；若 `220`=1 使 232 跑自由口，则必须接 RS-485）→ 上位机选端口 → 连接（默认 115200 / 1）
3. 顶部状态变绿，版本显示 `0x2416`，轴容量 `8`
4. 当前默认 `AXIS_COUNT=8`；若关闭自动注册，0 轴时仍可浏览模板和点位，但运动类按钮保持禁用
5. 确认实际接线和安全条件后，再做 JOG / 定位 / 回原 / 示教
6. 点位名称、坐标和速度可先写 RAM，再点“保存到 Flash”
7. 软限位需先切到调试模式且轴空闲，再点轴配置页的“写入并保存”

---

## 6. 常见问题

| 现象 | 可能原因 |
| --- | --- |
| 版本不是 `0x2416` | 固件未更新或串口参数不对 |
| 连接后轴数为 0 | 固件关闭了自动注册，或轴模板创建失败 |
| 软件轴高速抖动 | TIM5 软件脉冲中断负载过高，应降低单轴频率或并发轴数 |
| 保存 Flash 失败 | 有轴 Busy，或未先同步当前页面 RAM |
| 回原失败 | 模板中 `sensor_home` 仍为 `NULL` |

---

## 7. 二次开发指引

- 新增/启用轴：修改 `Firmware/Core/Src/app/axis/axis_manager.c` 的注册逻辑；上位机 `NUM_AXES=8` 已支持
- 更换波特率：顶部下拉 + 固件 `app_config.h::UART_BAUDRATE`
- 打包发布：
  ```powershell
  pip install pyinstaller
  pyinstaller -F -w -n TeachAxis TeachAxis.spec
  ```

版本：v2.2 (2026-10) — 默认 8 轴 / 剩余步数恢复 / 真实命令回执 / 点位名称 UTF-16 /
每点运行速度（掉电保持，`Axis_MoveToPoint` 直接取用）/ 可选软限位
