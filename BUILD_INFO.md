# 2416_F407 V2.0 构建与验证信息

## 版本基线

- 固件协议版本：`0x2416`
- 上位机版本：`1.0`
- 轴容量：8 轴（4 轴硬件 PWM + 4 轴 TIM5 软件脉冲）
- 上位机窗口标题：`2416_F407 在线示教调控轴 v1.0`
- 归档顶层目录：`2416_V2.0/`

## 固件构建

- 构建时间：`2026-10-24 15:59:20 +08:00`
- 目标：STM32F407xx（Debug）
- 编译器：GNU Tools for STM32 13.3.1（arm-none-eabi-gcc 13.3.1）
- CMake：3.28.1
- 编译告警：0

资源占用：

| 项目 | 占用 |
| --- | --- |
| FLASH | 31280 B / 128 KB（23.86%） |
| RAM | 20208 B / 128 KB（15.42%） |

2026-10-28 固件已随回原方案升级重新编译（Release），烧录产物：

| 文件 | 大小 | SHA256 |
| --- | --- | --- |
| `dist/init_2416_f407.bin` | 31280 B | E06B378CD3B1F83771C48226F154A0856CA68F47F7065033E7AA71126C0046A3 |
| `dist/init_2416_f407.elf` | 75564 B | CC78AF90C43FB1F5398AD726601226EE3172B7622EA67574CAD9298852D0E94D |
| `dist/init_2416_f407.hex` | 88091 B | DA5F075AB480ED0850E1507D594F050DC7BFD29D5D13E626B7BC3885D66B9810 |

烧录建议：优先使用 `init_2416_f407.hex`（ST-LINK Utility / STM32CubeProgrammer）；`init_2416_f407.bin` 为裸二进制，加载地址 `0x08000000`；`init_2416_f407.elf` 含调试符号。

## 上位机构建

- Python：3.14.5
- PySide6：6.11.1
- pyserial：3.5
- PyInstaller：6.22.3
- 构建时间：`2026-10-28 15:08:00 +08:00`
- 打包命令：`python -m PyInstaller --noconfirm --clean --distpath ..\dist TeachAxis.spec`
- 产物：`dist/TeachAxis.exe`（单文件、无控制台）
- exe 大小：49029563 B
- exe SHA256：`63FEF7D3FBFA8208ABDEDdab7888D48B624CB8E059E03E3508BEC3A39DE6BCC1`

### 本次升级内容（2026-10-28 回原方案升级）

- 固件：两种回原模式（PRECISION / TRIGGER）的回退阶段均由"固定步数回退"改为"反向退到传感器释放沿"，免标定，自适应快找停止过冲。
- 固件：新增独立回退速度参数 `home_backoff_freq`（NVRAM 掉电保持，默认 300 Hz），Modbus 寄存器 `221`（读写，216/214 已被占用故顺延）。
- 固件：废弃寄存器 `204`（回退步数），仅保留写入兼容；废弃 API 参数 `backoff_steps`。
- 上位机：回原参数区"回退步数"改为"回退 Hz"（范围 100..100000），应用时写寄存器 221；读取配置时同步读 221（旧固件读失败时保持默认 300）。
- 上位机：回原模式选项文案更新为"精回原 (退到释放+慢找)"/"退到释放即回原"。

本次打包所用上位机源码：

| 文件 | SHA256 |
| --- | --- |
| `HostTool/teach_axis.py` | 4032833680CF210FBAF93F0952AE4757CA193F99CBC4BBC20D27CA0C026A5E7C |
| `HostTool/comm_worker.py` | BADCEB6C41C888894076438E6CF14EF4F7FE5D0315F5563377C85B0D258568C4 |
| `HostTool/modbus_master.py` | 3148FE5E3768EAC44A9F057D01DCE74608176DAC021E5AECCE48AADB3FE95CA4 |

## 验证结果

- 语法检查：`teach_axis.py`、`comm_worker.py`、`modbus_master.py` 通过 `py_compile`
- 通信回归测试：**39/39 通过**（`Ran 39 tests ... OK`）
- 离屏界面检查：标题 `2416_F407 在线示教调控轴 v1.0`，8 个轴导航卡，8 个轴控制卡，3 个标签页，16 个 Y 按钮全部可点击
- 打包程序启动冒烟：exe 启动 8 秒后进程存活且 `Responding=True`；测试结束后已关闭其全部父子进程
- **实体串口 / RS485 联调未在本次归档步骤中进行**

## 本次上位机交互修正

- 启动与断开后固定显示已配置的全部 8 个轴位
- 轴导航按固件实际注册情况显示与启用
- Y 按钮始终可点击，是否允许写入由运行模式与固件反馈判定
- 模式切换日志仅在状态变化时记录一次
- 未改动固件、Modbus 寄存器映射、轴注册与 Y 点映射

## 串口通道

- RS232：USART1（PA9/PA10），115200-8N1，Modbus RTU 或自由口（寄存器 `220`，掉电保持）
- RS485：USART3（PC10/PC11，AF7），收/发方向 PB14（1=发送，0=接收），115200-8N1，Modbus RTU
- 两端口共用同一套寄存器映射；从站地址 `207` 与帧统计 `212`/`213` 按端口独立

## 归档完整性

归档根目录 `SHA256SUMS.txt` 覆盖除自身以外的全部交付文件，路径使用正斜杠分隔。本次归档已移除 `build/`、`__pycache__`、`*.pyc` 与上位机运行时配置 `last_config.json`。
