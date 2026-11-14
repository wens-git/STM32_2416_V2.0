# STM32F407_2416_INIT 架构说明

## 工程定位

本工程是 `2416 / STM32F407` 八轴通用调试固件，延续了 `1612 / F103` 已验证工程的分层方式，但资源规模扩大为:

- 最多 8 轴步进控制：4 路硬件 PWM + 4 路 TIM5 软件脉冲
- 启动默认注册全部 `8` 轴（`AXIS_APP_AUTO_REGISTER=1`）
- 24 路输入 `X0~X27`
- 16 路输出 `Y0~Y17`
- HostTool 通过 `Modbus RTU 115200 8N1` 联机
- 固件版本握手值固定为 `0x2416`

当前代码状态下，调试链路、点位名称链路、FC16 事务保护、通信看门狗和 Flash 持久化保护都已经接入；`axis_manager.c` 中真实传感器绑定仍保留为待现场落地项。

## 目录职责

- `Core/Src/main.c`: MCU 启动入口，只负责 HAL 初始化、时钟、IO、板级外设、调度器和 IWDG。
- `Core/Src/app/app.c`: 应用层总控，只做 `business` 与 `debug` 两层组合。
- `Core/Src/app/axis/axis_manager.c`: 轴模板与注册状态唯一入口，维护 8 套固定引脚模板、注册计数、原点/限位/定位触发以及回原方向。
- `Core/Src/app/business/business_app.c`: 正常业务层入口；当前主要负责 `AxisApp_Init()`、周期调用 `Axis_Tick()` 所需的上层壳逻辑。
- `Core/Src/app/debug/motor_debug_service.c`: Modbus 调试协议核心，负责寄存器映射、模式切换、JOG/定位/回原、点位与名称访问、通信看门狗、异常码语义和 FC16 事务提交。
- `Core/Src/tasks/main_task.c`: 调度器任务适配层，当前只保留 `App_Poll()` 转发。
- `Core/Src/Dependence`: BSP、NVRAM、UART、Modbus、轴控制、硬件 PWM、TIM5 软件脉冲等底层依赖。
- `Core/Inc/app`: 应用层头文件，按 `axis`、`business`、`debug` 拆分。
- `HostTool`: PySide6 上位机，包含串口联机、实时状态、I/O、点位名称与坐标管理。

## 启动与调度链路

当前启动顺序以 `main.c` 实际代码为准:

1. `HAL_Init()`
2. `SystemClock_Config()`
3. `BSP_IWDG_Init(SYSTEM_IWDG_TIMEOUT_MS)`，当前为 `8000ms`
4. `IO_Init()`
5. `System_Init()`
6. `while(1) { scheduler_run(); __WFI(); }`

其中 `System_Init()` 的内部顺序固定为:

1. `BSP_UART_Init()`
2. `PWM_Motor_Init()`
3. `SoftPWM_Init()`
4. `App_Init()`
5. `Scheduler_Setup()`

调度侧约束如下:

- `scheduler_add(Task_Main, 1)`，主任务每 1ms 调一次。
- `Task_Main()` 只调用 `App_Poll()`。
- `Tick_Hook()` 的顺序是 `IO_Scan() -> Axis_Tick() -> App_Tick1ms()`。
- 这个顺序不能随意改动，因为轴状态机必须消费“本周期最新输入状态”。

## 分层原则

- 启动层不写业务: `main.c` 不直接处理寄存器、轴命令或上位机协议。
- 轴绑定只能由业务侧定义: `axis_manager.c` 是硬件绑定唯一真源，调试服务只读导出，不允许上位机改轴绑定。
- 正常业务与调试协议并行: `business_app.c` 和 `motor_debug_service.c` 并列存在，由 `app.c` 统一编排。
- 调试服务可编译开关控制: `APP_DEBUG_SERVICE_ENABLE` 默认开启，控制 `Debug_Service_*` 是否接入 `App_Init/App_Tick1ms/App_Poll`。
- 调试层不直接创建轴: 轴实例始终通过 `AxisApp_Init()` / 业务注册接口创建，再由调试协议按 `Axis_ID[]` 访问。
- 业务保持寄存器与 1612 一致: 原始地址 `0~99` 直接映射到 `Modbus_HoldingRegisters[0~99]`，业务代码可在 `business_app.c` 中直接读写。

## 轴模板与注册现状

`axis_manager.c` 当前提供 8 套固定模板，默认 `AXIS_APP_AUTO_REGISTER = 1`，因此上电后 `REG201 AXIS_COUNT = 8`：

- 轴0: 硬件 PWM `PWM_CH_Y1` 脉冲, `Y0` 方向
- 轴1: 硬件 PWM `PWM_CH_Y3` 脉冲, `Y2` 方向
- 轴2: 硬件 PWM `PWM_CH_Y4` 脉冲, `Y5` 方向
- 轴3: 硬件 PWM `PWM_CH_Y16` 脉冲, `Y17` 方向
- 轴4: 软件脉冲 `SOFT_PWM_CH_Y6` 脉冲, `Y7` 方向
- 轴5: 软件脉冲 `SOFT_PWM_CH_Y10` 脉冲, `Y11` 方向
- 轴6: 软件脉冲 `SOFT_PWM_CH_Y12` 脉冲, `Y13` 方向
- 轴7: 软件脉冲 `SOFT_PWM_CH_Y14` 脉冲, `Y15` 方向

当前传感器绑定现状:

- 原点、正限位、负限位、定位触发输入均暂为 `NULL`（轴报警体系已删除，模板中不再有报警输入项）
- 因此“回原命令是否可执行”取决于后续是否在模板中补齐 `sensor_home`
- 上位机读取到的轴配置寄存器是只读导出结果，不代表现场已经完成接线
- 软件轴和硬件 PWM 轴的配置上限均为 `100 kHz`；软件脉冲由 TIM5 中断调度，高频多轴并发必须按现场 CPU 负载和抖动实测降额

## 调试模式说明

调试服务支持两种模式切换入口:

- 写保持寄存器 `215 OP_MODE`
- 本地输入 `X0` 的按下沿切换，逻辑位于 `IODEF.c`

运行模式含义如下:

- `0`: 正常运行模式
- `1`: 调试模式
- 上电默认进入 `0` 正常运行模式，需要调试运动时由上位机或本地输入显式切换。

正常运行模式下的限制以当前代码为准:

- JOG、绝对定位、相对定位、回原、点位写入、点位名称写入、FC05 输出写入被锁定
- 系统寄存器仅允许 `206(E_STOP)`、`214(HEARTBEAT)`、`215(OP_MODE)` 写入
- 轴寄存器仅允许写 `TARGET_LO/TARGET_HI/CMD`，但 `CMD` 里只有 `STOP / SET_POS` 会被接受（轴报警已删除，不再有清警命令）

调试模式下开放:

- JOG、绝对定位、相对定位、回原
- 点位坐标与点位名称写入
- 保存点位到 Flash
- 输出线圈写入；轴复用的脉冲/方向输出仅在对应轴空闲时允许手动测试

切换运行模式时会中止全部轴运动，防止上一模式启动的 JOG 或定位动作跨模式继续运行。
进入调试模式后，`Business_App_Tick1ms()` 和 `Business_App_Poll()` 暂停执行，避免正常业务状态机重新启动刚刚停止的轴；调试服务和通信看门狗保持运行。

## Modbus 协议与安全约束

当前协议要点如下:

- 通用业务保持寄存器: `0~99`，对应 `Modbus_HoldingRegisters[]`，支持 FC03/FC06/FC16
- 从站握手版本: `REG200 = 0x2416`
- 当前注册轴数: `REG201`，默认 `8`，最大 `8`
- 轴容量: `REG216 = 8`
- 轴状态区: `320~399`，每轴 10 寄存器
- 扩展轴配置区: `1400~1471`，每轴 9 寄存器
- 点位区: `400~527`，每轴 16 寄存器
- 点位名称区: `600~1367`，每轴 `8 点 * 12 UTF-16 码元`
- 保存点位: 写 `REG217 = 0xA55A`

异常码语义已收紧为:

- 非法地址返回 `0x02`
- 非法值、模式受限、命令条件不满足返回 `0x03`
- 设备内部故障返回 `0x04`

当前读写链路还具备以下保护:

- FC16 多寄存器写入采用影子事务，全部成功后才统一提交
- FC16 事务中禁止包含轴命令寄存器；先批量写参数，提交成功后再用 FC06 单独写命令
- 写 `REG207 SLAVE_ADDR` 时先按旧站号发送成功响应，再切换到新站号
- 连续读块中的保留位已显式补齐，避免上位机整块读取因“寄存器空洞”失败
- `REG204 home_backoff` 禁止写 `0`
- `REG205 home_mode`: `0=精回原`, `1=触发即回原`
- `CMD_SAVE_POINTS` 在任一轴 Busy 时拒绝执行
- 轴命令按底层 API 的真实结果应答；状态、安全输入或脉冲资源拒绝动作时返回异常，而不是成功帧
- 点位保存到 Flash 前，上位机当前页面数据必须先同步到 MCU RAM

## 通信看门狗

通信看门狗相关寄存器如下:

- `210`: `COMM_WDT_MS`
- `211`: `COMM_WDT_ST`
- `212`: `COMM_FRAME`
- `213`: `COMM_ERR`
- `214`: `HEARTBEAT`

当前行为:

- 默认超时 `2000ms`
- 有效 Modbus 帧或心跳写入都会喂狗
- 超时后固件自动停止所有轴
- 上位机连接成功后会把设备当前 `COMM_WDT_MS` 回填到 UI，并自动同步后台心跳周期，确保心跳严格小于设备超时值

## NVRAM / 点位持久化

当前点位链路为“RAM 镜像 + 显式保存”模型:

- 写点位坐标或名称，只先改 RAM 镜像
- 调用 `CMD_SAVE_POINTS` 后才真正写入 Flash
- 写 Flash 前会检查所有轴是否空闲
- Flash 写后依赖 `NVRAM_Save()` 内部回读校验决定成功/失败
- 擦除和编程期间会分阶段刷新 IWDG；上位机仅对保存命令使用 `10s` 超时且不自动重试

这意味着:

- “写入到设备”只更新当前运行态 RAM
- “保存到 Flash”才是掉电保持动作
- 若现场联调时需要验证掉电保持，必须执行完整保存流程并重新上电复查

## 上位机对接口径

当前 HostTool 代码口径如下:

- 串口参数: `115200 8N1`
- 固件版本校验: `0x2416`
- 轴数支持: `0..8`，连接后按 `REG201` 动态显示
- 运行模式显示: 跟随 `REG215`
- 点位页保存 Flash: 先下发当前页面 RAM，再触发保存
- 轴配置页: 只读模板导出，不允许从上位机改硬件绑定
- 0 轴时示教“记录当前位置 / 移动到点”禁用，但 8 套定位点模板仍可浏览编辑

## 后续业务开发位置

- 正常量产业务流程: 写入 `Core/Src/app/business/business_app.c`
- 普通上位机业务数据: 直接使用 `Modbus_HoldingRegisters[0~99]`；单个 `uint16_t` 可直接访问，多字数据需约定字序并使用命令寄存器提交
- 轴绑定、传感器绑定、回原方向、硬件接线落地: 写入 `Core/Src/app/axis/axis_manager.c`
- 调试协议、寄存器布局、调试限制策略: 写入 `Core/Src/app/debug/motor_debug_service.c`
- NVRAM 数据模型或持久化格式扩展: 写入 `Core/Src/Dependence/nvram.c`
- HostTool 联机逻辑或界面行为调整: 写入 `HostTool`
- 新增周期任务: 优先放入 `App_Poll()` 内分发；确实需要独立周期时再修改 `Scheduler_Setup()`

## Flash 分区

- 应用程序使用 `0x08000000~0x0801FFFF`，共 128KB
- Generic persistent store 使用 Sector 5/6 (`0x08020000~0x0805FFFF`)，共 256KB
- Sector 7 使用 `0x08060000~0x0807FFFF`，共 128KB，专用于旧版点位 NVRAM
- 链接脚本不允许代码或只读数据进入 Sector 5/6/7，避免 NVRAM 擦除操作破坏固件
## Generic persistent variables

The keyword-style `NVRAM_PERSISTENT(type, name, initializer)` macro registers a
scalar variable for automatic persistence. Existing arrays and structures can
be declared normally and registered with `NVRAM_PERSISTENT_REGISTER(name)`.
Descriptors are retained in the firmware image and values are stored in an
alternating two-bank record store in Sectors 5 and 6
(`0x08020000~0x0805FFFF`). Each record contains a CRC and a commit word
written last. A save writes and validates the new bank before erasing the old
bank, so an interrupted write or erase leaves at least one committed snapshot
available at boot.

The generic store is polled from `App_Poll()` and saves after
`NVRAM_AUTO_SAVE_DELAY_MS` (default 5000 ms) without further explicit calls.
The save guard defers Flash erase/program operations while any registered axis
is busy. This mechanism does not use the host-tool point database or Modbus
register map; existing point persistence remains in Sector 7 and continues to
use the explicit `NVRAM_Save()` command path.
