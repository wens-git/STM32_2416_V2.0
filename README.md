# STM32F407 2416 控制器工程

本目录是可独立归档和交付的完整工程，包含 STM32F407 单片机固件、在线示教上位机、协议回归测试及必要的构建配置。归档版本已经删除编译缓存、重复发布文件和未参与本项目编译的 CMSIS 扩展包，但保留了固件与上位机的全部有效源码。

## 一、项目能力

- 固件版本握手值：`0x2416`
- 最大轴容量：8 轴，其中 4 轴使用硬件 PWM，4 轴使用 TIM5 软件脉冲
- 默认注册轴数：8（`AXIS_APP_AUTO_REGISTER=1`），上位机根据寄存器 `REG201` 动态显示
- 通信方式：双串口通道，均为 `115200-8N1`
  - **RS232**（USART1，PA9/PA10）：Modbus RTU **或** 自由口，由保持寄存器 `220` 运行期切换（`0`=Modbus，`1`=自由口，掉电保持）
  - **RS485**（USART3，PC10/PC11，方向脚 PB14）：Modbus RTU
  - 两端口共用同一套寄存器映射；`207` 从站地址、`212`/`213` 帧统计按端口独立
  - 上位机另保留可选 Modbus TCP 客户端
- 点位数据：支持坐标、UTF-16 名称、每点运行速度及 Flash 持久化（均由上位机写入，掉电保持）
- 安全功能：急停、可选软限位（默认关闭、掉电保持）、回原、通信看门狗和轴禁用控制；硬限位仅服务回原，日常误触不报警不锁轴

## 二、目录说明

| 路径 | 内容 | 归档要求 |
| --- | --- | --- |
| `Firmware/Core` | 单片机入口、中断、板级驱动、轴控制和业务逻辑 | 必须保留 |
| `Firmware/Drivers` | 本工程编译所需的 STM32F4 HAL 与 CMSIS 头文件 | 必须保留 |
| `Firmware/cmake` | ARM GCC 工具链和 CubeMX 源文件清单 | 必须保留 |
| `Firmware/*.ld`、`*.s` | 链接脚本和芯片启动文件 | 必须保留 |
| `HostTool` | PySide6 上位机源码、通信模块、测试和打包配置 | 必须保留 |
| `dist/TeachAxis.exe` | 已打包的 Windows 上位机，可直接运行 | 发布文件 |
| `dist/init_2416_f407.hex` / `.bin` / `.elf` | 已编译的可烧录固件镜像 | 发布文件 |
| `CMakeLists.txt`、`CMakePresets.json` | 固件顶层构建入口 | 必须保留 |
| `BRINGUP_CHECKLIST.md` | 板卡上电与现场联调检查表 | 建议保留 |
| `轴控制函数与业务使用指南.md` | 轴 API、暂停/插入/剩余步数恢复、Modbus 与安全边界 | 建议保留 |
| `ui_f407_1440x900.png`、`ui_f407_1180x760.png` | 上位机界面截图（当前版本） | 建议保留 |

`build/`、`HostTool/build/`、`HostTool/dist/`、`__pycache__/` 均为可再生内容，不属于源码归档。根目录只保留一份正式发布的 `dist/TeachAxis.exe`，避免同一程序重复占用空间。

## 三、环境要求

### 固件

- CMake 3.22 或更高版本
- Ninja
- GNU Arm Embedded Toolchain（需要 `arm-none-eabi-gcc`、`objcopy` 和 `size`）
- 目标芯片：STM32F407xx

### 上位机

- Windows 10/11
- Python 3.9 或更高版本
- 依赖版本见 `HostTool/requirements.txt`

## 四、固件构建

在本 README 所在目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

链接成功后，固件镜像统一输出到源码根目录的 `dist/`：

- `init_2416_f407.elf`：包含调试符号的链接结果
- `init_2416_f407.hex`：适合多数烧录工具的 Intel HEX 镜像
- `init_2416_f407.bin`：纯二进制烧录镜像
- `init_2416_f407.map`：链接映射文件，位于 `build/Debug/Firmware/`，用于分析 Flash/RAM 占用

构建目录与 `dist/` 中的镜像均为可再生成内容。删除 `build/` 后按上述命令可完整重建；`dist/` 会在每次链接后自动刷新。

## 五、上位机运行与测试

```powershell
python -m pip install -r HostTool/requirements.txt
python HostTool/teach_axis.py
python -m unittest discover -s HostTool -p "test_*.py" -v
```

不安装 Python 环境时，可直接运行 `dist/TeachAxis.exe`。详细操作、寄存器映射和常见问题见 `HostTool/README.md`。

## 六、重新打包上位机

```powershell
python -m pip install pyinstaller
cd HostTool
pyinstaller --clean TeachAxis.spec
```

打包结果默认生成在 `HostTool/dist/TeachAxis.exe`。确认测试通过后，可用它替换根目录 `dist/TeachAxis.exe`，随后删除 `HostTool/build/` 和 `HostTool/dist/`。

## 七、归档完整性说明

本归档保留了程序运行和离线重建需要的内容。已删除的 CMSIS 网页文档、DSP、NN、RTOS、预编译库以及其他 STM32 系列设备包均未被当前 CMake 工程引用；删除它们不会改变 STM32F407 固件代码或生成结果。如未来需要引入 CMSIS-DSP、CMSIS-RTOS 或迁移到其他芯片，应从对应版本的 ST/CMSIS 官方软件包重新补充依赖。

建议归档前最后执行一次固件全新构建和上位机回归测试，并记录编译器版本、固件校验值及现场验证日期。


## 八、串口调试切换说明

- 上电默认：`215`（运行模式）为 **正常模式**；`220`（232 协议）读取掉电保持值，出厂默认 `0` = Modbus。
- 按一次 `X0`（PA0，低有效，10 ms 消抖）在 **调试模式 / 正常模式** 之间切换；`215` 不持久化，重启回到正常模式。
- 接调试上位机时可任选其一：**RS485**（固定 Modbus，推荐）或 **RS232**（要求 `220`=0）。
- 想让 RS232 跑自由口对接第三方上位机/PLC，同时用 RS485 接调试上位机：经 RS485 写 `220`=1 即可。切换时会重新武装通信看门狗，不会误触发急停。
- 两端口可同时工作、互不阻塞；寄存器是同一份，仅从站地址与帧统计按端口分开。

## 九、归档文件与校验

| 文件 | 说明 |
| --- | --- |
| `README.md` | 工程总览、目录说明与构建步骤 |
| `BUILD_INFO.md` | 固件与上位机的构建时间、产物大小、SHA256 及验证结果 |
| `SHA256SUMS.txt` | 归档完整性清单，覆盖除自身以外的全部文件 |

校验方法（在本目录执行）：

```powershell
Get-Content SHA256SUMS.txt | ForEach-Object {
    $hash, $path = $_ -split '  ', 2
    $actual = (Get-FileHash -LiteralPath $path.Replace('/', '\') -Algorithm SHA256).Hash
    if ($actual -ne $hash) { Write-Output "MISMATCH: $path" }
}
```

归档不包含 `build/`、`HostTool/build/`、`__pycache__/`、`*.pyc` 以及上位机运行时生成的 `last_config.json`。
