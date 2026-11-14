# 2416 Studio · 新版桌面上位机

参照桌面 Pyside+Web+PLCProgram 的 PySide6 + Vue + QWebChannel 架构与浅色界面，为当前 STM32F407 / 2416 固件重新适配。程序运行时不启动 HTTP 服务，不打开外部浏览器。

## 启动

双击当前工程根目录的“启动上位机.bat”，或 HostStudio/启动上位机.bat。脚本优先启动 Releases/20261111/Studio2416/Studio2416.exe。打包目录中的 _internal 必须与 EXE 一起保留。更新前已打开的旧版窗口需先退出。

默认离线。点击右上角连接设置：

- 串口 Modbus RTU：选择 COM，默认 115200、8N1、从站 1。支持 RS485；RS232 需要固件处于 Modbus 模式。连接时校验固件版本 0x2416，读取实际轴数与配置。
- 模拟设备：提供八轴操作；E–H 轴的原点绑定仅用于演示，模拟数据只在本次连接中保留。模拟保存不代表真实设备 Flash。

## 页面与操作

- 总览：连接、注册轴数、回原数量、八轴位置与运动状态。
- 轴与点位：连续点动、绝对/相对定位、剩余步数恢复、当前位置清零、八个定位点的示教/写入/运行、Flash 保存、本地轴名称。
- 回原与限位：真实传感器绑定、回原方向、模式与快找/回退/慢找频率、回原流程、可选软限位。
- I/O：24 路输入、16 路输出镜像及轴通道映射。调试模式下每个 Y 点都提供“置 ON / 置 OFF”，写入后读取设备线圈状态。轴复用输出只在对应轴空闲时开放；其他空闲轴的输出可独立操作。
- 诊断：设备帧统计、通信看门狗、通信错误和本次会话的操作历史。

手动操作前切换到调试模式。当前项目固件的所有原点指针为 NULL，真实设备会显示“未绑定”并禁用回原；必须在固件模板中配置实际传感器才可回原。上位机不会将模拟绑定写入板卡。固件没有整机启动/暂停协议，因此顶部只提供模式切换、读取设备与全部急停。

“回原与限位 → 全局回原参数”新增回原方向选择：0=负向，1=正向。点击“写入并读回”后统一用于所有轴的上位机回原。该参数需要本次新版固件：保持寄存器 222（0 基地址）读写，默认负向，通过固件通用 NVRAM 自动掉电保持。运动中不允许修改。旧固件返回不支持时显示升级提示，其他回原参数仍可按原协议设置。业务代码自行调用 Axis_HomeWithModeEx 时传入的方向参数保持原有语义。

新版固件已构建在 Firmware/dist/init_2416_f407.hex / .bin / .elf，并同步到工程根目录 dist。实际控制板需要烧录新版才能使用全局方向。数字输出手动控制沿用已有 FC05 协议，原固件已支持，无需为该功能单独升级。

位置单位是 pulse，速度单位是 Hz。无编码器机械位置反馈。点位写入 RAM 后读回，轴空闲时可保存到 Flash。设备参数的掉电保持仍遵循固件实现。名称最多 11 个 UTF-16 单元；本地轴名仅存于 Qt 页面存储。日志位于 %LOCALAPPDATA%/2416Studio/desktop.log。

## 结构

    backend/protocol.py     从当前 HostTool 复制的 2416 RTU 协议封装
    backend/device.py       串口适配器与模拟设备
    backend/controller.py   单一通信线程、请求队列、采样、心跳、输入校验
    desktop/window.py       本地 WebEngine 窗口和 QWebChannel 桥
    desktop/smoke.py        仅模拟的真实桌面集成验证
    desktop_launcher.py     入口、日志、单实例
    web/src/               Vue 界面及样式
    web/dist/              已构建的本地界面资源
    App/Studio2416/         打包的 Windows 程序
    validation/            桌面验证报告与截图

所有真实串口事务由单一后台线程执行。停止请求优先，并使旧队列动作失效。写命令在响应丢失时不自动重发，防止相对定位重复执行。点动用 650ms 主机续期，松开/切页/失焦/最小化/关闭/页面加载时停止点动；普通定位不会因为失焦自动中止。窗口关闭等待通信线程退出。真实设备断线后的停车依赖板卡通信看门狗及现场硬件安全回路。

## 源码开发

    python -m pip install -r requirements.txt
    cd web
    npm ci
    npm run build
    cd ..
    python desktop_launcher.py

修改前端后必须重新 build。构建 Windows 包需安装 PyInstaller：

    python -m pip install pyinstaller
    powershell -NoProfile -ExecutionPolicy Bypass -File tools/BuildDesktop.ps1

验证：

    python -m unittest discover -s tests -v
    python desktop_launcher.py --smoke-report validation/native-smoke.json

验证模式只连接本地模拟设备，完成后退出。截图与 JSON 写入指定报告所在目录。本次验证不等同于真实控制板联调，未执行真实设备连接、烧录或运动。
