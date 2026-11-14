/**
 * @file    app.c
 * @brief   应用层总控实现。
 * @details 该文件负责把正常业务逻辑与调试服务组合成完整应用。
 *          设计原则:
 *          1) main.c 不直接依赖任何具体业务模块；
 *          2) debug 与 business 分层并行存在，互不包含对方私有实现；
 *          3) 调度器只调 App_Poll(), 1ms 钩子只调 App_Tick1ms()。
 */
#include "app/app.h"
#include "app_config.h"
#include "app/business/business_app.h"
#include "app/axis/axis_manager.h"
#include "motor_axis.h"
#include "nvram.h"
#if APP_DEBUG_SERVICE_ENABLE
#include "app/debug/motor_debug_service.h"
#endif
#if RS485_ENABLE
#include "app/rs485_service.h"
#endif
/* USART1 协议归属查询：
 * - 启用调试服务时由保持寄存器 220 对应的运行期选择器决定（Debug_Service_Rs232IsFreePort）；
 * - 裁掉调试服务时退回编译期常量 RS232_PROTOCOL_MODE。 */
#if !RS232_RUNTIME_PROTOCOL_SWITCH
/* 关闭运行期切换: 严格按编译期常量路由，忽略 NVRAM 中可能残留的旧选择。 */
#define APP_RS232_IS_FREEPORT()  ((RS232_PROTOCOL_MODE) == RS232_PROTO_FREEPORT)
#elif APP_DEBUG_SERVICE_ENABLE
#define APP_RS232_IS_FREEPORT()  Debug_Service_Rs232IsFreePort()
#else
#define APP_RS232_IS_FREEPORT()  ((RS232_PROTOCOL_MODE) == RS232_PROTO_FREEPORT)
#endif

/* 运行期切换时两条 USART1 协议都要链接进来，编译期自由口构建同样需要。 */
#if (RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT) || RS232_RUNTIME_PROTOCOL_SWITCH
#include "app/rs232_service.h"
#endif

static bool app_nvram_save_guard(void)
{
    for (uint8_t i = 0; i < AXIS_APP_CAPACITY; i++) {
        if (AxisApp_IsRegistered(i) && Axis_IsBusy(AxisApp_GetAxisId(i))) {
            return false;
        }
    }
    return true;
}

void App_Init(void)
{
    Business_App_Init();
    NVRAM_Init();
    NVRAM_Auto_Init();
    /* 掉电保持的软限位参数必须在 NVRAM_Auto_Init() 载入后下发到轴运行时。 */
    AxisApp_ApplySoftLimits();
    NVRAM_Auto_SetSaveGuard(app_nvram_save_guard);
#if APP_DEBUG_SERVICE_ENABLE
    Debug_Service_Init();
#endif
#if RS485_ENABLE
    RS485_Service_Init();
#endif
}

void App_Tick1ms(void)
{
    NVRAM_Auto_Tick1ms();
#if APP_DEBUG_SERVICE_ENABLE
    Debug_Service_Tick1ms();
    if (Debug_Service_IsDebugMode()) return;
#endif
    Business_App_Tick1ms();
}

void App_Poll(void)
{
    /* USART1 的协议归属在运行期由保持寄存器 220 决定（RS232_RUNTIME_PROTOCOL_SWITCH）：
     *   220 = 0 -> USART1 运行 Modbus RTU 从站，供调试上位机使用；
     *   220 = 1 -> USART1 运行自由口协议，帧交给 RS232_FreePort_OnFrame()。
     * 两条路径同时驻留，切换不需要重新编译或重新烧录。
     * USART3/RS485 始终独立运行 Modbus，因此 220 可以随时通过 485 改写，
     * 即使 232 当前已经从 Modbus 切走也不会失联。 */
#if APP_DEBUG_SERVICE_ENABLE
    /* 先处理模式切换；调试模式下业务层不得重新占用运动资源。 */
    if (!APP_RS232_IS_FREEPORT()) {
        Debug_Service_Poll();
    }
#endif
#if (RS232_PROTOCOL_MODE == RS232_PROTO_FREEPORT) || RS232_RUNTIME_PROTOCOL_SWITCH
    /* 自由口路径保留在 APP_DEBUG_SERVICE_ENABLE 之外：即使调试服务被裁掉，
       编译期自由口构建仍然可用。运行期切换模式下该分支由 220 控制。 */
    if (APP_RS232_IS_FREEPORT()) {
        RS232_Service_Poll();
    }
#endif
#if RS485_ENABLE
    /* Independent USART3/RS485 poll; stays active in debug mode. */
    RS485_Service_Poll();
#endif
    NVRAM_Auto_Poll();
#if APP_DEBUG_SERVICE_ENABLE
    if (Debug_Service_IsDebugMode()) return;
#endif
    Business_App_Poll();
}
