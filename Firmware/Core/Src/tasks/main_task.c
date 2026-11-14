/**
 * @file    main_task.c
 * @brief   调度器任务适配层。
 * @details 原工程将 Modbus 调试协议、轴控制、I/O 映射和任务函数全部写在
 *          main_task.c 中。当前版本已将实际功能迁移到 app/ 分层目录:
 *          - app/business: 正常业务逻辑；
 *          - app/debug: 上位机调试与示教协议；
 *          - tasks: 只保留调度器可调用的薄适配函数。
 */
#include "tasks/main_task.h"
#include "app/app.h"

void Task_Main(void)
{
    App_Poll();
}
