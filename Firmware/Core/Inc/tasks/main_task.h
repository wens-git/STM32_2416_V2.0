/**
 * @file    main_task.h
 * @brief   调度器任务适配层接口。
 * @details 该文件不再承载具体调试协议或业务逻辑，只提供调度器可注册的
 *          任务函数。真正的应用实现位于 app/ 目录。
 */
#ifndef __MAIN_TASK_H
#define __MAIN_TASK_H

void Task_Main(void);

#endif /* __MAIN_TASK_H */
