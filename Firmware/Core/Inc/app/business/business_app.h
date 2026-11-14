/**
 * @file    business_app.h
 * @brief   正常业务逻辑入口。
 * @details 本模块只处理设备在正式运行模式下应该执行的业务流程。
 *          调试、示教、上位机寄存器协议等功能不得写入本模块，
 *          以保证量产运行逻辑清晰、稳定、可审计。
 */
#ifndef __BUSINESS_APP_H__
#define __BUSINESS_APP_H__

void Business_App_Init(void);
void Business_App_Tick1ms(void);
void Business_App_Poll(void);

#endif /* __BUSINESS_APP_H__ */
