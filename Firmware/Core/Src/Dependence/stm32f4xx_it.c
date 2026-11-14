/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   中断服务函数 (ISR) 实现文件
  * @details 本文件包含所有中断向量的服务程序:
  *
  *          【Cortex-M4 内核异常】
  *          - NMI_Handler:        不可屏蔽中断 (极少触发)
  *          - HardFault_Handler:  硬件故障 (内存越界/非法指令 → LED闪烁+复位)
  *          - MemManage_Handler:  内存管理错误 (MPU 违规)
  *          - BusFault_Handler:   总线错误 (无效地址访问)
  *          - UsageFault_Handler: 用法错误 (未定义指令/除零 等)
  *          - SVC/DebugMon/PendSV: OS 相关 (当前未使用)
  *
  *          【系统定时器】
  *          - SysTick_Handler:    1ms 系统节拍 (HAL 时基 + 调度器时基)
  *
  *          【外设中断】
  *          - USART1_IRQHandler:  串口1 IDLE 空闲中断检测 + HAL 标准处理
  *          - DMA2_Stream5_IRQHandler: USART1 RX DMA 传输完成
  *          - DMA2_Stream7_IRQHandler: USART1 TX DMA 传输完成
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "soft_pwm.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scheduler.h"  /* scheduler_hw_tick_isr: 调度器硬件节拍入口 */
#include "bsp_uart.h"   /* BSP_UART_IDLE_Handler: IDLE 空闲帧检测, huart1: 串口句柄 */
#include "bsp_uart485.h" /* USART3 RS485: independent DMA1 and IDLE handling */
#include "motor_axis.h" /* Axis_PulseISR: PWM 脉冲计数中断回调 */
#include "pwm_motor.h"  /* PWM_GetTimHandle: 获取定时器句柄 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
/* 注: huart1 在 bsp_uart.c 中定义，通过 bsp_uart.h 外部声明，
 *      在中断处理中用于 HAL_UART_IRQHandler() 和 DMA IRQHandler() */
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 处理器异常 / 中断处理函数                           */
/******************************************************************************/

/**
  * @brief  NMI (不可屏蔽中断) 处理函数
  * @note   触发场景: CSS (时钟安全系统) 检测到 HSE 失效时自动触发。
  *         本项目使用 HSI，理论上不会触发 NMI，保留死循环以防万一。
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief  HardFault (硬件故障) 处理函数
  * @details 触发场景: 空指针解引用、栈溢出、非法指令等严重错误。
  *          工业级处理策略:
  *          1) __disable_irq()   — 关闭中断，防止级联故障
  *          2) 直接操作 GPIO 寄存器闪烁 LED_ERR (PB11) 3 次
  *             (此时 HAL 库可能已损坏，必须用寄存器直接操作)
  *          3) NVIC_SystemReset() — 触发软复位，尝试恢复
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __disable_irq();  /* 关闭全局中断 */

  /* 快闪 LED_ERR 3 次 (直接寄存器操作，不依赖 HAL 库) */
  for (volatile int i = 0; i < 3; i++) {
    GPIOB->BSRR = GPIO_PIN_11;        /* BSRR 低 16 位: 置位 → LED_ERR 点亮 */
    for (volatile uint32_t d = 0; d < 300000; d++) {} /* 软件延时 ~300ms */
    GPIOB->BSRR = GPIO_PIN_11 << 16;  /* BSRR 高 16 位: 复位 → LED_ERR 熄灭 */
    for (volatile uint32_t d = 0; d < 300000; d++) {} /* 软件延时 ~300ms */
  }
  NVIC_SystemReset();  /* 触发软复位 (AIRCR.SYSRESETREQ) */
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* 不会执行到此处，NVIC_SystemReset() 将先生效 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief  MemManage (内存管理) 故障处理
  * @note   触发场景: MPU 保护区域违规访问。
  *         本项目未启用 MPU，此处理器一般不会触发。
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief  BusFault (总线错误) 处理函数
  * @note   触发场景: 访问无效内存地址、外设未初始化就读写等。
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief  UsageFault (用法错误) 处理函数
  * @note   触发场景: 未定义指令、未对齐访问、除零错误、
  *         非法状态转换 (cpsr T位异常) 等。
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief  SVC (系统服务调用) 处理函数
  * @note   由 SVC 指令触发，常用于 RTOS 内核切换。
  *         本项目为裸机调度器，未使用 SVC。
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief  DebugMon (调试监视) 处理函数
  * @note   仅在连接调试器时使用 (DWT 数据观察点触发)。
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief  PendSV (可挂起系统服务) 处理函数
  * @note   RTOS 中用于上下文切换，本裸机项目未使用。
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief  SysTick \u5b9a\u65f6\u5668\u4e2d\u65ad\u5904\u7406 \u2014 \u7cfb\u7edf\u65f6\u57fa\u6838\u5fc3
  * @details SysTick \u914d\u7f6e\u4e3a 1ms \u4e2d\u65ad (168MHz / 168000 = 1kHz)\u3002
  *          \u6bcf\u6b21\u89e6\u53d1\u6267\u884c\u4e24\u4e2a\u5173\u952e\u64cd\u4f5c:
  *          1) HAL_IncTick()          \u2014 \u9012\u589e HAL \u5e93\u5168\u5c40\u8ba1\u6570\u5668 (HAL_GetTick)
  *          2) scheduler_hw_tick_isr()  \u2014 \u8c03\u5ea6\u5668\u786c\u4ef6\u8282\u62cd:
  *             - \u8c03\u7528\u94a9\u5b50\u51fd\u6570 (Tick_Hook: \u7535\u673a\u9a71\u52a8+IO\u626b\u63cf)
  *             - \u6bcf 1 \u4e2a\u786c\u4ef6\u8282\u62cd\u4ea7\u751f 1 \u4e2a\u5e94\u7528\u8282\u62cd (1ms)
  *             - \u68c0\u67e5\u5e76\u6807\u8bb0\u5230\u671f\u4efb\u52a1\u7684 is_ready \u6807\u5fd7
  * @warning \u6b64 ISR \u662f\u6574\u4e2a\u7cfb\u7edf\u7684\u65f6\u57fa\u6839\u6e90\uff0c\u4f18\u5148\u7ea7\u4e0d\u5e94\u88ab\u5176\u4ed6\u4e2d\u65ad\u957f\u65f6\u95f4\u62a2\u5360
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();             /* HAL \u65f6\u57fa\u8ba1\u6570 +1ms (\u4f9b HAL_Delay/HAL_GetTick \u4f7f\u7528) */
  /* USER CODE BEGIN SysTick_IRQn 1 */
  scheduler_hw_tick_isr();   /* \u8c03\u5ea6\u5668\u786c\u4ef6\u8282\u62cd: \u94a9\u5b50\u8c03\u7528 + \u4efb\u52a1\u5c31\u7eea\u68c0\u67e5 */
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/*            STM32F4xx \u5916\u8bbe\u4e2d\u65ad\u5904\u7406\u51fd\u6570                                    */
/*   \u51fd\u6570\u540d\u5fc5\u987b\u4e0e startup_stm32f407xx.s \u4e2d\u7684\u4e2d\u65ad\u5411\u91cf\u8868\u4e00\u81f4            */
/******************************************************************************/

/* USER CODE BEGIN 1 */

/**
  * @brief  USART1 \u5168\u5c40\u4e2d\u65ad\u5904\u7406\u51fd\u6570
  * @details \u5904\u7406\u6d41\u7a0b (DMA+IDLE \u6a21\u5f0f):
  *          1) BSP_UART_IDLE_Handler() \u2014 \u68c0\u6d4b IDLE \u6807\u5fd7\uff0c\u82e5\u7f6e\u4f4d\u5219:
  *             a) \u6e05\u9664 IDLE \u6807\u5fd7 (SR+DR \u8bfb\u5e8f\u5217)
  *             b) \u505c\u6b62 DMA \u63a5\u6536
  *             c) \u8ba1\u7b97\u63a5\u6536\u957f\u5ea6 = \u603b\u7f13\u51b2\u533a - DMA \u5269\u4f59\u8ba1\u6570
  *             d) \u8bbe\u7f6e frame_ready \u6807\u5fd7
  *             e) \u91cd\u65b0\u542f\u52a8 DMA \u63a5\u6536
  *          2) HAL_UART_IRQHandler() \u2014 \u5904\u7406\u5176\u4ed6\u4e2d\u65ad\u6e90 (ORE/FE/NE \u9519\u8bef\u7b49)
  */
void USART1_IRQHandler(void)
{
  BSP_UART_IDLE_Handler();       /* \u4f18\u5148\u5904\u7406 IDLE \u7a7a\u95f2\u5e27\u68c0\u6d4b */
  HAL_UART_IRQHandler(&huart1);  /* HAL \u6807\u51c6\u4e2d\u65ad\u5904\u7406 (\u9519\u8bef\u6807\u5fd7\u6e05\u9664\u7b49) */
}

/**
  * @brief  DMA2 Stream5 \u4e2d\u65ad\u5904\u7406 \u2014 USART1 RX DMA \u4f20\u8f93\u5b8c\u6210/\u534a\u5b8c\u6210
  * @note   DMA2_Stream5 \u5728 STM32F407 DMA \u901a\u9053\u6620\u5c04\u4e2d\u5bf9\u5e94 USART1_RX (Channel 4)
  */
void DMA2_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart1.hdmarx);  /* \u4ea4\u7ed9 HAL DMA \u5904\u7406 RX \u4e2d\u65ad\u6807\u5fd7 */
}

/**
  * @brief  DMA2 Stream7 \u4e2d\u65ad\u5904\u7406 \u2014 USART1 TX DMA \u4f20\u8f93\u5b8c\u6210
  * @note   DMA2_Stream7 \u5728 STM32F407 DMA \u901a\u9053\u6620\u5c04\u4e2d\u5bf9\u5e94 USART1_TX (Channel 4)
  */
void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart1.hdmatx);  /* \u4ea4\u7ed9 HAL DMA \u5904\u7406 TX \u4e2d\u65ad\u6807\u5fd7 */
}


/* ==========================================================================
 * RS485 (USART3) interrupts. Separate from USART1/DMA2 above.
 * ========================================================================== */

void USART3_IRQHandler(void)
{
  BSP_UART485_IDLE_Handler();
  HAL_UART_IRQHandler(&huart3);
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart3.hdmarx);
}

void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(huart3.hdmatx);
}

void TIM5_IRQHandler(void)
{
  SoftPWM_TimerElapsedISR(SoftPWM_GetTimHandle());
}

/* ---------- \u5b9a\u65f6\u5668 CC \u4e2d\u65ad\u5904\u7406 (\u7535\u673a PWM \u8109\u51b2\u8ba1\u6570) ---------- */

/**
  * @brief  TIM1 Capture Compare \u4e2d\u65ad \u2014 Y3 (PA8, TIM1_CH1) \u8109\u51b2\u8ba1\u6570
  */
void TIM1_CC_IRQHandler(void)
{
  /* 通过 PWM_GetTimHandle() 获取 Y3 通道 (PA8) 所绑定的 TIM1 HAL 句柄,
   * 交由 HAL 库统一处理 CC 中断: 清除中断标志 → 调用 PulseFinishedCallback */
  HAL_TIM_IRQHandler(PWM_GetTimHandle(PWM_CH_Y3));
}

/**
  * @brief  TIM3 \u5168\u5c40\u4e2d\u65ad \u2014 Y16 (PB5, TIM3_CH2) / Y17 (PB4, TIM3_CH1) \u8109\u51b2\u8ba1\u6570
  */
void TIM3_IRQHandler(void)
{
  /* 通过 PWM_GetTimHandle() 获取 Y16 通道 (PB5) 所绑定的 TIM3 HAL 句柄,
   * TIM3 同时服务 Y16(CH2) 和 Y17(CH1), HAL 内部根据 SR 标志区分具体通道 */
  HAL_TIM_IRQHandler(PWM_GetTimHandle(PWM_CH_Y16));
}

/**
  * @brief  TIM4 \u5168\u5c40\u4e2d\u65ad \u2014 Y4 (PB7) / Y5 (PB8) / Y15 (PB6) \u8109\u51b2\u8ba1\u6570
  */
void TIM4_IRQHandler(void)
{
  /* 通过 PWM_GetTimHandle() 获取 Y4 通道 (PB7) 所绑定的 TIM4 HAL 句柄,
   * TIM4 同时服务 Y4(CH2)/Y5(CH3)/Y15(CH1), HAL 内部根据 SR 标志区分具体通道 */
  HAL_TIM_IRQHandler(PWM_GetTimHandle(PWM_CH_Y4));
}

/**
  * @brief  TIM8 Capture Compare \u4e2d\u65ad \u2014 Y1 (PC6, TIM8_CH1) / Y2 (PC7, TIM8_CH2) \u8109\u51b2\u8ba1\u6570
  */
void TIM8_CC_IRQHandler(void)
{
  /* 通过 PWM_GetTimHandle() 获取 Y1 通道 (PC6) 所绑定的 TIM8 HAL 句柄,
   * TIM8 同时服务 Y1(CH1) 和 Y2(CH2), HAL 内部根据 SR 标志区分具体通道 */
  HAL_TIM_IRQHandler(PWM_GetTimHandle(PWM_CH_Y1));
}

/**
  * @brief  HAL PWM \u8109\u51b2\u5b8c\u6210\u56de\u8c03 \u2014 \u6bcf\u4e2a PWM \u5468\u671f\u89e6\u53d1\u4e00\u6b21
  * @note   \u7531 HAL_TIM_IRQHandler \u5185\u90e8\u8c03\u7528\uff0c\u8f6c\u53d1\u7ed9\u8f74\u63a7\u5236\u6a21\u5757\u8fdb\u884c\u8109\u51b2\u8ba1\u6570
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* 将 PWM 脉冲完成事件转发给轴控制模块的中断服务函数;
   * Axis_PulseISR 内部流程: PWM_IdentifyIRQ 识别通道 → 反查轴 ID
   * → 更新绝对位置 position±1 → 若 use_counting 为真则检查是否到达目标步数 */
  Axis_PulseISR(htim);
}

/* USER CODE END 1 */
