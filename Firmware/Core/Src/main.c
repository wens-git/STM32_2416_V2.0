/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 系统主程序，负责启动、时钟配置和主循环调度。
  * @details        本文件是 STM32F407 2416 工业控制系统的入口，负责:
 *                 1) HAL 库初始化 + 系统时钟配置 (HSI 16MHz → PLL → 168MHz)
 *                 2) 独立看门狗 (IWDG) 启动
 *                 3) IO 初始化 (24路输入消抖 + 16路输出)
  *                 4) 板级外设初始化 (UART DMA+IDLE 等)
  *                 5) 步进电机注册 (脉冲 + 方向引脚回调)
  *                 6) 协作式调度器初始化与任务注册
  *                 7) 主循环中仅运行 scheduler_run() 调度就绪任务
  *
 *                 【系统启动顺序】
 *                 HAL_Init → SystemClock_Config → BSP_IWDG_Init
 *                   → IO_Init → System_Init
 *                   → (BSP_UART_Init → PWM_Motor_Init → App_Init → Scheduler_Setup)
 *                 → while(1) { scheduler_run(); }
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tasks/main_task.h"   /* 调度器任务适配层: Task_Main */
#include "app/app.h"           /* 应用层总入口: 正常业务 + 调试服务 */
#include "IODEF.h"             /* IO 定义与输入消抖: IO_Init, IO_Scan, Xn/Yn */
#include "app_config.h"        /* UART_LINK_TEST_ENABLE 等全局开关 */
#include "bsp_uart.h"          /* USART1 DMA+IDLE 驱动: BSP_UART_Init, huart1 */
#include "bsp_uart485.h"       /* USART3 DMA+IDLE: BSP_UART485_Init, PB14 direction */
#include "bsp_iwdg.h"          /* 独立看门狗驱动: BSP_IWDG_Init, BSP_IWDG_Feed */
#include "pwm_motor.h"         /* 硬件 PWM 电机控制: PWM_Motor_Init, PWM_Enable/Disable */
#include "motor_axis.h"        /* 多轴运动控制: Axis_Create, Axis_Jog/MoveRel/MoveAbs/Home */
#include "scheduler.h"         /* 协作式调度器: scheduler_init, scheduler_add, scheduler_run */
#include <stdio.h>             /* printf 重定向 (通过 bsp_uart.c 中的 fputc) */
#include <stdbool.h>           /* bool 类型定义 */
#include "soft_pwm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void); /* 系统时钟配置: HSI 16MHz → PLL → 168MHz */
/* USER CODE BEGIN PFP */
/* ---- 系统初始化架构函数 (分层设计，便于维护) ---- */
static void System_Init(void); /* 系统整体初始化入口 */
static void Scheduler_Setup(void); /* 调度器初始化 + 任务注册 */

/* ---- 钩子函数与周期任务 ---- */
void Tick_Hook(void); /* 1ms 硬件节拍钩子: 电机驱动 + 输入扫描 */
void Task_WDT_Feed(void); /* 看门狗喂狗任务 + 运行指示灯 */
#if UART_LINK_TEST_ENABLE
static void Task_UART_LinkTest(void); /* USART1 周期链路测试发送 */
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  1ms 硬件节拍钩子 — 由 scheduler_register_hook() 注册
 * @note   在 SysTick ISR → scheduler_hw_tick_isr() 中被调用，
 *         执行周期严格的实时操作。
 */
void Tick_Hook(void) {
    IO_Scan(); /* 输入端口消抖扫描 (每 1ms 采样一次) */
    Axis_Tick(); /* 多轴运动控制: 使用本周期最新输入状态做传感器监控 + 回原状态机 */
    App_Tick1ms(); /* 应用层 1ms 节拍: 业务轻量检测 + 调试通信看门狗 */
}

/**
 * @brief  看门狗喂狗任务 + 运行指示灯闪烁
 * @note   由调度器周期调用 (默认 500ms)
 */
void Task_WDT_Feed(void) {
    BSP_IWDG_Feed(); /* 重载独立看门狗计数器，防止超时复位 */

    static uint8_t led_toggle = 0;
    led_toggle ^= 1;
    LED_RUN(led_toggle);
}

#if UART_LINK_TEST_ENABLE
/**
 * @brief  周期发送链路测试帧 (默认 100ms 一次, 见 UART_LINK_TEST_PERIOD_MS)
 */
static void Task_UART_LinkTest(void) {
    BSP_UART_LinkTest_Ping();
}
#endif

/**
 * @brief  调度器初始化 + 任务注册
 * @note   【TODO: 根据项目需求注册任务函数和周期】
 */
static void Scheduler_Setup(void) {
    scheduler_init();
    scheduler_register_hook(Tick_Hook);

    /* 核心任务 — 必须注册 */
    scheduler_add(Task_Main, 1);  /* Modbus 帧检测: 1ms */
#if UART_LINK_TEST_ENABLE
    scheduler_add(Task_UART_LinkTest, UART_LINK_TEST_PERIOD_MS / SCHEDULER_TICK_MS);
#endif
    scheduler_add(Task_WDT_Feed, TASK_WDT_PERIOD_MS / SCHEDULER_TICK_MS);
}

/**
 * @brief  系统整体初始化入口 — 依次调用各子模块初始化
 */
static void System_Init(void) {
    BSP_UART_Init();

#if RS485_ENABLE
    /* Independent USART3/DMA1/PB14 path; USART1 configuration is untouched. */
    BSP_UART485_Init(RS485_BAUDRATE);
#endif

    PWM_Motor_Init();
    if (!SoftPWM_Init()) {
        Error_Handler();
    }
    App_Init();
    Scheduler_Setup();

#ifdef DEBUG_PRINTF
    printf("\r\n--- STM32F407 2416 Init Completed ---\r\n");
#endif
}

/* USER CODE END 0 */

/**
 * @brief  应用程序入口点
 */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    BSP_IWDG_Init(SYSTEM_IWDG_TIMEOUT_MS);

    IO_Init();
    System_Init();

    while (1) {
        scheduler_run();
        __WFI();
    }
}

/**
 * @brief  系统时钟配置 — HSI 16MHz → PLL → SYSCLK 168MHz
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief  错误处理 — LED 闪烁后系统复位
 */
void Error_Handler(void) {
    __disable_irq();

    for (volatile uint32_t i = 0; i < 5; i++) {
        LED_ERR(1);
        for (volatile uint32_t d = 0; d < 500000; d++) {
        }
        LED_ERR(0);
        for (volatile uint32_t d = 0; d < 500000; d++) {
            
        }
    }
    NVIC_SystemReset();
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */
