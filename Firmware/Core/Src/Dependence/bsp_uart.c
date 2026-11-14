/**
 * @file    bsp_uart.c
 * @brief   板级 USART1 串口驱动源文件 — DMA 收发 + IDLE 空闲帧检测
 * @details 本驱动采用 STM32 DMA + IDLE 中断的经典不定长帧接收方案:
 *
 *          【接收流程】
 *          1. DMA 在后台将 USART1 接收到的字节连续写入 rx_dma_buf[]
 *          2. 当总线出现一帧空闲 (超过 1 个字节时间无数据) 时，USART1 触发 IDLE 中断
 *          3. IDLE 中断处理函数停止 DMA，计算本帧长度，拷贝到 rx_frame_buf[]
 *          4. 上层通过 BSP_UART_FrameReady() 查询、BSP_UART_GetFrame() 取帧
 *          5. 处理完成后调用 BSP_UART_ReleaseFrame() 清除标志
 *
 *          【发送流程】
 *          调用 BSP_UART_Send() → 等待上次 DMA 发送完成 → 启动新的 DMA 发送
 *
 *          【硬件资源】
 *          - USART1: PA9(TX) / PA10(RX)
 *          - DMA2 Stream5 Channel4: USART1_RX
 *          - DMA2 Stream7 Channel4: USART1_TX
 */

#include "bsp_uart.h"
#include "bsp_uart485.h"
#include "app_config.h"  /* 引入 DEBUG_PRINTF 条件编译宏: 控制 fputc/printf 重定向是否编译 */
#include <string.h>

/* ==========================================================================
 *  全局 / 静态变量定义
 * ========================================================================== */

UART_HandleTypeDef huart1;                          /**< USART1 HAL 句柄 (全局，供中断函数引用) */
static DMA_HandleTypeDef hdma_usart1_rx;            /**< DMA2 Stream5 — USART1 接收通道句柄 */
static DMA_HandleTypeDef hdma_usart1_tx;            /**< DMA2 Stream7 — USART1 发送通道句柄 */

/* ---- 双缓冲机制 ----
 * rx_dma_buf[]:   DMA 硬件直接写入的缓冲区 (可能随时被下一帧覆盖)
 * rx_frame_buf[]: IDLE 中断中将完整帧拷贝到两个槽位之一，供上层安全读取
 */
static uint8_t rx_dma_buf[UART_RX_BUF_SIZE];         /**< DMA 接收原始缓冲区 */
static uint8_t rx_frame_buf[2][UART_RX_BUF_SIZE];    /**< 双帧安全拷贝缓冲区 */
static volatile uint16_t rx_frame_len[2] = {0, 0};   /**< 两个槽位各自的有效字节数 */
static volatile uint8_t rx_frame_read_idx = 0;       /**< 当前供上层读取的槽位索引 */
static volatile uint8_t rx_frame_write_idx = 0;      /**< 下一帧写入槽位索引 */
static volatile uint8_t rx_frame_count = 0;          /**< 当前待处理完整帧个数，范围 0~2 */
static volatile uint16_t rx_drop_count = 0;          /**< 上一帧未处理时被丢弃的完整帧计数 */

/* ==========================================================================
 *  BSP_UART_Init() — USART1 参数配置 + IDLE 中断 + DMA 启动
 * ========================================================================== */

/**
 * @brief  初始化 USART1 (115200-8N1) 并启动 DMA 接收
 * @details 配置流程:
 *          1) 填充 HAL 串口参数结构体并调用 HAL_UART_Init()
 *             → 内部会回调 HAL_UART_MspInit() 完成 GPIO/DMA/NVIC 配置
 *          2) 使能 IDLE 线路空闲中断 (用于帧结束检测)
 *          3) 启动 DMA 连续接收
 */
void BSP_UART_Init(void)
{
  /* ---- 步骤 1: 配置 USART1 通信参数 ---- */
  huart1.Instance          = USART1;                  /* 使用 USART1 外设 */
  huart1.Init.BaudRate     = UART_BAUDRATE;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;      /* 8 位数据位 */
  huart1.Init.StopBits     = UART_STOPBITS_1;         /* 1 位停止位 */
  huart1.Init.Parity       = UART_PARITY_NONE;        /* 无校验位 */
  huart1.Init.Mode         = UART_MODE_TX_RX;         /* 全双工收发 */
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;     /* 无硬件流控 (工控场景通常不用) */
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;    /* 16 倍过采样 (提高抗噪声能力) */

  /* 调用 HAL 初始化，内部自动回调 HAL_UART_MspInit() 完成底层配置 */
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();  /* 初始化失败：进入错误处理 (闪灯 + 复位) */
  }

  /* ---- 步骤 2: 使能 IDLE 空闲中断 ---- */
  /* IDLE 中断在总线空闲超过 1 个字节传输时间后触发，用作"帧结束"标志 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

  /* ---- 步骤 3: 启动 DMA 接收 ---- */
  /* DMA 模式为 NORMAL (非循环)。每次帧结束后在 IDLE 中断中停止并重启 */
  HAL_UART_Receive_DMA(&huart1, rx_dma_buf, UART_RX_BUF_SIZE);
}

/* ==========================================================================
 *  HAL_UART_MspInit() — USART1 底层硬件初始化 (MSP 回调)
 * ==========================================================================
 * 该函数由 HAL_UART_Init() 内部自动调用，负责:
 * - 使能外设时钟 (USART1 + GPIOA + DMA2)
 * - 配置 USART1 的 TX/RX 引脚为复用推挽模式
 * - 配置并启动 DMA 接收/发送通道
 * - 配置 NVIC 中断优先级
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if(huart->Instance == USART1)
  {
    /* ---- 1. 使能外设时钟 ---- */
    __HAL_RCC_USART1_CLK_ENABLE();   /* USART1 外设时钟 (挂在 APB2 总线，84MHz) */
    __HAL_RCC_GPIOA_CLK_ENABLE();    /* GPIOA 端口时钟 (PA9=TX, PA10=RX) */
    __HAL_RCC_DMA2_CLK_ENABLE();     /* DMA2 时钟 (USART1 的 TX/RX DMA 通道均在 DMA2 上) */

    /* ---- 2. 配置 GPIO 引脚: PA9(TX) + PA10(RX) ---- */
    GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;  /* TX=PA9, RX=PA10 */
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;            /* 复用推挽输出 (USART 功能需要) */
    GPIO_InitStruct.Pull      = GPIO_PULLUP;                /* 上拉 (总线空闲时保持高电平，防止误触发) */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;  /* 最高速率 (减小上升下降沿时间) */
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;            /* 复用功能 AF7 = USART1 */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ---- 3. 配置 DMA RX: DMA2 Stream5 Channel4 (USART1_RX) ---- */
    /* 参考 RM0090 Table 43: DMA2 Stream5 Channel4 = USART1_RX */
    hdma_usart1_rx.Instance                 = DMA2_Stream5;         /* DMA2 流 5 */
    hdma_usart1_rx.Init.Channel             = DMA_CHANNEL_4;        /* 通道 4 = USART1_RX */
    hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY; /* 方向: 外设(USART DR) → 内存(rx_dma_buf) */
    hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;     /* 外设地址不递增 (始终读 USART1->DR) */
    hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;      /* 内存地址递增 (逐字节写入缓冲区) */
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据宽度: 8位 (UART 字符宽度) */
    hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;  /* 内存数据宽度: 8位 */
    hdma_usart1_rx.Init.Mode                = DMA_NORMAL;           /* 普通模式 (非循环，接收完 N 字节后停止) */
    hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;    /* 高优先级 (接收数据不能丢失) */
    hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; /* 禁用 FIFO (直接模式，降低延迟) */
    HAL_DMA_Init(&hdma_usart1_rx);
    __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);  /* 将 DMA 句柄绑定到 UART 句柄的 hdmarx 指针 */

    /* ---- 4. 配置 DMA TX: DMA2 Stream7 Channel4 (USART1_TX) ---- */
    /* 参考 RM0090 Table 43: DMA2 Stream7 Channel4 = USART1_TX */
    hdma_usart1_tx.Instance                 = DMA2_Stream7;         /* DMA2 流 7 */
    hdma_usart1_tx.Init.Channel             = DMA_CHANNEL_4;        /* 通道 4 = USART1_TX */
    hdma_usart1_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH; /* 方向: 内存 → 外设(USART DR) */
    hdma_usart1_tx.Init.PeriphInc           = DMA_PINC_DISABLE;     /* 外设地址不递增 (始终写 USART1->DR) */
    hdma_usart1_tx.Init.MemInc              = DMA_MINC_ENABLE;      /* 内存地址递增 (逐字节读取缓冲区) */
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;  /* 外设数据宽度: 8位 */
    hdma_usart1_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;  /* 内存数据宽度: 8位 */
    hdma_usart1_tx.Init.Mode                = DMA_NORMAL;           /* 普通模式 (发送完成后停止) */
    hdma_usart1_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;  /* 中优先级 (发送可容忍少量延迟) */
    hdma_usart1_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE; /* 禁用 FIFO */
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);  /* 将 DMA 句柄绑定到 UART 句柄的 hdmatx 指针 */

    /* ---- 5. NVIC 中断优先级配置 ---- */
    /* 抢占优先级 1 (仅次于 SysTick)，确保串口数据不丢失 */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 1, 0);  /* RX DMA 中断: 抢占=1, 子优先级=0 */
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1, 1);  /* TX DMA 中断: 抢占=1, 子优先级=1 */
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);        /* USART1 中断 (含 IDLE): 抢占=1, 子优先级=0 */
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  }
  else if (huart->Instance == USART3)
  {
    /* RS485 uses an independent driver and DMA1; the USART1 branch above
       is intentionally left unchanged. */
    BSP_UART485_MspInit(huart);
  }
}

/* ==========================================================================
 *  BSP_UART_IDLE_Handler() — USART1 IDLE 空闲中断处理
 * ==========================================================================
 * 该函数在 USART1_IRQHandler() (stm32f4xx_it.c) 中被调用。
 *
 * 原理: 当 USART1 RX 线上超过 1 个字节传输时间没有新数据到达时，
 *       硬件自动置位 IDLE 标志。此时表示"一帧数据已全部到达"。
 *
 * 处理流程:
 * 1) 检测并清除 IDLE 标志 (先读 SR 再读 DR，符合 STM32F4 参考手册要求)
 * 2) 停止 DMA 接收，通过 DMA 剩余计数反算本帧字节数
 * 3) 将帧数据从 DMA 缓冲区拷贝到帧缓冲区 (防止重启 DMA 后覆盖)
 * 4) 置位 rx_frame_ready 标志，通知上层有新帧
 * 5) 重新启动 DMA 接收下一帧
 */
void BSP_UART_IDLE_Handler(void)
{
  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))  /* 检查 IDLE 标志是否置位 */
  {
    /* 清除 IDLE 标志: STM32F4 需要依次读 SR 和 DR 来清除 (由 HAL 宏自动完成) */
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    /* 停止 DMA 传输，使 NDTR (剩余计数) 值冻结，便于计算已接收长度 */
    HAL_UART_DMAStop(&huart1);

    /* 计算本帧实际接收字节数 = 总缓冲区大小 - DMA 剩余未传输字节数 */
    uint16_t dma_remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    uint16_t frame_len = UART_RX_BUF_SIZE - dma_remaining;

    /* 有效帧检查: 长度 > 0 且不超过缓冲区上限 */
    if (frame_len > 0 && frame_len <= UART_RX_BUF_SIZE)
    {
      /* 双帧排队: 两个槽位都满时丢弃最新帧并累计计数 */
      if (rx_frame_count < 2U)
      {
        memcpy(rx_frame_buf[rx_frame_write_idx], rx_dma_buf, frame_len);
        rx_frame_len[rx_frame_write_idx] = frame_len;
        rx_frame_write_idx ^= 1U;
        rx_frame_count++;
      }
      else
      {
        rx_drop_count++;
      }
    }

    /* 重新启动 DMA 接收，准备接收下一帧 */
    HAL_UART_Receive_DMA(&huart1, rx_dma_buf, UART_RX_BUF_SIZE);
  }
}

/* ==========================================================================
 *  BSP_UART_Send() — DMA 非阻塞发送 (带忙等待超时保护)
 * ========================================================================== */

/**
 * @brief  通过 DMA 发送数据
 * @param  data 待发送数据指针
 * @param  len  待发送字节数
 * @note   如果上一次 DMA 发送尚未完成，最多等待 100ms 后放弃本次发送。
 *         该设计避免了主循环因发送阻塞导致看门狗超时复位。
 */
bool BSP_UART_Send(const uint8_t *data, uint16_t len)
{
  if (data == NULL || len == 0U || len > UART_TX_BUF_SIZE) {
    return false;
  }

  /* 等待上次 DMA 发送完成 (轮询 HAL 状态机，带超时保护) */
  uint32_t start = HAL_GetTick();
  while (huart1.gState != HAL_UART_STATE_READY) {
    if ((HAL_GetTick() - start) > 100U) return false;
  }
  /* 启动 DMA 发送 — 数据通过 DMA 自动搬运到 USART1->DR，CPU 无需参与 */
  if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)data, len) != HAL_OK) {
    return false;
  }

  start = HAL_GetTick();
  while (huart1.gState != HAL_UART_STATE_READY) {
    if ((HAL_GetTick() - start) > 100U) {
      HAL_UART_AbortTransmit(&huart1);
      return false;
    }
  }

  start = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
    if ((HAL_GetTick() - start) > 20U) return false;
  }
  return true;
}

/* ==========================================================================
 *  帧查询与释放接口
 * ========================================================================== */

/**
 * @brief  查询是否有完整帧待处理
 * @return true = 有新帧 (由 IDLE 中断置位)
 */
bool BSP_UART_FrameReady(void)
{
  return rx_frame_count > 0U;
}

/**
 * @brief  获取最近接收到的帧数据
 * @param[out] buf 返回帧数据缓冲区指针 (指向静态内部数组，无需外部分配)
 * @param[out] len 返回帧数据长度
 * @note   该指针在调用 BSP_UART_ReleaseFrame() 前有效；
 *         不要在两帧之间长期持有该指针。
 */
void BSP_UART_GetFrame(uint8_t **buf, uint16_t *len)
{
  uint8_t idx = rx_frame_read_idx;
  *buf = rx_frame_buf[idx];
  *len = rx_frame_len[idx];
}

/**
 * @brief  释放帧 — 调用此函数表示上层已处理完当前帧
 * @note   释放后 BSP_UART_FrameReady() 将返回 false，直到下一帧到达
 */
void BSP_UART_ReleaseFrame(void)
{
  ATOMIC_ENTER();
  if (rx_frame_count > 0U) {
    rx_frame_len[rx_frame_read_idx] = 0;
    rx_frame_read_idx ^= 1U;
    rx_frame_count--;
  }
  ATOMIC_EXIT();
}

uint16_t BSP_UART_GetDroppedFrames(void)
{
  return rx_drop_count;
}

/* ==========================================================================
 *  printf 重定向 (仅供调试使用)
 * ========================================================================== */

/**
 * @brief  重定向 fputc 到 USART1，使 printf() 可通过串口输出
 * @note   使用阻塞发送 (HAL_UART_Transmit) 确保每个字符都可靠输出。
 *         仅在调试阶段使用；量产固件中 DEBUG_PRINTF 未定义，printf 为空操作。
 */
#ifdef DEBUG_PRINTF  /* 仅当 app_config.h 中定义了 DEBUG_PRINTF 时编译以下代码 */
/**
 * @brief  重定向 C 标准库 fputc 到 USART1, 使 printf() 可通过串口输出调试信息
 * @param  ch 待发送的字符 (ASCII 码)
 * @param  f  文件流指针 (标准库内部使用, 此处忽略)
 * @return 发送的字符值 (与 ch 相同, 符合 fputc 返回值规范)
 * @note   使用 HAL 阻塞发送确保字符顺序正确; 量产时注释掉 DEBUG_PRINTF 即可消除
 */
int fputc(int ch, FILE *f)
{
    /* 通过 HAL 阻塞式发送单字节到 USART1, 超时 10ms 防止串口故障导致死锁;
     * (uint8_t *)&ch: 将 int 型字符的低字节地址传给 HAL */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 10);
    return ch;  /* 返回已发送的字符, 供 printf 内部链式调用 */
}
#endif  /* DEBUG_PRINTF */
