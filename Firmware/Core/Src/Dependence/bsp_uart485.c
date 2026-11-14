/**
 * @file    bsp_uart485.c
 * @brief   Independent RS485 driver (USART3 + DMA1 + PB14 direction control).
 *
 * Hardware mapping on C2416MR:
 *   PC10 = USART3_TX (AF7)
 *   PC11 = USART3_RX (AF7)
 *   PB14 = DE + /RE, 0 = receive, 1 = transmit
 *   DMA1 Stream1 Channel4 = USART3_RX
 *   DMA1 Stream3 Channel4 = USART3_TX
 *
 * The RS232 path in bsp_uart.c (USART1/DMA2) is not shared or modified.
 */

#include "bsp_uart485.h"
#include "app_config.h"
#include <string.h>

UART_HandleTypeDef huart3;

static DMA_HandleTypeDef hdma_usart3_rx;
static DMA_HandleTypeDef hdma_usart3_tx;

static uint8_t uart485_rx_dma_buf[UART485_RX_BUF_SIZE];
static uint8_t uart485_rx_frame_buf[2][UART485_RX_BUF_SIZE];
static volatile uint16_t uart485_rx_frame_len[2] = {0U, 0U};
static volatile uint8_t uart485_rx_read_idx = 0U;
static volatile uint8_t uart485_rx_write_idx = 0U;
static volatile uint8_t uart485_rx_frame_count = 0U;
static volatile uint16_t uart485_rx_drop_count = 0U;

static uint8_t uart485_tx_buf[UART485_TX_BUF_SIZE];
static volatile bool uart485_tx_busy = false;

static void uart485_set_rx_mode(void)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
}

static void uart485_set_tx_mode(void)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_PORT, RS485_DE_PIN, GPIO_PIN_SET);
}

static void uart485_start_rx(void)
{
    (void)HAL_UART_Receive_DMA(&huart3, uart485_rx_dma_buf, UART485_RX_BUF_SIZE);
}

void BSP_UART485_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance != USART3) {
        return;
    }

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* PC10 TX / PC11 RX */
    GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* PB14 direction control; low first so the bus is in receive mode. */
    HAL_GPIO_WritePin(RS485_DE_GPIO_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = RS485_DE_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_DE_GPIO_PORT, &GPIO_InitStruct);

    /* USART3 RX: DMA1 Stream1 Channel4 */
    hdma_usart3_rx.Instance                 = DMA1_Stream1;
    hdma_usart3_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode                = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

    /* USART3 TX: DMA1 Stream3 Channel4 */
    hdma_usart3_tx.Instance                 = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode                = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority            = DMA_PRIORITY_MEDIUM;
    hdma_usart3_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

    /* Lower priority than USART1 and motor timers (both use preemption 1).
     * NOTE: the C2416MR reference program routes USART3 through PC10/PC11.
     * Verify the actual board revision before production use. */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void BSP_UART485_Init(uint32_t baudrate)
{
    uart485_tx_busy = false;
    uart485_set_rx_mode();

    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = baudrate;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }

    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    uart485_start_rx();
}

bool BSP_UART485_Send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > UART485_TX_BUF_SIZE || uart485_tx_busy) {
        return false;
    }

    /* Half duplex: stop RX DMA before driving the differential bus. */
    (void)HAL_UART_AbortReceive(&huart3);

    memcpy(uart485_tx_buf, data, len);
    uart485_tx_busy = true;
    uart485_set_tx_mode();

    if (HAL_UART_Transmit_DMA(&huart3, uart485_tx_buf, len) != HAL_OK) {
        uart485_set_rx_mode();
        uart485_tx_busy = false;
        uart485_start_rx();
        return false;
    }
    return true;
}

bool BSP_UART485_TxBusy(void)
{
    return uart485_tx_busy;
}

bool BSP_UART485_FrameReady(void)
{
    return uart485_rx_frame_count > 0U;
}

void BSP_UART485_GetFrame(uint8_t **buf, uint16_t *len)
{
    uint8_t idx = uart485_rx_read_idx;
    *buf = uart485_rx_frame_buf[idx];
    *len = uart485_rx_frame_len[idx];
}

void BSP_UART485_ReleaseFrame(void)
{
    ATOMIC_ENTER();
    if (uart485_rx_frame_count > 0U) {
        uart485_rx_frame_len[uart485_rx_read_idx] = 0U;
        uart485_rx_read_idx ^= 1U;
        uart485_rx_frame_count--;
    }
    ATOMIC_EXIT();
}

uint16_t BSP_UART485_GetDroppedFrames(void)
{
    return uart485_rx_drop_count;
}

void BSP_UART485_IDLE_Handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) == RESET) {
        return;
    }

    __HAL_UART_CLEAR_IDLEFLAG(&huart3);

    /* Abort RX only; never use HAL_UART_DMAStop() here because it can
       also stop an in-progress TX DMA transfer. */
    (void)HAL_UART_AbortReceive(&huart3);

    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart3.hdmarx);
    uint16_t frame_len = (remaining <= UART485_RX_BUF_SIZE)
                       ? (uint16_t)(UART485_RX_BUF_SIZE - remaining) : 0U;

    if (frame_len > 0U) {
        if (uart485_rx_frame_count < 2U) {
            memcpy(uart485_rx_frame_buf[uart485_rx_write_idx],
                   uart485_rx_dma_buf, frame_len);
            uart485_rx_frame_len[uart485_rx_write_idx] = frame_len;
            uart485_rx_write_idx ^= 1U;
            uart485_rx_frame_count++;
        } else {
            uart485_rx_drop_count++;
        }
    }

    uart485_start_rx();
}

/* HAL callbacks. Only USART3 is handled here; USART1 has its own path. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart485_set_rx_mode();
        uart485_tx_busy = false;
        uart485_start_rx();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart485_set_rx_mode();
        uart485_tx_busy = false;
        (void)HAL_UART_AbortReceive(&huart3);
        uart485_start_rx();
    }
}