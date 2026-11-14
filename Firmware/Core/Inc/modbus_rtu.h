/**
 * @file    modbus_rtu.h
 * @brief   Modbus RTU 从机协议栈公共接口。
 * @details 协议栈只负责帧校验、功能码解析、异常响应和响应帧组装，具体寄存器、
 *          线圈及串口发送行为全部由应用层回调提供。当前支持 FC01、FC02、FC03、
 *          FC05、FC06 和 FC16。所有应用回调成功时返回 MB_CB_OK，失败时返回标准
 *          Modbus 异常码。
 */
#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdbool.h>
#include <stdint.h>

#define MODBUS_RX_BUF_SIZE 256U /**< 单帧接收缓冲区容量。 */
#define MODBUS_TX_BUF_SIZE 256U /**< 单帧发送缓冲区容量。 */
#define MODBUS_HOLDING_REG_COUNT 100U /**< 内置保持寄存器数量，原始地址 0~99。 */

#define MB_FUNC_READ_COILS                0x01U
#define MB_FUNC_READ_DISCRETE_INPUTS      0x02U
#define MB_FUNC_READ_HOLDING_REGISTERS    0x03U
#define MB_FUNC_WRITE_SINGLE_COIL         0x05U
#define MB_FUNC_WRITE_SINGLE_REGISTER     0x06U
#define MB_FUNC_WRITE_MULTIPLE_REGISTERS  0x10U

#define MB_EX_ILLEGAL_FUNCTION            0x01U
#define MB_EX_ILLEGAL_DATA_ADDRESS        0x02U
#define MB_EX_ILLEGAL_DATA_VALUE          0x03U
#define MB_EX_SLAVE_DEVICE_FAILURE        0x04U

/* 应用回调成功时返回 0，失败时返回一个标准 Modbus 异常码。 */
#define MB_CB_OK                          0x00U
#define MB_WRITE_OK                       MB_CB_OK

/**
 * @brief 内置保持寄存器区，直接对应 Modbus 原始地址 0~99。
 * @note  业务代码可直接读写该数组；单个 uint16_t 访问在 Cortex-M4 上是原子的。
 */
extern volatile uint16_t Modbus_HoldingRegisters[MODBUS_HOLDING_REG_COUNT];

typedef struct {
    uint8_t slave_addr; /**< 本机从站地址，有效范围通常为 1 至 247。 */

    uint8_t rx_buf[MODBUS_RX_BUF_SIZE]; /**< 当前请求帧的内部副本。 */
    uint16_t rx_len;                    /**< 当前请求帧长度。 */
    uint8_t tx_buf[MODBUS_TX_BUF_SIZE]; /**< 响应帧组装缓冲区。 */
    uint16_t tx_len;                    /**< 尚未附加 CRC 时的响应长度。 */

    /** @brief 发送完整 RTU 响应帧的回调。 */
    void (*uart_send)(uint8_t *data, uint16_t len);

    /** @brief 按地址读取线圈、离散输入和保持寄存器的应用回调。 */
    uint8_t (*read_coil)(uint16_t addr, uint8_t *value);
    uint8_t (*read_discrete_input)(uint16_t addr, uint8_t *value);
    uint8_t (*read_holding_register)(uint16_t addr, uint16_t *value);

    /** @brief 按地址写入线圈和保持寄存器的应用回调。 */
    uint8_t (*write_coil)(uint16_t addr, uint8_t state);
    uint8_t (*write_holding_register)(uint16_t addr, uint16_t value);

    /* FC16 事务钩子用于暂存多寄存器写入，并在全部成功后原子提交。 */
    bool (*begin_write_holding_registers)(uint16_t start_addr, uint16_t quantity);
    void (*end_write_holding_registers)(bool commit);
} Modbus_Slave_t;

/** @brief 清空从机上下文并设置从站地址；初始化后由应用层绑定回调。 */
void Modbus_Init(Modbus_Slave_t *mb, uint8_t slave_addr);
/**
 * @brief 校验并处理一帧完整的 Modbus RTU 请求。
 * @note 地址不匹配、长度非法或 CRC 错误的帧会被静默丢弃；协议级参数错误会返回异常帧。
 */
void Modbus_ProcessFrame(Modbus_Slave_t *mb, uint8_t *data, uint16_t len);
/** @brief 计算 Modbus 标准 CRC16，返回值低字节应先发送。 */
uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len);

#endif /* MODBUS_RTU_H */
