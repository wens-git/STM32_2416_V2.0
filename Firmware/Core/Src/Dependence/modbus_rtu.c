/**
 * @file    modbus_rtu.c
 * @brief   Modbus RTU 从机通信协议栈源文件 (工业级优化版)
 * @details 实现标准 Modbus RTU 从机报文解析与响应构建。
 *          采用基于结构体回调的面向对象设计，解除了与具体 GPIO / 全局变量的硬编码绑定。
 *
 *          ┌─────────────────────────────────────────────────────────┐
 *          │              Modbus RTU 帧格式 (从机视角)               │
 *          ├────────┬───────────┬──────────────┬──────────┤
 *          │ 地址(1)│ 功能码(1) │ 数据域(N)    │ CRC16(2) │
 *          ├────────┴───────────┴──────────────┴──────────┤
 *          │  CRC = 低字节在前 + 高字节在后 (Modbus 标准)  │
 *          └─────────────────────────────────────────────────────────┘
 *
 * @version 1.0
 * @date    2026-03-20
 */

#include "modbus_rtu.h"
#include <string.h>

volatile uint16_t Modbus_HoldingRegisters[MODBUS_HOLDING_REG_COUNT];

/* ==============================================================================
 *                            内部 CRC16 查表定义
 * ============================================================================== */
static const uint8_t auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/* ==============================================================================
 *                               内部函数声明
 * ============================================================================== */
static void mb_send_response(Modbus_Slave_t *mb);
static void mb_send_exception(Modbus_Slave_t *mb, uint8_t func_code, uint8_t ex_code);
static void mb_handle_read_coils(Modbus_Slave_t *mb);
static void mb_handle_read_discrete_inputs(Modbus_Slave_t *mb);
static void mb_handle_read_holding_registers(Modbus_Slave_t *mb);
static void mb_handle_write_single_coil(Modbus_Slave_t *mb);
static void mb_handle_write_single_register(Modbus_Slave_t *mb);
static void mb_handle_write_multiple_registers(Modbus_Slave_t *mb);

static uint8_t mb_read_holding_register(Modbus_Slave_t *mb,
                                        uint16_t addr,
                                        uint16_t *value)
{
    if (value == NULL) return MB_EX_SLAVE_DEVICE_FAILURE;
    if (addr < MODBUS_HOLDING_REG_COUNT) {
        *value = Modbus_HoldingRegisters[addr];
        return MB_CB_OK;
    }
    if (mb->read_holding_register == NULL) return MB_EX_SLAVE_DEVICE_FAILURE;
    return mb->read_holding_register(addr, value);
}

static uint8_t mb_write_holding_register(Modbus_Slave_t *mb,
                                         uint16_t addr,
                                         uint16_t value)
{
    if (addr < MODBUS_HOLDING_REG_COUNT) {
        Modbus_HoldingRegisters[addr] = value;
        return MB_WRITE_OK;
    }
    if (mb->write_holding_register == NULL) return MB_EX_SLAVE_DEVICE_FAILURE;
    return mb->write_holding_register(addr, value);
}

/* ==============================================================================
 *                               公共接口实现
 * ============================================================================== */

uint16_t Modbus_CRC16(const uint8_t *buf, uint16_t len)
{
    /* 双表查找法避免逐位移位，适合在高频串口收帧路径中使用。 */
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint16_t uIndex;

    while (len--) {
        uIndex = uchCRCHi ^ *buf++;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uint16_t)((uchCRCLo << 8) | uchCRCHi);
}

void Modbus_Init(Modbus_Slave_t *mb, uint8_t slave_addr)
{
    if (mb == NULL) return;
    memset(mb, 0, sizeof(Modbus_Slave_t));
    mb->slave_addr = slave_addr;
}

void Modbus_ProcessFrame(Modbus_Slave_t *mb, uint8_t *data, uint16_t len)
{
    uint16_t calc_crc, recv_crc;
    uint8_t func_code;

    if (mb == NULL || data == NULL || len < 4 || len > MODBUS_RX_BUF_SIZE) return;

    /* 本实现不处理广播地址，非本机地址直接忽略且不发送响应。 */
    if (data[0] != mb->slave_addr) return;

    calc_crc = Modbus_CRC16(data, len - 2);
    recv_crc = (data[len - 1] << 8) | data[len - 2];

    if (calc_crc != recv_crc) return;

    memcpy(mb->rx_buf, data, len);
    mb->rx_len = len;
    
    func_code = mb->rx_buf[1];

    switch (func_code) {
        case MB_FUNC_READ_COILS:
            if (len != 8U) {
                mb_send_exception(mb, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            mb_handle_read_coils(mb);
            break;
        case MB_FUNC_READ_DISCRETE_INPUTS:
            if (len != 8U) {
                mb_send_exception(mb, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            mb_handle_read_discrete_inputs(mb);
            break;
        case MB_FUNC_READ_HOLDING_REGISTERS:
            if (len != 8U) {
                mb_send_exception(mb, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            mb_handle_read_holding_registers(mb);
            break;
        case MB_FUNC_WRITE_SINGLE_COIL:
            if (len != 8U) {
                mb_send_exception(mb, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            mb_handle_write_single_coil(mb);
            break;
        case MB_FUNC_WRITE_SINGLE_REGISTER:
            if (len != 8U) {
                mb_send_exception(mb, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            mb_handle_write_single_register(mb);
            break;
        case MB_FUNC_WRITE_MULTIPLE_REGISTERS:
            if (len < 9U || len != (uint16_t)(9U + mb->rx_buf[6])) {
                mb_send_exception(mb, func_code, MB_EX_ILLEGAL_DATA_VALUE);
                break;
            }
            mb_handle_write_multiple_registers(mb);
            break;
        default:
            mb_send_exception(mb, func_code, MB_EX_ILLEGAL_FUNCTION);
            break;
    }
}

/* ==============================================================================
 *                               内部功能实现
 * ============================================================================== */

static void mb_send_exception(Modbus_Slave_t *mb, uint8_t func_code, uint8_t ex_code)
{
    mb->tx_buf[0] = mb->slave_addr;
    mb->tx_buf[1] = func_code | 0x80;
    mb->tx_buf[2] = ex_code;
    mb->tx_len = 3;
    mb_send_response(mb);
}

static void mb_send_response(Modbus_Slave_t *mb)
{
    /* Modbus RTU 在线路上传输 CRC 时低字节在前。 */
    uint16_t crc = Modbus_CRC16(mb->tx_buf, mb->tx_len);
    mb->tx_buf[mb->tx_len++] = crc & 0xFF;
    mb->tx_buf[mb->tx_len++] = (crc >> 8) & 0xFF;
    
    if (mb->uart_send) {
        mb->uart_send(mb->tx_buf, mb->tx_len);
    }
}

static void mb_handle_read_coils(Modbus_Slave_t *mb)
{
    uint16_t start_addr = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t quantity   = (mb->rx_buf[4] << 8) | mb->rx_buf[5];
    uint8_t byte_count  = (quantity + 7) / 8;
    uint16_t i;
    uint8_t bit_pos;

    if (quantity < 1 || quantity > 2000) {
        mb_send_exception(mb, MB_FUNC_READ_COILS, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    if ((uint32_t)start_addr + quantity > 0x10000) {
        mb_send_exception(mb, MB_FUNC_READ_COILS, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    if (mb->read_coil == NULL) {
        mb_send_exception(mb, MB_FUNC_READ_COILS, MB_EX_SLAVE_DEVICE_FAILURE);
        return;
    }

    mb->tx_buf[0] = mb->slave_addr;
    mb->tx_buf[1] = MB_FUNC_READ_COILS;
    mb->tx_buf[2] = byte_count;
    
    memset(&mb->tx_buf[3], 0, byte_count);

    for (i = 0; i < quantity; i++) {
        uint8_t val = 0U;
        uint8_t result = mb->read_coil((uint16_t)(start_addr + i), &val);
        if (result != MB_CB_OK) {
            mb_send_exception(mb, MB_FUNC_READ_COILS, result);
            return;
        }
        if (val) {
            bit_pos = i % 8;
            mb->tx_buf[3 + (i / 8)] |= (1 << bit_pos);
        }
    }
    
    mb->tx_len = 3 + byte_count;
    mb_send_response(mb);
}

static void mb_handle_read_discrete_inputs(Modbus_Slave_t *mb)
{
    uint16_t start_addr = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t quantity   = (mb->rx_buf[4] << 8) | mb->rx_buf[5];
    uint8_t byte_count  = (quantity + 7) / 8;
    uint16_t i;
    uint8_t bit_pos;

    if (quantity < 1 || quantity > 2000) {
        mb_send_exception(mb, MB_FUNC_READ_DISCRETE_INPUTS, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    if ((uint32_t)start_addr + quantity > 0x10000) {
        mb_send_exception(mb, MB_FUNC_READ_DISCRETE_INPUTS, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    if (mb->read_discrete_input == NULL) {
        mb_send_exception(mb, MB_FUNC_READ_DISCRETE_INPUTS, MB_EX_SLAVE_DEVICE_FAILURE);
        return;
    }

    mb->tx_buf[0] = mb->slave_addr;
    mb->tx_buf[1] = MB_FUNC_READ_DISCRETE_INPUTS;
    mb->tx_buf[2] = byte_count;
    
    memset(&mb->tx_buf[3], 0, byte_count);

    for (i = 0; i < quantity; i++) {
        uint8_t val = 0U;
        uint8_t result = mb->read_discrete_input((uint16_t)(start_addr + i), &val);
        if (result != MB_CB_OK) {
            mb_send_exception(mb, MB_FUNC_READ_DISCRETE_INPUTS, result);
            return;
        }
        if (val) {
            bit_pos = i % 8;
            mb->tx_buf[3 + (i / 8)] |= (1 << bit_pos);
        }
    }
    
    mb->tx_len = 3 + byte_count;
    mb_send_response(mb);
}

static void mb_handle_read_holding_registers(Modbus_Slave_t *mb)
{
    uint16_t start_addr = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t quantity   = (mb->rx_buf[4] << 8) | mb->rx_buf[5];
    uint8_t byte_count  = quantity * 2;
    uint16_t i;

    if (quantity < 1 || quantity > 125) {
        mb_send_exception(mb, MB_FUNC_READ_HOLDING_REGISTERS, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    if ((uint32_t)start_addr + quantity > 0x10000) {
        mb_send_exception(mb, MB_FUNC_READ_HOLDING_REGISTERS, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    mb->tx_buf[0] = mb->slave_addr;
    mb->tx_buf[1] = MB_FUNC_READ_HOLDING_REGISTERS;
    mb->tx_buf[2] = byte_count;

    for (i = 0; i < quantity; i++) {
        uint16_t val = 0U;
        uint8_t result = mb_read_holding_register(
            mb, (uint16_t)(start_addr + i), &val);
        if (result != MB_CB_OK) {
            mb_send_exception(mb, MB_FUNC_READ_HOLDING_REGISTERS, result);
            return;
        }
        mb->tx_buf[3 + i * 2] = (val >> 8) & 0xFF;
        mb->tx_buf[4 + i * 2] = val & 0xFF;
    }

    mb->tx_len = 3 + byte_count;
    mb_send_response(mb);
}

static void mb_handle_write_single_coil(Modbus_Slave_t *mb)
{
    uint16_t addr  = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t value = (mb->rx_buf[4] << 8) | mb->rx_buf[5];
    uint8_t state  = 0;

    if (value == 0xFF00) {
        state = 1;
    } else if (value == 0x0000) {
        state = 0;
    } else {
        mb_send_exception(mb, MB_FUNC_WRITE_SINGLE_COIL, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    if (mb->write_coil == NULL) {
        mb_send_exception(mb, MB_FUNC_WRITE_SINGLE_COIL, MB_EX_SLAVE_DEVICE_FAILURE);
        return;
    }

    uint8_t result = mb->write_coil(addr, state);
    if (result != MB_WRITE_OK) {
        mb_send_exception(mb, MB_FUNC_WRITE_SINGLE_COIL, result);
        return;
    }

    memcpy(mb->tx_buf, mb->rx_buf, 6);
    mb->tx_len = 6;
    mb_send_response(mb);
}

static void mb_handle_write_single_register(Modbus_Slave_t *mb)
{
    uint16_t addr  = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t value = (mb->rx_buf[4] << 8) | mb->rx_buf[5];

    uint8_t result = mb_write_holding_register(mb, addr, value);
    if (result != MB_WRITE_OK) {
        mb_send_exception(mb, MB_FUNC_WRITE_SINGLE_REGISTER, result);
        return;
    }

    memcpy(mb->tx_buf, mb->rx_buf, 6);
    mb->tx_len = 6;
    mb_send_response(mb);
}

static void mb_handle_write_multiple_registers(Modbus_Slave_t *mb)
{
    uint16_t start_addr = (mb->rx_buf[2] << 8) | mb->rx_buf[3];
    uint16_t quantity   = (mb->rx_buf[4] << 8) | mb->rx_buf[5];
    uint8_t byte_count  = mb->rx_buf[6];
    uint16_t holding_backup[MODBUS_HOLDING_REG_COUNT];
    uint16_t holding_backup_start = 0U;
    uint16_t holding_backup_end = 0U;
    bool uses_application_registers;
    uint16_t i;

    if (quantity < 1 || quantity > 123 || byte_count != (quantity * 2)) {
        mb_send_exception(mb, MB_FUNC_WRITE_MULTIPLE_REGISTERS, MB_EX_ILLEGAL_DATA_VALUE);
        return;
    }

    if ((uint32_t)start_addr + quantity > 0x10000) {
        mb_send_exception(mb, MB_FUNC_WRITE_MULTIPLE_REGISTERS, MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    uses_application_registers =
        ((uint32_t)start_addr + quantity > MODBUS_HOLDING_REG_COUNT);

    if (start_addr < MODBUS_HOLDING_REG_COUNT) {
        uint32_t request_end = (uint32_t)start_addr + quantity;
        holding_backup_start = start_addr;
        holding_backup_end = (request_end < MODBUS_HOLDING_REG_COUNT)
                           ? (uint16_t)request_end
                           : MODBUS_HOLDING_REG_COUNT;
        for (i = holding_backup_start; i < holding_backup_end; i++) {
            holding_backup[i] = Modbus_HoldingRegisters[i];
        }
    }

    /* 应用层可借助事务钩子先写影子区，避免 FC16 只更新一部分寄存器。 */
    if (uses_application_registers &&
        mb->begin_write_holding_registers != NULL &&
        !mb->begin_write_holding_registers(start_addr, quantity)) {
        mb_send_exception(mb, MB_FUNC_WRITE_MULTIPLE_REGISTERS, MB_EX_SLAVE_DEVICE_FAILURE);
        return;
    }

    for (i = 0; i < quantity; i++) {
        uint16_t val = (mb->rx_buf[7 + i * 2] << 8) | mb->rx_buf[8 + i * 2];
        uint8_t result = mb_write_holding_register(
            mb, (uint16_t)(start_addr + i), val);
        if (result != MB_WRITE_OK) {
            for (uint16_t reg = holding_backup_start;
                 reg < holding_backup_end; reg++) {
                Modbus_HoldingRegisters[reg] = holding_backup[reg];
            }
            if (uses_application_registers &&
                mb->end_write_holding_registers != NULL) {
                mb->end_write_holding_registers(false);
            }
            mb_send_exception(mb, MB_FUNC_WRITE_MULTIPLE_REGISTERS, result);
            return;
        }
    }

    if (uses_application_registers &&
        mb->end_write_holding_registers != NULL) {
        mb->end_write_holding_registers(true);
    }

    memcpy(mb->tx_buf, mb->rx_buf, 6);
    mb->tx_len = 6;
    mb_send_response(mb);
}
