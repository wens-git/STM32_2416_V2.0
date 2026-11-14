/**
 * @file    IODEF.c
 * @brief   硬件 I/O 抽象层源文件 — GPIO 初始化、输入/输出操作、消抖扫描
 * @details 本文件实现了工控板全部数字 I/O 的底层操作:
 *          - IO_Init(): 使能 GPIO 时钟、配置引脚模式、绑定消抖实例
 *          - Y0()~Y17(): 输出端口控制 (同步更新 Y_State 镜像数组)
 *          - X0()~X27(): 输入端口物理电平读取 (供消抖模块回调使用)
 *          - IO_Scan():  驱动 24 路消抖状态机 + 刷新 X_State 数组
 *
 *          所有 GPIO 操作均通过 HAL 库函数完成，确保跨型号可移植性。
 */

#include "IODEF.h"
#include "app_config.h"
#if APP_DEBUG_SERVICE_ENABLE
#include "app/debug/motor_debug_service.h"
#endif

/* 数字输入统一封装
 * 2416 输入光耦输出端均为上拉到 MCU，触发时光耦导通拉低；统一取反归一为逻辑高=触发。
 */
static uint8_t io_read_di_normalized(GPIO_TypeDef *port, uint16_t pin, uint8_t mcu_pin_active_low)
{
    uint8_t mcu_high = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1u : 0u;
    if (mcu_pin_active_low)
        mcu_high ^= 1u;
    return mcu_high;
}

/* ==========================================================================
 * 全局状态镜像数组
 * ========================================================================== */

/** @brief 16 路输出状态镜像 (Y_State[0]~[15] 对应 Y0~Y17)，由 Yn() 函数同步更新 */
uint8_t Y_State[16] = {0};

/** @brief 24 路输入状态镜像 (X_State[0]~[23] 对应 X0~X27)，由 IO_Scan() 刷新 */
uint8_t X_State[24] = {0};

/* ==========================================================================
 * 24 路消抖输入实例实体定义 (与 IODEF.h 中的 extern 声明对应)
 * ========================================================================== */

/* ---- 第 0 组 (X0 ~ X7) ---- */
Input_t In_X0; /* X0  — PA0  */
Input_t In_X1; /* X1  — PA1  */
Input_t In_X2; /* X2  — PB15 */
Input_t In_X3; /* X3  — PA15 */
Input_t In_X4; /* X4  — PB3  */
Input_t In_X5; /* X5  — PC4  */
Input_t In_X6; /* X6  — PD6  */
Input_t In_X7; /* X7  — PD5  */

/* ---- 第 1 组 (X10 ~ X17) ---- */
Input_t In_X10; /* X10 — PD10 */
Input_t In_X11; /* X11 — PC8  */
Input_t In_X12; /* X12 — PC9  */
Input_t In_X13; /* X13 — PA12 */
Input_t In_X14; /* X14 — PA11 */
Input_t In_X15; /* X15 — PD15 */
Input_t In_X16; /* X16 — PD14 */
Input_t In_X17; /* X17 — PD13 */

/* ---- 第 2 组 (X20 ~ X27) ---- */
Input_t In_X20; /* X20 — PD12 */
Input_t In_X21; /* X21 — PD11 */
Input_t In_X22; /* X22 — PE15 */
Input_t In_X23; /* X23 — PE14 */
Input_t In_X24; /* X24 — PE13 */
Input_t In_X25; /* X25 — PE12 */
Input_t In_X26; /* X26 — PE11 */
Input_t In_X27; /* X27 — PE10 */

/* ==========================================================================
 * IO_Init() — GPIO 初始化 + 消抖实例绑定
 * ========================================================================== */

/**
 * @brief  初始化全部 I/O 端口
 * @details 执行以下步骤:
 *          1) 使能所有用到的 GPIO 端口时钟 (A/B/C/D/E)
 *          2) 将所有输出引脚配置为推挽输出，默认低电平 (MCU 空闲态)
 *          3) 将所有输入引脚配置为上拉输入
 *          4) 为 24 个输入通道创建消抖实例，绑定读取回调，设置消抖阈值
 */
void IO_Init(void) {
 GPIO_InitTypeDef GPIO_InitStruct = {0};

 /* ---- 步骤 1: 使能 GPIO 端口时钟 ---- */
 __HAL_RCC_GPIOA_CLK_ENABLE(); /* 端口 A: 包含 Y0/Y3, X0/X1/X3/X13/X14 等 */
 __HAL_RCC_GPIOB_CLK_ENABLE(); /* 端口 B: 包含 Y4~Y17, LED, X2/X4 等 */
 __HAL_RCC_GPIOC_CLK_ENABLE(); /* 端口 C: 包含 Y1/Y2/Y10, X5/X11/X12 */
 __HAL_RCC_GPIOD_CLK_ENABLE(); /* 端口 D: 包含 Y6/Y7, X6~X27 部分 */
 __HAL_RCC_GPIOE_CLK_ENABLE(); /* 端口 E: 包含 Y11~Y13, X22~X27 */

 /* ---- 步骤 2: 输出引脚初始化 ---- */
 /* 所有输出引脚默认拉低 (MCU 空闲态) */
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_RESET); /* Y0(PA6), Y3(PA8) */
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
 /* Y17~Y14, Y4, Y5 */
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, GPIO_PIN_RESET); /* LED_RUN(PB10), LED_ERR(PB11) */
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_13, GPIO_PIN_RESET); /* Y1(PC6), Y2(PC7), Y10(PC13) */
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 | GPIO_PIN_7, GPIO_PIN_RESET); /* Y7(PD4), Y6(PD7) */
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET); /* Y13(PE0), Y12(PE1), Y11(PE2) */

 /* 配置 PA6, PA8 为推挽输出 (Y0, Y3) */
 GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_8;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; /* 推挽输出模式 */
 GPIO_InitStruct.Pull = GPIO_PULLUP; /* 内部上拉 (增强驱动能力) */
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; /* 最高速率 (100MHz, 减少翻转延迟) */
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 /* 配置 PB4~PB11 为推挽输出 (Y17~Y14, Y4, Y5, LED_RUN, LED_ERR) */
 GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                       GPIO_PIN_11;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 /* 配置 PC6, PC7, PC13 为推挽输出 (Y1, Y2, Y10) */
 GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_13;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

 /* 配置 PD4, PD7 为推挽输出 (Y7, Y6) */
 GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_7;
 HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

 /* 配置 PE0, PE1, PE2 为推挽输出 (Y13, Y12, Y11) */
 GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
 HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

 /* ---- 步骤 3: 输入引脚初始化 ---- */
 /* 所有输入引脚配置为上拉输入模式 */

 /* PA0(X0), PA1(X1), PA11(X14), PA12(X13), PA15(X3) */
 GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT; /* 输入模式 */
 GPIO_InitStruct.Pull = GPIO_PULLUP; /* 内部上拉 (无外部信号时默认高电平) */
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

 /* PB3(X4), PB15(X2) */
 GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_15;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

 /* PC4(X5), PC8(X11), PC9(X12) */
 GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

 /* PD5(X7), PD6(X6), PD10(X10), PD11(X21), PD12(X20), PD13(X17), PD14(X16), PD15(X15) */
 GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                       GPIO_PIN_15;
 HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

 /* PE10(X27), PE11(X26), PE12(X25), PE13(X24), PE14(X23), PE15(X22) */
 GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
 HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

 /* ---- 步骤 4: 消抖模块初始化 ---- */
 /* 将 24 个 Input_t 实例与底层引脚读取函数绑定 */
 /* 消抖阈值 = 10 次连续一致采样。以 1ms 扫描周期计算，消抖时间 = 10ms */
 /* 该值适用于大多数工业级机械按键和光耦隔离输入信号 */

 /* 第 0 组 */
 input_init(&In_X0, X0, 10); /* X0  ← PA0,  消抖 10ms */
 input_init(&In_X1, X1, 10); /* X1  ← PA1,  消抖 10ms */
 input_init(&In_X2, X2, 10); /* X2  ← PB15, 消抖 10ms */
 input_init(&In_X3, X3, 10); /* X3  ← PA15, 消抖 10ms */
 input_init(&In_X4, X4, 10); /* X4  ← PB3,  消抖 10ms */
 input_init(&In_X5, X5, 10); /* X5  ← PC4,  消抖 10ms */
 input_init(&In_X6, X6, 10); /* X6  ← PD6,  消抖 10ms */
 input_init(&In_X7, X7, 10); /* X7  ← PD5,  消抖 10ms */

 /* 第 1 组 */
 input_init(&In_X10, X10, 10); /* X10 ← PD10, 消抖 10ms */
 input_init(&In_X11, X11, 10); /* X11 ← PC8,  消抖 10ms */
 input_init(&In_X12, X12, 10); /* X12 ← PC9,  消抖 10ms */
 input_init(&In_X13, X13, 10); /* X13 ← PA12, 消抖 10ms */
 input_init(&In_X14, X14, 10); /* X14 ← PA11, 消抖 10ms */
 input_init(&In_X15, X15, 10); /* X15 ← PD15, 消抖 10ms */
 input_init(&In_X16, X16, 10); /* X16 ← PD14, 消抖 10ms */
 input_init(&In_X17, X17, 10); /* X17 ← PD13, 消抖 10ms */

 /* 第 2 组 */
 input_init(&In_X20, X20, 10); /* X20 ← PD12, 消抖 10ms */
 input_init(&In_X21, X21, 10); /* X21 ← PD11, 消抖 10ms */
 input_init(&In_X22, X22, 10); /* X22 ← PE15, 消抖 10ms */
 input_init(&In_X23, X23, 10); /* X23 ← PE14, 消抖 10ms */
 input_init(&In_X24, X24, 10); /* X24 ← PE13, 消抖 10ms */
 input_init(&In_X25, X25, 10); /* X25 ← PE12, 消抖 10ms */
 input_init(&In_X26, X26, 10); /* X26 ← PE11, 消抖 10ms */
 input_init(&In_X27, X27, 10); /* X27 ← PE10, 消抖 10ms */
}

/* ==========================================================================
 * 输出端口控制 (Y0 ~ Y17)
 * ==========================================================================
 * 每个函数完成两件事:
 * 1) 通过 HAL_GPIO_WritePin() 设置物理 GPIO 电平
 * 2) 将当前状态同步到 Y_State[] 数组 (供 Modbus 等协议读回输出状态)
 *
 * 参数 state: 1 = 输出高电平 (GPIO_PIN_SET)
 *             0 = 输出低电平 (GPIO_PIN_RESET)
 */

/* ---- 第 0 组输出 ---- */
void Y0(uint8_t state) {
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[0] = state;
} /* Y0  → PA6  */
void Y1(uint8_t state) {
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[1] = state;
} /* Y1 APul  → PC6  */
void Y2(uint8_t state) {
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[2] = state;
} /* Y2 BPul  → PC7  */
void Y3(uint8_t state) {
 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[3] = state;
} /* Y3 TransPul → PA8  */
void Y4(uint8_t state) {
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[4] = state;
} /* Y4 ADir → PB7  */
void Y5(uint8_t state) {
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[5] = state;
} /* Y5 BDir → PB8  */
void Y6(uint8_t state) {
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[6] = state;
} /* Y6 TransDir → PD7  */
void Y7(uint8_t state) {
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[7] = state;
} /* Y7  → PD4  */

/* ---- 第 1 组输出 ---- */
void Y10(uint8_t state) {
 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[8] = state;
} /* Y10 A站空料报警 → PC13 */
void Y11(uint8_t state) {
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[9] = state;
} /* Y11 A站上位机输出报警 → PE2  */
void Y12(uint8_t state) {
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[10] = state;
} /* Y12 B站料满报警 → PE1  */
void Y13(uint8_t state) {
 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[11] = state;
} /* Y13 → PE0  */
void Y14(uint8_t state) {
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[12] = state;
} /* Y14 → PB9  */
void Y15(uint8_t state) {
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[13] = state;
} /* Y15 → PB6  */
void Y16(uint8_t state) {
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[14] = state;
} /* Y16 → PB5  */
void Y17(uint8_t state) {
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
 Y_State[15] = state;
} /* Y17 → PB4  */

/* ==========================================================================
 * LED 指示灯控制
 * ========================================================================== */
void LED_RUN(uint8_t state) {
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
} /* 运行灯 → PB10 */
void LED_ERR(uint8_t state) {
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
} /* 故障灯 → PB11 */

/* ==========================================================================
 * 输入端口归一化读取 (X0 ~ X27)
 * ==========================================================================
 * 返回值: 1 = 逻辑高 = 已触发(外部端子低有效已动作), 0 = 未触发。
 * 经 io_read_di_normalized，供消抖回调与上层一致语义；瞬时读值未经消抖。
 */

/* ---- 第 0 组输入 ---- */
uint8_t X0(void) { return io_read_di_normalized(GPIOA, GPIO_PIN_0, 1); } /* X0  ← PA0  */
uint8_t X1(void) { return io_read_di_normalized(GPIOA, GPIO_PIN_1, 1); } /* X1  ← PA1  */
uint8_t X2(void) { return io_read_di_normalized(GPIOB, GPIO_PIN_15, 1); } /* X2  ← PB15 */
uint8_t X3(void) { return io_read_di_normalized(GPIOA, GPIO_PIN_15, 1); } /* X3  ← PA15 */
uint8_t X4(void) { return io_read_di_normalized(GPIOB, GPIO_PIN_3, 1); } /* X4  ← PB3  */
uint8_t X5(void) { return io_read_di_normalized(GPIOC, GPIO_PIN_4, 1); } /* X5  ← PC4  */
uint8_t X6(void) { return io_read_di_normalized(GPIOD, GPIO_PIN_6, 1); } /* X6  ← PD6  */
uint8_t X7(void) { return io_read_di_normalized(GPIOD, GPIO_PIN_5, 1); } /* X7  ← PD5  */

/* ---- 第 1 组输入 ---- */
uint8_t X10(void) { return io_read_di_normalized(GPIOD, GPIO_PIN_10, 1); } /* X10 ← PD10 */
uint8_t X11(void) { return io_read_di_normalized(GPIOC, GPIO_PIN_8, 1); } /* X11 ← PC8  */
uint8_t X12(void) { return io_read_di_normalized(GPIOC, GPIO_PIN_9, 1); } /* X12 ← PC9  */
uint8_t X13(void) { return io_read_di_normalized(GPIOA, GPIO_PIN_12, 1); } /* X13 ← PA12 */
uint8_t X14(void) { return io_read_di_normalized(GPIOA, GPIO_PIN_11, 1); } /* X14 ← PA11 */
uint8_t X15(void) { return io_read_di_normalized(GPIOD, GPIO_PIN_15, 1); } /* X15 ← PD15 */
uint8_t X16(void) { return io_read_di_normalized(GPIOD, GPIO_PIN_14, 1); } /* X16 ← PD14 */
uint8_t X17(void) { return io_read_di_normalized(GPIOD, GPIO_PIN_13, 1); } /* X17 ← PD13 */

/* ---- 第 2 组输入 ---- */
uint8_t X20(void) { return io_read_di_normalized(GPIOD, GPIO_PIN_12, 1); } /* X20 ← PD12 */
uint8_t X21(void) { return io_read_di_normalized(GPIOD, GPIO_PIN_11, 1); } /* X21 ← PD11 */
uint8_t X22(void) { return io_read_di_normalized(GPIOE, GPIO_PIN_15, 1); } /* X22 ← PE15 */
uint8_t X23(void) { return io_read_di_normalized(GPIOE, GPIO_PIN_14, 1); } /* X23 ← PE14 */
uint8_t X24(void) { return io_read_di_normalized(GPIOE, GPIO_PIN_13, 1); } /* X24 ← PE13 */
uint8_t X25(void) { return io_read_di_normalized(GPIOE, GPIO_PIN_12, 1); } /* X25 ← PE12 */
uint8_t X26(void) { return io_read_di_normalized(GPIOE, GPIO_PIN_11, 1); } /* X26 ← PE11 */
uint8_t X27(void) { return io_read_di_normalized(GPIOE, GPIO_PIN_10, 1); } /* X27 ← PE10 */

/* ==========================================================================
 * IO_Scan() — 全局输入消抖扫描 + X_State 数组刷新
 * ==========================================================================
 * 由 Tick_Hook() 在每次 SysTick 中断 (1ms) 中调用。
 * 分两步执行:
 * 1) 驱动 24 路消抖状态机更新 (调用 input_update)
 * 2) 将消抖后的稳定状态写入 X_State[] 数组 (供旧代码兼容和 Modbus 查询)
 */
void IO_Scan(void) {
 /* ---- 第一步: 驱动 24 路输入消抖状态机更新 ---- */
 /* 每调用一次 input_update()，对应通道的消抖计数器推进一步 */

 /* 第 0 组 */
 input_update(&In_X0); /* 更新 X0  消抖状态 */
 input_update(&In_X1); /* 更新 X1  消抖状态 */
 input_update(&In_X2); /* 更新 X2  消抖状态 */
 input_update(&In_X3); /* 更新 X3  消抖状态 */
 input_update(&In_X4); /* 更新 X4  消抖状态 */
 input_update(&In_X5); /* 更新 X5  消抖状态 */
 input_update(&In_X6); /* 更新 X6  消抖状态 */
 input_update(&In_X7); /* 更新 X7  消抖状态 */

 /* 第 1 组 */
 input_update(&In_X10); /* 更新 X10 消抖状态 */
 input_update(&In_X11); /* 更新 X11 消抖状态 */
 input_update(&In_X12); /* 更新 X12 消抖状态 */
 input_update(&In_X13); /* 更新 X13 消抖状态 */
 input_update(&In_X14); /* 更新 X14 消抖状态 */
 input_update(&In_X15); /* 更新 X15 消抖状态 */
 input_update(&In_X16); /* 更新 X16 消抖状态 */
 input_update(&In_X17); /* 更新 X17 消抖状态 */

 /* 第 2 组 */
 input_update(&In_X20); /* 更新 X20 消抖状态 */
 input_update(&In_X21); /* 更新 X21 消抖状态 */
 input_update(&In_X22); /* 更新 X22 消抖状态 */
 input_update(&In_X23); /* 更新 X23 消抖状态 */
 input_update(&In_X24); /* 更新 X24 消抖状态 */
 input_update(&In_X25); /* 更新 X25 消抖状态 */
 input_update(&In_X26); /* 更新 X26 消抖状态 */
 input_update(&In_X27); /* 更新 X27 消抖状态 */

#if APP_DEBUG_SERVICE_ENABLE
 /* X0 按下沿作为模式切换键，保持和旧版工程一致。 */
 if (input_check_down(&In_X0)) {
  Debug_Service_ToggleMode();
 }
#endif

 /* ---- 第二步: 将消抖结果同步到 X_State[] 数组 ---- */
 /* input_get() 返回 true = 触发(高电平有效)，此处转换为 1 存入数组 */
 /* 该数组主要供旧代码兼容和 Modbus 离散输入读取 (FC02) 使用 */

 /* 第 0 组 → X_State[0] ~ X_State[7] */
 X_State[0] = input_get(&In_X0) ? 1 : 0; /* X0  */
 X_State[1] = input_get(&In_X1) ? 1 : 0; /* X1  */
 X_State[2] = input_get(&In_X2) ? 1 : 0; /* X2  */
 X_State[3] = input_get(&In_X3) ? 1 : 0; /* X3  */
 X_State[4] = input_get(&In_X4) ? 1 : 0; /* X4  */
 X_State[5] = input_get(&In_X5) ? 1 : 0; /* X5  */
 X_State[6] = input_get(&In_X6) ? 1 : 0; /* X6  */
 X_State[7] = input_get(&In_X7) ? 1 : 0; /* X7  */

 /* 第 1 组 → X_State[8] ~ X_State[15] */
 X_State[8] = input_get(&In_X10) ? 1 : 0; /* X10 */
 X_State[9] = input_get(&In_X11) ? 1 : 0; /* X11 */
 X_State[10] = input_get(&In_X12) ? 1 : 0; /* X12 */
 X_State[11] = input_get(&In_X13) ? 1 : 0; /* X13 */
 X_State[12] = input_get(&In_X14) ? 1 : 0; /* X14 */
 X_State[13] = input_get(&In_X15) ? 1 : 0; /* X15 */
 X_State[14] = input_get(&In_X16) ? 1 : 0; /* X16 */
 X_State[15] = input_get(&In_X17) ? 1 : 0; /* X17 */

 /* 第 2 组 → X_State[16] ~ X_State[23] */
 X_State[16] = input_get(&In_X20) ? 1 : 0; /* X20 */
 X_State[17] = input_get(&In_X21) ? 1 : 0; /* X21 */
 X_State[18] = input_get(&In_X22) ? 1 : 0; /* X22 */
 X_State[19] = input_get(&In_X23) ? 1 : 0; /* X23 */
 X_State[20] = input_get(&In_X24) ? 1 : 0; /* X24 */
 X_State[21] = input_get(&In_X25) ? 1 : 0; /* X25 */
 X_State[22] = input_get(&In_X26) ? 1 : 0; /* X26 */
 X_State[23] = input_get(&In_X27) ? 1 : 0; /* X27 */
}
