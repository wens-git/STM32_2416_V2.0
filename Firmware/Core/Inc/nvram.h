/**
 * @file    nvram.h
 * @brief   NVRAM 非易失存储模块 — 使用 STM32F407 内部 Flash 存储配置数据
 * @details 本模块利用 STM32F407 内部 Flash 的 Sector 7 (0x08060000)
 *          作为非易失存储区域，用于保存上位机写入的轴定位点数据。
 *
 *          【存储策略】
 *          - 使用 Flash Sector 7 作为 NVRAM 区域，与程序区隔离
 *          - 数据结构: magic + 数据区 + CRC32 校验
 *          - 写入时擦除整个 sector 后按 32-bit word 编程
 *          - 读取时直接从 Flash 地址指针读取，校验 CRC
 *
 *          【使用方式】
 *          启动时调用 NVRAM_Init() 读取 Flash 并加载到 RAM 镜像。
 *          修改后调用 NVRAM_Save() 写入 Flash。
 *          定位点通过 NVRAM_GetPositionPoint / NVRAM_SetPositionPoint 读写。
 *          点位速度通过 NVRAM_GetPositionPointSpeed / NVRAM_SetPositionPointSpeed 读写。
 *
 *          【数据布局 (Sector 7, 共 2056 字节)】
 *          Offset 0:    magic        (4 bytes)  = 0x38545841 ("AXT8")
 *          Offset 4:    points[8][8]            = 8轴 x 8点 x int32_t 坐标
 *          Offset 260:  point_names[8][8][12]   = UTF-16 码元 (点位名称)
 *          Offset 1796: point_speeds[8][8]      = 8轴 x 8点 x uint32_t 速度 (Hz)
 *          Offset 2052: crc32        (4 bytes)
 *
 *          【旧格式迁移】
 *          早期 "AXT8 无速度区" 数据 (crc32 位于 Offset 1796) 在首次启动时自动迁移:
 *          保留坐标与名称, 速度统一填充 NVRAM_POINT_SPEED_DEFAULT, 随后立即回写。
 */

#ifndef __NVRAM_H__
#define __NVRAM_H__

#include <stdint.h>
#include <stdbool.h>

#define NVRAM_POINTS_PER_AXIS  8    /**< 每轴 8 个定位点 */
#define NVRAM_MAX_AXES         8    /**< 最多 8 轴 */
#define NVRAM_POINT_NAME_LEN   12   /**< 每个点位名称固定 12 个 UTF-16 码元 */

/** 点位速度默认值 (Hz)。旧格式迁移与非法值回退时使用。 */
#define NVRAM_POINT_SPEED_DEFAULT  1250U

/*
 * The point database above is the legacy host-tool contract and remains in
 * Sector 7.  The APIs below are a separate generic persistence store in
 * Sectors 5 and 6.  They must not be mixed with NVRAM_Save().
 */
typedef struct {
    const void *address;
    uint32_t size;
    const char *name;
} NVRAM_PersistDescriptor_t;

#if defined(__GNUC__)
#define NVRAM_PERSISTENT_REGISTER(variable) \
    static const NVRAM_PersistDescriptor_t nvram_persist_desc_##variable \
        __attribute__((used, section(".persist_desc"), aligned(4))) = { \
            &(variable), sizeof(variable), #variable }
#else
#define NVRAM_PERSISTENT_REGISTER(variable) \
    static const NVRAM_PersistDescriptor_t nvram_persist_desc_##variable = { \
        &(variable), sizeof(variable), #variable }
#endif

/* Use this form for scalar variables. Use NVRAM_PERSISTENT_REGISTER() after
 * an existing array/struct declaration when its initializer is complex. */
#define NVRAM_PERSISTENT(type, variable, initializer) \
    static type variable = initializer; \
    NVRAM_PERSISTENT_REGISTER(variable)

/**
 * @brief  初始化 NVRAM 模块 — 从 Flash 加载数据到 RAM
 * @note   启动时调用一次。若 Flash 数据无效 (CRC 错/无 magic)，自动初始化为零。
 */
void NVRAM_Init(void);

/**
 * @brief  将当前 RAM 数据保存到 Flash
 * @note   会擦除并重写整个 NVRAM sector，耗时约~100ms。
 *         调用过程中会短暂关闭中断。
 * @return true=写入并回读校验成功，false=写入失败或校验失败
 */
bool NVRAM_Save(void);

/**
 * @brief  读取指定轴指定定位点的绝对位置
 * @param  axis  轴索引 (0~7)
 * @param  index 定位点索引 (0~7)
 * @return 绝对位置 (脉冲数)，无效参数返回 0
 */
int32_t NVRAM_GetPositionPoint(uint8_t axis, uint8_t index);

/**
 * @brief  写入指定轴指定定位点的绝对位置 (仅修改 RAM，不立即写入 Flash)
 * @param  axis  轴索引 (0~7)
 * @param  index 定位点索引 (0~7)
 * @param  pos   绝对位置 (脉冲数)
 * @note   修改后需显式调用 NVRAM_Save() 写入 Flash。
 */
void NVRAM_SetPositionPoint(uint8_t axis, uint8_t index, int32_t pos);

/**
 * @brief  读取指定轴指定定位点名称中的一个 UTF-16 码元
 * @param  axis       轴索引 (0~7)
 * @param  index      定位点索引 (0~7)
 * @param  char_index 字符索引 (0~11)
 * @return UTF-16 码元；无效参数返回 0
 */
uint16_t NVRAM_GetPositionPointNameUnit(uint8_t axis, uint8_t index, uint8_t char_index);

/**
 * @brief  写入指定轴指定定位点名称中的一个 UTF-16 码元 (仅修改 RAM)
 * @param  axis       轴索引 (0~7)
 * @param  index      定位点索引 (0~7)
 * @param  char_index 字符索引 (0~11)
 * @param  value      UTF-16 码元；写入 0 表示字符串终止
 * @note   修改后需显式调用 NVRAM_Save() 写入 Flash。
 */
void NVRAM_SetPositionPointNameUnit(uint8_t axis, uint8_t index, uint8_t char_index, uint16_t value);

/**
 * @brief  读取指定轴指定定位点的运行速度
 * @param  axis  轴索引 (0~7)
 * @param  index 定位点索引 (0~7)
 * @return 速度 (Hz)；无效参数返回 NVRAM_POINT_SPEED_DEFAULT
 */
uint32_t NVRAM_GetPositionPointSpeed(uint8_t axis, uint8_t index);

/**
 * @brief  写入指定轴指定定位点的运行速度 (仅修改 RAM，不立即写入 Flash)
 * @param  axis  轴索引 (0~7)
 * @param  index 定位点索引 (0~7)
 * @param  speed 速度 (Hz)，调用者负责限制在 [PWM_FREQ_MIN, PWM_FREQ_MAX]
 * @note   修改后需显式调用 NVRAM_Save() 写入 Flash。
 */
void NVRAM_SetPositionPointSpeed(uint8_t axis, uint8_t index, uint32_t speed);

/** Initialize the separate generic variable persistence store. */
void NVRAM_Auto_Init(void);

/** Account for one millisecond. Must only be called from the 1ms tick hook. */
void NVRAM_Auto_Tick1ms(void);

/** Detect changed registered variables and perform delayed Flash writes. */
void NVRAM_Auto_Poll(void);

/** Explicitly restart the autosave debounce timer after a programmatic update. */
void NVRAM_Auto_MarkDirty(void);

/** Return true when the generic store has observed unsaved changes. */
bool NVRAM_Auto_IsDirty(void);

/** Return false to defer an autosave while motion or another critical action runs. */
typedef bool (*NVRAM_Auto_SaveGuard_t)(void);
void NVRAM_Auto_SetSaveGuard(NVRAM_Auto_SaveGuard_t guard);

/** Explicitly save only the generic variable store; does not touch host points. */
bool NVRAM_Auto_Save(void);

#endif /* __NVRAM_H__ */
