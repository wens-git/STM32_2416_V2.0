/**
 * @file    nvram.c
 * @brief   NVRAM 非易失存储实现 — 基于 STM32F407 内部 Flash
 * @details
 *          【Flash 扇区使用】
 *          当前工程统一使用 Sector 7 (0x08060000) 作为 NVRAM 存储区域。
 *          这样可兼容 512KB 器件的有效地址范围，并与当前实现常量保持一致。
 *
 *          【数据布局】
 *          Offset 0:    magic        (4 bytes)   = 0x38545841 ("AXT8")
 *          Offset 4:    points[8][8]            (256 bytes)  = 8轴 x 8点 x int32_t
 *          Offset 260:  point_names[8][8][12]   (1536 bytes) = UTF-16 码元
 *          Offset 1796: point_speeds[8][8]      (256 bytes)  = 8轴 x 8点 x uint32_t (Hz)
 *          Offset 2052: crc32                   (4 bytes)    = 前 2052 字节的 CRC32
 *          Total: 2056 bytes
 *
 *          【版本迁移】
 *          NVRAM_MAGIC "AXT8" 自早期版本起未变, 早期镜像没有速度区, crc32 位于
 *          Offset 1796。若新格式校验失败但旧 "AXT8" 布局 CRC 正确, 则保留坐标与
 *          名称、速度为 NVRAM_POINT_SPEED_DEFAULT, 并立即回写为当前格式。
 *
 *          【安全措施】
 *          - NVRAM_Save() 在写入前关闭全局中断
 *          - 写入完成后校验 CRC，失败则重试一次
 */

#include "nvram.h"
#include "app_config.h"
#include "bsp_iwdg.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

/* ==========================================================================
 * Flash 扇区选择
 * ========================================================================== */
/* STM32F407VET6 (512KB):
 * Sector 0~3: 16KB, Sector 4: 64KB, Sector 5~7: 128KB.
 * Sector 7 spans 0x08060000~0x0807FFFF and is reserved by the linker script.
 */
#define NVRAM_FLASH_SECTOR    FLASH_SECTOR_7
#define NVRAM_FLASH_ADDR      0x08060000U

/* ==========================================================================
 * 数据布局常量
 * ========================================================================== */
#define NVRAM_MAGIC             0x38545841U  /* "AXT8" */
#define NVRAM_LEGACY_MAGIC      0x50545841U  /* 旧版四轴数据格式标识。 */
#define NVRAM_LEGACY_AXES       4U           /* 旧版数据格式固定轴数。 */

/* ==========================================================================
 * RAM 镜像 — 上电加载，修改后写回
 * ========================================================================== */
typedef struct {
    uint32_t magic;
    int32_t  points[NVRAM_MAX_AXES][NVRAM_POINTS_PER_AXIS]; /* 8×8×4 = 256 bytes */
    uint16_t point_names[NVRAM_MAX_AXES][NVRAM_POINTS_PER_AXIS][NVRAM_POINT_NAME_LEN];
    uint32_t point_speeds[NVRAM_MAX_AXES][NVRAM_POINTS_PER_AXIS]; /* 8×8×4 = 256 bytes */
    uint32_t crc32;
} NVRAM_Data_t;

/* "AXT8" 早期布局: 有坐标与名称, 没有速度区 (crc32 位于 Offset 1796)。
 * 仅用于上电迁移识别, 不得用于写入。 */
typedef struct {
    uint32_t magic;
    int32_t  points[NVRAM_MAX_AXES][NVRAM_POINTS_PER_AXIS];
    uint16_t point_names[NVRAM_MAX_AXES][NVRAM_POINTS_PER_AXIS][NVRAM_POINT_NAME_LEN];
    uint32_t crc32;
} NVRAM_DataV1_t;

typedef struct {
    uint32_t magic;
    int32_t  points[NVRAM_LEGACY_AXES][NVRAM_POINTS_PER_AXIS];
    uint16_t point_names[NVRAM_LEGACY_AXES][NVRAM_POINTS_PER_AXIS][NVRAM_POINT_NAME_LEN];
    uint32_t crc32;
} NVRAM_LegacyNamedData_t;

typedef struct {
    uint32_t magic;
    int32_t  points[NVRAM_LEGACY_AXES][NVRAM_POINTS_PER_AXIS];
    uint8_t  reserved[20];
    uint32_t crc32;
} NVRAM_LegacyData_t;

#define NVRAM_DATA_SIZE       ((uint32_t)offsetof(NVRAM_Data_t, crc32))
#define NVRAM_DATA_V1_SIZE    ((uint32_t)offsetof(NVRAM_DataV1_t, crc32))
#define NVRAM_LEGACY_NAMED_DATA_SIZE ((uint32_t)offsetof(NVRAM_LegacyNamedData_t, crc32))
#define NVRAM_LEGACY_DATA_SIZE ((uint32_t)offsetof(NVRAM_LegacyData_t, crc32))

static NVRAM_Data_t nvram_ram;  /**< RAM 镜像：所有读写都通过此变量 */

static void nvram_fill_default_name(uint8_t axis, uint8_t index)
{
    uint16_t *dst;

    if (axis >= NVRAM_MAX_AXES || index >= NVRAM_POINTS_PER_AXIS) {
        return;
    }

    dst = nvram_ram.point_names[axis][index];
    memset(dst, 0, sizeof(nvram_ram.point_names[axis][index]));
    dst[0] = (uint16_t)('A' + axis);
    dst[1] = (uint16_t)'-';
    dst[2] = (uint16_t)'P';
    dst[3] = (uint16_t)('1' + index);
}

static void nvram_init_default_speeds(void)
{
    uint8_t axis;
    uint8_t index;

    for (axis = 0; axis < NVRAM_MAX_AXES; axis++) {
        for (index = 0; index < NVRAM_POINTS_PER_AXIS; index++) {
            nvram_ram.point_speeds[axis][index] = NVRAM_POINT_SPEED_DEFAULT;
        }
    }
}

static void nvram_init_default_names(void)
{
    uint8_t axis;
    uint8_t index;

    for (axis = 0; axis < NVRAM_MAX_AXES; axis++) {
        for (index = 0; index < NVRAM_POINTS_PER_AXIS; index++) {
            nvram_fill_default_name(axis, index);
        }
    }
}

/* ==========================================================================
 * CRC32 计算 (标准 CRC-32/MPEG-2)
 * ========================================================================== */
static uint32_t nvram_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80000000U)
                crc = (crc << 1) ^ 0x04C11DB7U;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ==========================================================================
 * Flash 读取
 * ========================================================================== */
static void nvram_load_from_flash(void)
{
    const uint32_t *src = (const uint32_t *)NVRAM_FLASH_ADDR;
    uint32_t *dst = (uint32_t *)&nvram_ram;
    uint32_t word_count = (sizeof(NVRAM_Data_t) + 3) / 4;

    for (uint32_t i = 0; i < word_count; i++) {
        dst[i] = src[i];
    }
}

static bool nvram_verify_flash_image(void)
{
    const NVRAM_Data_t *flash = (const NVRAM_Data_t *)NVRAM_FLASH_ADDR;
    uint32_t calc_crc;

    if (flash->magic != NVRAM_MAGIC) {
        return false;
    }

    calc_crc = nvram_crc32((const uint8_t *)flash, NVRAM_DATA_SIZE);
    if (calc_crc != flash->crc32) {
        return false;
    }

    return memcmp((const void *)flash, (const void *)&nvram_ram, sizeof(NVRAM_Data_t)) == 0;
}

/* ==========================================================================
 * Flash 写入 (擦除 + 编程)
 * ========================================================================== */
static HAL_StatusTypeDef nvram_write_to_flash(void)
{
    HAL_StatusTypeDef status;
    uint32_t sector_error = 0;
    FLASH_EraseInitTypeDef erase_cfg = {0};

    /* 1. 解锁 Flash。首次启动时 IWDG 尚未启动，此调用安全地不执行。 */
    BSP_IWDG_Feed();
    HAL_FLASH_Unlock();

    /* 2. 擦除目标扇区 */
    erase_cfg.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_cfg.Sector       = NVRAM_FLASH_SECTOR;
    erase_cfg.NbSectors    = 1;
    erase_cfg.VoltageRange = FLASH_VOLTAGE_RANGE_3; /* 2.7~3.6V, for VCC=3.3V */

    status = HAL_FLASHEx_Erase(&erase_cfg, &sector_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        return status;
    }
    BSP_IWDG_Feed();

    /* 3. 按 32-bit word 写入 */
    uint32_t *src = (uint32_t *)&nvram_ram;
    uint32_t addr = NVRAM_FLASH_ADDR;
    uint32_t word_count = (sizeof(NVRAM_Data_t) + 3) / 4;

    for (uint32_t i = 0; i < word_count; i++) {
        if ((i & 0x3FU) == 0U) {
            BSP_IWDG_Feed();
        }
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return status;
        }
        addr += 4;
    }

    /* 4. 锁定 Flash */
    HAL_FLASH_Lock();
    return HAL_OK;
}

/* ==========================================================================
 * 公共 API
 * ========================================================================== */

void NVRAM_Init(void)
{
    const NVRAM_LegacyNamedData_t *legacy_named =
        (const NVRAM_LegacyNamedData_t *)NVRAM_FLASH_ADDR;
    const NVRAM_LegacyData_t *legacy = (const NVRAM_LegacyData_t *)NVRAM_FLASH_ADDR;

    /* 从 Flash 加载 */
    nvram_load_from_flash();

    /* 校验 magic 和 CRC */
    bool valid = false;
    if (nvram_ram.magic == NVRAM_MAGIC) {
        uint32_t calc_crc = nvram_crc32((const uint8_t *)&nvram_ram, NVRAM_DATA_SIZE);
        if (calc_crc == nvram_ram.crc32) {
            valid = true;
            /* 速度为 0 视为未写入的非法值, 回退默认值并刷新 CRC。 */
            bool speed_fixed = false;
            for (uint8_t a = 0; a < NVRAM_MAX_AXES; a++) {
                for (uint8_t i = 0; i < NVRAM_POINTS_PER_AXIS; i++) {
                    if (nvram_ram.point_speeds[a][i] == 0U) {
                        nvram_ram.point_speeds[a][i] = NVRAM_POINT_SPEED_DEFAULT;
                        speed_fixed = true;
                    }
                }
            }
            if (speed_fixed) {
                nvram_ram.crc32 = nvram_crc32((const uint8_t *)&nvram_ram, NVRAM_DATA_SIZE);
                ATOMIC_ENTER();
                HAL_StatusTypeDef s = nvram_write_to_flash();
                (void)s;
                ATOMIC_EXIT();
            }
        }
    }

    /* 迁移: "AXT8" 早期布局 (无速度区) */
    if (!valid) {
        const NVRAM_DataV1_t *v1 = (const NVRAM_DataV1_t *)NVRAM_FLASH_ADDR;
        if (v1->magic == NVRAM_MAGIC &&
            nvram_crc32((const uint8_t *)v1, NVRAM_DATA_V1_SIZE) == v1->crc32) {
            memset(&nvram_ram, 0, sizeof(nvram_ram));
            nvram_ram.magic = NVRAM_MAGIC;
            memcpy(nvram_ram.points, v1->points, sizeof(v1->points));
            memcpy(nvram_ram.point_names, v1->point_names, sizeof(v1->point_names));
            nvram_init_default_speeds();
            nvram_ram.crc32 = nvram_crc32((const uint8_t *)&nvram_ram, NVRAM_DATA_SIZE);
            valid = true;

            ATOMIC_ENTER();
            HAL_StatusTypeDef s = nvram_write_to_flash();
            (void)s;
            ATOMIC_EXIT();
        }
    }

    if (!valid &&
        legacy_named->magic == NVRAM_LEGACY_MAGIC &&
        nvram_crc32((const uint8_t *)legacy_named, NVRAM_LEGACY_NAMED_DATA_SIZE) ==
            legacy_named->crc32) {
        memset(&nvram_ram, 0, sizeof(nvram_ram));
        nvram_ram.magic = NVRAM_MAGIC;
        memcpy(nvram_ram.points, legacy_named->points, sizeof(legacy_named->points));
        nvram_init_default_names();
        memcpy(nvram_ram.point_names, legacy_named->point_names,
               sizeof(legacy_named->point_names));
        nvram_init_default_speeds();
        nvram_ram.crc32 = nvram_crc32((const uint8_t *)&nvram_ram, NVRAM_DATA_SIZE);
        valid = true;

        ATOMIC_ENTER();
        HAL_StatusTypeDef s = nvram_write_to_flash();
        (void)s;
        ATOMIC_EXIT();
    }

    if (!valid &&
        legacy->magic == NVRAM_LEGACY_MAGIC &&
        nvram_crc32((const uint8_t *)legacy, NVRAM_LEGACY_DATA_SIZE) == legacy->crc32) {
        memset(&nvram_ram, 0, sizeof(nvram_ram));
        nvram_ram.magic = NVRAM_MAGIC;
        memcpy(nvram_ram.points, legacy->points, sizeof(legacy->points));
        nvram_init_default_names();
        nvram_init_default_speeds();
        nvram_ram.crc32 = nvram_crc32((const uint8_t *)&nvram_ram, NVRAM_DATA_SIZE);
        valid = true;

        ATOMIC_ENTER();
        HAL_StatusTypeDef s = nvram_write_to_flash();
        (void)s;
        ATOMIC_EXIT();
    }

    /* 数据无效：初始化为零 */
    if (!valid) {
        memset(&nvram_ram, 0, sizeof(nvram_ram));
        nvram_ram.magic = NVRAM_MAGIC;
        nvram_init_default_names();
        nvram_init_default_speeds();

        /* 首次初始化时写入 Flash，后续重启不再重复写入 */
        uint32_t calc_crc = nvram_crc32((const uint8_t *)&nvram_ram, NVRAM_DATA_SIZE);
        nvram_ram.crc32 = calc_crc;

        ATOMIC_ENTER();
        HAL_StatusTypeDef s = nvram_write_to_flash();
        (void)s;
        ATOMIC_EXIT();
    }
}

bool NVRAM_Save(void)
{
    /* 计算 CRC */
    uint32_t calc_crc = nvram_crc32((const uint8_t *)&nvram_ram, NVRAM_DATA_SIZE);
    nvram_ram.crc32 = calc_crc;

    /* 原子写入 + 重试一次 */
    bool ok = false;
    ATOMIC_ENTER();
    HAL_StatusTypeDef status = nvram_write_to_flash();
    if (status == HAL_OK && nvram_verify_flash_image()) {
        ok = true;
    } else {
        status = nvram_write_to_flash();
        ok = (status == HAL_OK) && nvram_verify_flash_image();
    }
    ATOMIC_EXIT();

    return ok;
}

int32_t NVRAM_GetPositionPoint(uint8_t axis, uint8_t index)
{
    if (axis >= NVRAM_MAX_AXES || index >= NVRAM_POINTS_PER_AXIS) return 0;
    return nvram_ram.points[axis][index];
}

void NVRAM_SetPositionPoint(uint8_t axis, uint8_t index, int32_t pos)
{
    if (axis >= NVRAM_MAX_AXES || index >= NVRAM_POINTS_PER_AXIS) return;
    nvram_ram.points[axis][index] = pos;
}

uint16_t NVRAM_GetPositionPointNameUnit(uint8_t axis, uint8_t index, uint8_t char_index)
{
    if (axis >= NVRAM_MAX_AXES ||
        index >= NVRAM_POINTS_PER_AXIS ||
        char_index >= NVRAM_POINT_NAME_LEN) {
        return 0;
    }
    return nvram_ram.point_names[axis][index][char_index];
}

void NVRAM_SetPositionPointNameUnit(uint8_t axis, uint8_t index, uint8_t char_index, uint16_t value)
{
    if (axis >= NVRAM_MAX_AXES ||
        index >= NVRAM_POINTS_PER_AXIS ||
        char_index >= NVRAM_POINT_NAME_LEN) {
        return;
    }
    nvram_ram.point_names[axis][index][char_index] = value;
}

uint32_t NVRAM_GetPositionPointSpeed(uint8_t axis, uint8_t index)
{
    if (axis >= NVRAM_MAX_AXES || index >= NVRAM_POINTS_PER_AXIS) {
        return NVRAM_POINT_SPEED_DEFAULT;
    }
    return nvram_ram.point_speeds[axis][index];
}

void NVRAM_SetPositionPointSpeed(uint8_t axis, uint8_t index, uint32_t speed)
{
    if (axis >= NVRAM_MAX_AXES || index >= NVRAM_POINTS_PER_AXIS) {
        return;
    }
    nvram_ram.point_speeds[axis][index] = speed;
}
