/**
 * @file    nvram_auto.c
 * @brief   Descriptor-based automatic persistence in separate Sectors 5 and 6.
 *
 * Registered variables are serialized into one committed snapshot per bank.
 * A record is committed by programming its final word last, and the new bank
 * is validated before the old bank is erased.  Sector 7 and its host-tool
 * point database are intentionally not accessed here.
 */

#include "nvram.h"
#include "app_config.h"
#include "bsp_iwdg.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

#define AUTO_FLASH_BANK_COUNT   2U
#define AUTO_RECORD_MAGIC       0x4155544FU /* "AUTO" */
#define AUTO_RECORD_VERSION     1U
#define AUTO_COMMIT_MAGIC       0xC04D4D49U
#define AUTO_MAX_DESCRIPTORS    64U
#define AUTO_MAX_PAYLOAD        8192U
#define AUTO_HEADER_SIZE        24U
#define AUTO_DEBOUNCE_MS        NVRAM_AUTO_SAVE_DELAY_MS

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    uint32_t payload_size;
    uint32_t payload_crc;
    uint32_t commit;
} AutoRecordHeader_t;

typedef struct {
    uint32_t name_hash;
    uint32_t size;
} AutoEntryHeader_t;

typedef struct {
    uint32_t sector;
    uint32_t start;
    uint32_t end;
} AutoFlashBank_t;

typedef struct {
    uint32_t latest;
    uint32_t next;
    uint32_t sequence;
    bool found;
} AutoBankScan_t;

static const AutoFlashBank_t auto_banks[AUTO_FLASH_BANK_COUNT] = {
    { FLASH_SECTOR_5, 0x08020000U, 0x08040000U },
    { FLASH_SECTOR_6, 0x08040000U, 0x08060000U }
};

extern const NVRAM_PersistDescriptor_t __persist_desc_start[];
extern const NVRAM_PersistDescriptor_t __persist_desc_end[];

static uint8_t auto_payload[AUTO_MAX_PAYLOAD];
static uint32_t auto_payload_size;
static uint32_t auto_snapshot_size;
static uint32_t auto_snapshot_crc;
static uint32_t auto_dirty_age_ms;
static bool auto_dirty;
static bool auto_initialized;
static NVRAM_Auto_SaveGuard_t auto_save_guard;

static uint32_t auto_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (uint8_t bit = 0; bit < 8U; bit++) {
            crc = (crc & 0x80000000U) != 0U
                ? (crc << 1) ^ 0x04C11DB7U
                : crc << 1;
        }
    }
    return crc;
}

static uint32_t auto_name_hash(const char *name)
{
    uint32_t hash = 2166136261U;

    if (name == NULL) return 0U;
    while (*name != '\0') {
        hash ^= (uint8_t)*name++;
        hash *= 16777619U;
    }
    return hash;
}

static uint32_t auto_descriptor_count(void)
{
    uintptr_t start = (uintptr_t)__persist_desc_start;
    uintptr_t end = (uintptr_t)__persist_desc_end;

    if (end < start) return 0U;
    return (uint32_t)((end - start) / sizeof(NVRAM_PersistDescriptor_t));
}

static bool auto_descriptors_valid(uint32_t count)
{
    const NVRAM_PersistDescriptor_t *descs = __persist_desc_start;

    if (count > AUTO_MAX_DESCRIPTORS) return false;
    for (uint32_t i = 0; i < count; i++) {
        if (descs[i].address == NULL || descs[i].size == 0U ||
            descs[i].name == NULL || auto_name_hash(descs[i].name) == 0U) {
            return false;
        }
        for (uint32_t j = 0; j < i; j++) {
            /* Records identify entries by hash plus size. Reject any hash
             * collision up front so two variables can never be ambiguous. */
            if (auto_name_hash(descs[i].name) == auto_name_hash(descs[j].name)) {
                return false;
            }
        }
    }
    return true;
}

static bool auto_build_payload(uint8_t *buffer, uint32_t capacity, uint32_t *size_out)
{
    const NVRAM_PersistDescriptor_t *descs = __persist_desc_start;
    uint32_t count = auto_descriptor_count();
    uint32_t offset = sizeof(uint32_t);

    if (size_out == NULL || !auto_descriptors_valid(count)) return false;
    if (offset > capacity) return false;
    memcpy(&buffer[0], &count, sizeof(count));

    for (uint32_t i = 0; i < count; i++) {
        uint32_t size = descs[i].size;
        uint32_t padded;
        AutoEntryHeader_t entry = { auto_name_hash(descs[i].name), size };

        if (size > AUTO_MAX_PAYLOAD || size > UINT32_MAX - 3U) {
            return false;
        }
        padded = (size + 3U) & ~3U;
        if (offset > capacity || sizeof(entry) > capacity - offset ||
            padded > capacity - offset - sizeof(entry)) {
            return false;
        }
        memcpy(&buffer[offset], &entry, sizeof(entry));
        offset += sizeof(entry);
        memcpy(&buffer[offset], descs[i].address, size);
        if (padded > size) {
            memset(&buffer[offset + size], 0, padded - size);
        }
        offset += padded;
    }

    *size_out = offset;
    return true;
}

static bool auto_record_valid(uint32_t address, const AutoFlashBank_t *bank,
                              AutoRecordHeader_t *header_out)
{
    const AutoRecordHeader_t *header;

    if (bank == NULL || address < bank->start ||
        address > bank->end - AUTO_HEADER_SIZE) {
        return false;
    }
    header = (const AutoRecordHeader_t *)address;

    if (header->magic != AUTO_RECORD_MAGIC ||
        header->version != AUTO_RECORD_VERSION ||
        header->commit != AUTO_COMMIT_MAGIC ||
        header->payload_size < sizeof(uint32_t) ||
        header->payload_size > AUTO_MAX_PAYLOAD ||
        header->payload_size > bank->end - address - AUTO_HEADER_SIZE) {
        return false;
    }
    if (auto_crc32((const uint8_t *)(address + AUTO_HEADER_SIZE),
                   header->payload_size) != header->payload_crc) {
        return false;
    }
    if (header_out != NULL) *header_out = *header;
    return true;
}

static bool auto_erase_bank(const AutoFlashBank_t *bank)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;
    HAL_StatusTypeDef status;

    if (bank == NULL) return false;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = bank->sector;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    HAL_FLASH_Lock();
    return status == HAL_OK;
}

static void auto_scan_bank(const AutoFlashBank_t *bank, AutoBankScan_t *scan)
{
    uint32_t address;

    if (bank == NULL || scan == NULL) return;
    address = bank->start;
    scan->latest = 0U;
    scan->next = address;
    scan->sequence = 0U;
    scan->found = false;

    while (address <= bank->end - AUTO_HEADER_SIZE) {
        const uint32_t *words = (const uint32_t *)address;
        AutoRecordHeader_t header;
        uint32_t record_size;

        if (words[0] == 0xFFFFFFFFU) break;
        if (!auto_record_valid(address, bank, &header)) {
            /* A reset can leave a partially programmed tail.  It cannot be
             * reused without erasing the sector, so report the sector as full
             * and let the next save reclaim it. */
            address = bank->end;
            break;
        }
        if (header.payload_size > UINT32_MAX - 3U) break;
        record_size = AUTO_HEADER_SIZE + ((header.payload_size + 3U) & ~3U);
        if (record_size > bank->end - address) {
            address = bank->end;
            break;
        }
        scan->latest = address;
        scan->sequence = header.sequence;
        scan->found = true;
        address += record_size;
    }

    scan->next = address;
}

static bool auto_sequence_is_newer(uint32_t candidate, uint32_t current)
{
    return (int32_t)(candidate - current) > 0;
}

static uint32_t auto_find_latest(uint32_t *bank_index, uint32_t *next_address,
                                 uint32_t *sequence, bool *found_out,
                                 AutoBankScan_t scans[AUTO_FLASH_BANK_COUNT])
{
    uint32_t selected = 0U;
    bool found = false;

    for (uint32_t i = 0; i < AUTO_FLASH_BANK_COUNT; i++) {
        auto_scan_bank(&auto_banks[i], &scans[i]);
        if (scans[i].found &&
            (!found || auto_sequence_is_newer(scans[i].sequence,
                                               scans[selected].sequence))) {
            selected = i;
            found = true;
        }
    }

    if (bank_index != NULL) *bank_index = found ? selected : 0U;
    if (next_address != NULL) *next_address = scans[found ? selected : 0U].next;
    if (sequence != NULL) *sequence = found ? scans[selected].sequence : 0U;
    if (found_out != NULL) *found_out = found;
    return found ? scans[selected].latest : 0U;
}

static bool auto_load_payload(const uint8_t *payload, uint32_t payload_size)
{
    const NVRAM_PersistDescriptor_t *descs = __persist_desc_start;
    uint32_t count;
    uint32_t descriptor_count = auto_descriptor_count();
    uint64_t matched_descriptors = 0U;

    if (payload_size < sizeof(count)) return false;
    memcpy(&count, payload, sizeof(count));
    if (count > AUTO_MAX_DESCRIPTORS ||
        !auto_descriptors_valid(descriptor_count)) return false;

    /* Validate the complete image before changing any registered variable. */
    uint32_t offset = sizeof(uint32_t);
    for (uint32_t i = 0; i < count; i++) {
        AutoEntryHeader_t entry;

        if (offset + sizeof(entry) > payload_size) return false;
        memcpy(&entry, &payload[offset], sizeof(entry));
        offset += sizeof(entry);
        uint32_t padded;
        if (entry.size == 0U || entry.size > AUTO_MAX_PAYLOAD ||
            entry.size > UINT32_MAX - 3U) return false;
        padded = (entry.size + 3U) & ~3U;
        if (offset > payload_size || padded > payload_size - offset) return false;

        for (uint32_t j = 0; j < descriptor_count; j++) {
            if (auto_name_hash(descs[j].name) == entry.name_hash &&
                descs[j].size == entry.size) {
                uint64_t bit = UINT64_C(1) << j;
                if ((matched_descriptors & bit) != 0U) return false;
                matched_descriptors |= bit;
                break;
            }
        }
        offset += padded;
    }
    if (offset != payload_size) return false;

    /* Unknown entries belong to older firmware and are ignored.  New
     * descriptors keep their compiled defaults until they are first saved. */
    offset = sizeof(uint32_t);
    for (uint32_t i = 0; i < count; i++) {
        AutoEntryHeader_t entry;
        memcpy(&entry, &payload[offset], sizeof(entry));
        offset += sizeof(entry);
        for (uint32_t j = 0; j < descriptor_count; j++) {
            if (auto_name_hash(descs[j].name) == entry.name_hash &&
                descs[j].size == entry.size) {
                memcpy((void *)descs[j].address, &payload[offset], entry.size);
                break;
            }
        }
        offset += (entry.size + 3U) & ~3U;
    }
    return true;
}

static bool auto_write_record(const AutoFlashBank_t *bank, uint32_t address,
                              uint32_t sequence)
{
    AutoRecordHeader_t header = {
        AUTO_RECORD_MAGIC, AUTO_RECORD_VERSION, sequence, auto_payload_size,
        auto_crc32(auto_payload, auto_payload_size), 0xFFFFFFFFU
    };
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t word_count;
    uint32_t padded_payload = (auto_payload_size + 3U) & ~3U;
    uint32_t record_size = (AUTO_HEADER_SIZE + padded_payload + 3U) & ~3U;

    if (bank == NULL || address < bank->start ||
        address > bank->end - record_size) return false;
    HAL_FLASH_Unlock();

    /* Leave the commit word erased until the complete payload is present. */
    for (uint32_t offset = 0;
         offset < offsetof(AutoRecordHeader_t, commit);
         offset += 4U) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + offset,
                                    ((const uint32_t *)&header)[offset / 4U]);
        if (status != HAL_OK) break;
    }
    word_count = padded_payload / 4U;
    for (uint32_t i = 0; status == HAL_OK && i < word_count; i++) {
        uint32_t word = 0U;
        memcpy(&word, &auto_payload[i * 4U], sizeof(word));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                    address + AUTO_HEADER_SIZE + i * 4U, word);
    }
    if (status == HAL_OK) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                    address + offsetof(AutoRecordHeader_t, commit),
                                    AUTO_COMMIT_MAGIC);
    }
    HAL_FLASH_Lock();
    return status == HAL_OK && auto_record_valid(address, bank, NULL);
}

bool NVRAM_Auto_Save(void)
{
    AutoBankScan_t scans[AUTO_FLASH_BANK_COUNT];
    uint32_t active_bank;
    uint32_t target_bank;
    uint32_t next_address;
    uint32_t previous_sequence;
    uint32_t latest;
    bool found;
    bool ok;

    if (!auto_initialized ||
        (auto_save_guard != NULL && !auto_save_guard()) ||
        !auto_build_payload(auto_payload, sizeof(auto_payload),
                                                   &auto_payload_size)) {
        return false;
    }
    latest = auto_find_latest(&active_bank, &next_address, &previous_sequence,
                              &found, scans);
    (void)latest;

    /* Always write a complete snapshot to the inactive bank.  The previous
     * bank is kept intact until the new record has passed validation. */
    target_bank = found ? (active_bank ^ 1U) : 0U;
    next_address = auto_banks[target_bank].start;
    if (auto_payload_size > UINT32_MAX - 3U ||
        AUTO_HEADER_SIZE + ((auto_payload_size + 3U) & ~3U) >
            auto_banks[target_bank].end - next_address) {
        return false;
    }

    ATOMIC_ENTER();
    BSP_IWDG_Feed();
    ok = auto_erase_bank(&auto_banks[target_bank]);
    if (ok) {
        BSP_IWDG_Feed();
        ok = auto_write_record(&auto_banks[target_bank], next_address,
                               previous_sequence + 1U);
    }
    if (ok && found) {
        /* This erase is deliberately last.  If it fails, both banks remain
         * valid and the next save can reclaim the older one. */
        BSP_IWDG_Feed();
        (void)auto_erase_bank(&auto_banks[active_bank]);
    }
    ATOMIC_EXIT();
    if (!ok) return false;

    auto_snapshot_size = auto_payload_size;
    auto_snapshot_crc = auto_crc32(auto_payload, auto_payload_size);
    auto_dirty = false;
    auto_dirty_age_ms = 0U;
    return true;
}

void NVRAM_Auto_Init(void)
{
    AutoBankScan_t scans[AUTO_FLASH_BANK_COUNT];
    uint32_t bank_index;
    uint32_t next_address;
    uint32_t sequence;
    bool found;
    uint32_t latest = auto_find_latest(&bank_index, &next_address, &sequence,
                                       &found, scans);

    (void)bank_index;
    (void)next_address;
    (void)sequence;

    auto_initialized = false;
    auto_dirty = false;
    auto_dirty_age_ms = 0U;
    auto_payload_size = 0U;
    auto_snapshot_size = 0U;
    auto_snapshot_crc = 0U;

    if (found) {
        const AutoRecordHeader_t *header = (const AutoRecordHeader_t *)latest;
        if (auto_load_payload((const uint8_t *)(latest + AUTO_HEADER_SIZE),
                              header->payload_size)) {
            auto_initialized = true;
        }
    }
    if (auto_build_payload(auto_payload, sizeof(auto_payload), &auto_payload_size)) {
        auto_snapshot_size = auto_payload_size;
        auto_snapshot_crc = auto_crc32(auto_payload, auto_payload_size);
        auto_initialized = true;
    } else {
        auto_initialized = false;
    }
}

void NVRAM_Auto_Tick1ms(void)
{
    if (auto_dirty && auto_dirty_age_ms < UINT32_MAX) auto_dirty_age_ms++;
}

void NVRAM_Auto_Poll(void)
{
    uint32_t current_size;
    uint32_t current_crc;

    if (!auto_initialized || !auto_build_payload(auto_payload, sizeof(auto_payload),
                                                   &current_size)) return;
    current_crc = auto_crc32(auto_payload, current_size);
    if (current_size != auto_snapshot_size ||
        current_crc != auto_snapshot_crc) {
        if (!auto_dirty) auto_dirty_age_ms = 0U;
        auto_dirty = true;
    }
    auto_payload_size = current_size;
    if (auto_dirty && auto_dirty_age_ms >= AUTO_DEBOUNCE_MS &&
        (auto_save_guard == NULL || auto_save_guard())) {
        (void)NVRAM_Auto_Save();
    }
}

void NVRAM_Auto_MarkDirty(void)
{
    if (!auto_initialized) return;
    auto_dirty = true;
    auto_dirty_age_ms = 0U;
}

bool NVRAM_Auto_IsDirty(void)
{
    return auto_dirty;
}

void NVRAM_Auto_SetSaveGuard(NVRAM_Auto_SaveGuard_t guard)
{
    auto_save_guard = guard;
}
