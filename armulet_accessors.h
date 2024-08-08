#pragma once
#include "pico.h"
#include "armulet.h"

#define A_ROM_START    0x0000
#define A_ROM_END      0x4000

#define A_RAM_START    0x20000000
//#if PICO_RP2350
#define A_RAM_END      0x20082000
//#else
//#define A_RAM_END      0x20042000
//#endif

#define A_FLASH_START    0x10000000
#define A_FLASH_END      0x10200000

#if !ARMULET_USE_REAL_MEMORY
extern uint8_t rom_memory[A_ROM_END - A_ROM_START];
extern uint8_t ram_memory[A_RAM_END - A_RAM_START];
extern uint8_t flash_memory[A_FLASH_END - A_FLASH_START];
#endif

static inline uint32_t armulet_get_pc(const armulet_cpu_t *cpu) {
    assert(!(cpu->regs[ARM_REG_PC] & 1));
    return cpu->regs[ARM_REG_PC];
}

static inline bool armulet_get_N(const armulet_cpu_t *cpu) {
#if ARMULET_USE_LAZY_NZ
    return ((int32_t)cpu->lazy_nz_val) < 0;
#else
    return cpu->_N;
#endif
}

static inline bool armulet_get_Z(const armulet_cpu_t *cpu) {
#if ARMULET_USE_LAZY_NZ
    return !cpu->lazy_nz_val;
#else
    return cpu->_Z;
#endif
}

static inline void armulet_set_NZ(armulet_cpu_t *cpu, bool N, bool Z) {
#if ARMULET_USE_LAZY_NZ
    if (Z == 1) {
        assert(!N);
        cpu->lazy_nz_val = 0;
    } else {
        cpu->lazy_nz_val = N ? 0xffffffffu : 1;
    }
#else
    cpu->_N = N;
    cpu->_Z = Z;
#endif
}

static inline bool armulet_get_V(armulet_cpu_t *cpu) {
#if ARMULET_USE_LAZY_Z
    return (((int32_t) (cpu->lazy_v0 ^ cpu->lazy_v1)) < 0) && (((int32_t) (cpu->lazy_v2 ^ cpu->lazy_v1)) >= 0);
#else
    return cpu->_V;
#endif
}

static inline void armulet_set_V(armulet_cpu_t *cpu, bool V) {
#if ARMULET_USE_LAZY_Z
    cpu->lazy_v0 = V ? -1 : 0;
    cpu->lazy_v1 = cpu->lazy_v2 = 0;
#else
    cpu->_V = V;
#endif
}

static inline bool armulet_is_rom(uint32_t pc) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
    return pc >= A_ROM_START && pc < A_ROM_END;
#pragma GCC diagnostic pop
}

static inline bool armulet_is_ram(uint32_t pc) {
    return pc >= A_RAM_START && pc < A_RAM_END;
}

static inline bool armulet_is_flash(uint32_t pc) {
    return pc >= A_FLASH_START && pc < A_FLASH_END;
}

static inline bool armulet_is_valid_pc(uint32_t pc) {
    return armulet_is_rom(pc) || armulet_is_ram(pc) || armulet_is_flash(pc);
}

static inline void armulet_update_nz(armulet_cpu_t *cpu, uint32_t val) {
#if ARMULET_USE_LAZY_NZ
    cpu->lazy_nz_val = val;
#else
    cpu->_Z = !val;
    cpu->_N = ((int32_t) val) < 0;
#endif
}

#if ARMULET_FEATURE_SPECIAL_READ
static bool armulet_is_special_address(uint32_t addr) {
    return (addr >> 28u) == 0xeu;
}
#endif

static inline void *armulet_resolve_address(uint32_t addr, int size) {
#if ARMULET_USE_REAL_MEMORY
    (void)size;
    void *rc = (void*)addr;
#else
    void *rc = 0;
    if (armulet_is_rom(addr)) rc = rom_memory + addr - A_ROM_START;
    else if (armulet_is_ram(addr)) rc = ram_memory + addr - A_RAM_START;
    else if (armulet_is_flash(addr)) rc = flash_memory + addr - A_FLASH_START;
    hard_assert(rc);
    hard_assert(!((size - 1) & (uintptr_t) addr)); // check alignment
#endif
    return rc;
}

static inline uint8_t armulet_read_u8(uint32_t addr) {
#if ARMULET_FEATURE_SPECIAL_READ
    if (is_special_address(addr)) return (uint8_t)armulet_cb_special_read(addr, 1);
#endif
    void *mem = armulet_resolve_address(addr, 1);
    return *(uint8_t *) mem;
}

static inline uint16_t armulet_read_u16(uint32_t addr) {
#if ARMULET_FEATURE_SPECIAL_READ
    if (is_special_address(addr)) return (uint16_t)armulet_cb_special_read(addr, 2);
#endif
    void *mem = armulet_resolve_address(addr, 2);
    return *(uint16_t *) mem;
}

static inline uint32_t armulet_read_u32(uint32_t addr) {
#if ARMULET_FEATURE_SPECIAL_READ
    if (is_special_address(addr)) return armulet_cb_special_read(addr, 4);
#endif
    void *mem = armulet_resolve_address(addr, 4);
    return *(uint32_t *) mem;
}

static inline void armulet_write_u8(uint32_t addr, uint8_t val) {
#if ARMULET_FEATURE_SPECIAL_READ
    if (is_special_address(addr)) { armulet_cb_special_write(addr, 1, val); return; }
#endif
    void *mem = armulet_resolve_address(addr, 1);
    *(uint8_t *) mem = val;
}

static inline void armulet_write_u16(uint32_t addr, uint16_t val) {
#if ARMULET_FEATURE_SPECIAL_READ
    if (is_special_address(addr)) { armulet_cb_special_write(addr, 2, val); return; }
#endif
    void *mem = armulet_resolve_address(addr, 2);
    *(uint16_t *) mem = val;
}

static inline void armulet_write_u32(uint32_t addr, uint32_t val) {
#if ARMULET_FEATURE_SPECIAL_READ
    if (is_special_address(addr)) { armulet_cb_special_write(addr, 4, val); return; }
#endif
    void *mem = armulet_resolve_address(addr, 4);
    *(uint32_t *) mem = val;
}

static inline void armulet_update_pc(armulet_cpu_t *cpu, uint32_t new_pc) {
    hard_assert(new_pc & 1u);
    cpu->regs[ARM_REG_PC] = new_pc & ~1u;
#if !ARMULET_USE_ASM
    cpu->pc_delta = 0;
#endif
}
