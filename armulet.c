#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <stdio.h>

#include "armulet_accessors.h"

#if ARMULET_USE_ASM
#include "varmulet.h"
#else
#include "carmulet.h"
#endif

void armulet_reset_cpu(armulet_cpu_t *cpu) {
    memset(cpu, 0, sizeof(armulet_cpu_t));
#if ARMULET_USE_LAZY_NZ
    cpu->lazy_nz_val = 1;
#endif
}

#if ARMULET_DEBUG
#if ARMULET_USE_ASM
#include "varmulet.h"
varmulet_asm_hooks_t single_step_asm_hooks;
#endif

void armulet_single_step(armulet_cpu_t *cpu) {
#if ARMULET_FEATURE_STEP_STATUS
    cpu->step_status = ARMULET_IST_NORMAL;
#endif
#if ARMULET_USE_ASM
    varmulet_step(cpu, &single_step_asm_hooks);
#else
    carmulet_single_step(cpu);
#endif
}


#endif

#if !ARMULET_USE_REAL_MEMORY
void armulet_zap(uint8_t byte) {
    memset(rom_memory, byte, sizeof(rom_memory));
    memset(ram_memory, byte, sizeof(ram_memory));
    memset(flash_memory, byte, sizeof(flash_memory));
}

#endif
