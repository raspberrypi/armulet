/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if PICO_RP2350
#include "pico.h"
#endif

// function sections
// try to standardize on args to hooks
// in case we don't want to align
#include "varmulet_macros.inc.S"

#if ARMULET_FEATURE_PROFILER
.section .data
.global address_profile
.p2align 2
address_profile:
.space 65536
#endif

.section .text.varmulet_step

#if ARMULET_FEATURE_STEP
.global varmulet_step
// note: this is really a custom thing
varmulet_step:
    addi        sp, sp, -4
    sw          ra, (sp)
    call_hook_enter_fn_trash_tmp2_work1
#if VARMULET_USE_EARLY_INSTR_READ
    read_mem_u16_unchecked r_inst, r_pc
#endif
    mv          r_work0, r_next_inst
    la          r_next_inst, varmulet_step_done // override whatever is in there
    jr          r_work0
varmulet_step_done:
    call_hook_exit_fn_trash_tmp2_work1
    lw          ra, (sp)
    add         sp, sp, 4
    ret
#endif

.section .text.varmulet_run
.global varmulet_run
varmulet_run:
    mv          r_cpu, a0
    addi        sp, sp, -4
    sw          ra, (sp)
    call_hook_enter_fn_trash_tmp2_work1
#if ARMULET_FEATURE_CALL
    // Store the address to go to when we return from Armv6-M on the stack
    // (note the register mapped as r_tmp0 should have been saved by
    // varmulet_hook_default_enter_fn or similar in the previous jalr call)
    addi        sp, sp, -4
    lui         r_tmp0, %hi(varmulet_run_return)
    addi        r_tmp0, r_tmp0, %lo(varmulet_run_return)
    sw          r_tmp0, 0(sp)
#endif
    next_instruction

varmulet_run_return:
    call_hook_exit_fn_trash_tmp2_work1
    lw          a0, 0(r_cpu)
    lw          ra, (sp)
    add         sp, sp, 4
    ret

// these should be linked in
#if 0 && VARMULET_USE_WATERMARK
read8_special:
    ebreak

read16_special:
    ebreak

read32_special:
    ebreak

write8_special:
    ebreak

write16_special:
    ebreak

write32_special:
    ebreak
#endif

