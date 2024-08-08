/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if PICO_RP2350
#include "pico.h"
#endif

#include "varmulet_macros.inc.S"

.macro varmulet_step_status VAL, tmp
#if ARMULET_FEATURE_STEP_STATUS
    li      \tmp, \VAL
    sw      \tmp, CPU_OFFSET_STEP_STATUS(r_cpu)
#endif
.endm

.macro varmulet_step_status_ni VAL, tmp, delta
#if ARMULET_FEATURE_STEP_STATUS
    li      \tmp, \VAL
    sw      \tmp, CPU_OFFSET_STEP_STATUS(r_cpu)
    addi    r_pc, r_pc, -\delta
    next_instruction
#endif
.endm

.global install_varmulet_test_hooks
.text
install_varmulet_test_hooks:
#if ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
#error ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION not supported for tests
#endif
    la      a1, test_bkpt_instr
    sw      a1, (4*VASM_HOOKS_INDEX_BKPT_INSTR)(a0)
#if ARMULET_FEATURE_STEP_STATUS
    la      a1, test_svc_instr
    sw      a1, (4*VASM_HOOKS_INDEX_SVC_INSTR)(a0)
    la      a1, test_undefined16
    sw      a1, (4*VASM_HOOKS_INDEX_UNDEFINED16)(a0)
    la      a1, test_undefined32
    sw      a1, (4*VASM_HOOKS_INDEX_UNDEFINED32)(a0)
#endif
    ret

test_bkpt_instr:
    addi        r_pc, r_pc, -2
#if ARMULET_FEATURE_STEP_STATUS
    varmulet_step_status ARMULET_IST_BREAKPOINT, r_tmp0
#endif
    next_instruction

#if ARMULET_FEATURE_STEP_STATUS
test_svc_instr:
    varmulet_step_status ARMULET_IST_SVC, r_tmp0
    next_instruction

test_undefined16:
    varmulet_step_status_ni ARMULET_IST_UNDEFINED16, r_tmp0, 2
    next_instruction

test_undefined32:
    varmulet_step_status_ni ARMULET_IST_UNDEFINED32, r_tmp1, 4
    next_instruction
#endif