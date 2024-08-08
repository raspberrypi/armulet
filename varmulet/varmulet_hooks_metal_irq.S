/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if PICO_RP2350
#include "pico.h"
#endif

#include "varmulet_macros.inc.S"

// -------------------------------------------------------------------------------
// This is (semi-incomplete; it assumes only one IRQ handler)
// example code for handling an IRQ inside the ARM world (i.e. pushing an exception
// frame onto the ARM stack and continuing in the original varmulet instance
// -------------------------------------------------------------------------------
.global metal_irq_pending
.global metal_irq_hook_exc_return

metal_irq_pending:
    // the idea here is to disable IRQs for when we do an MRET (leaving the RISC-V IRQ pending)
    // and return from the exception with r_next_inst set to cause 'vtake_exception' to be called once
    // the currently executing instruction completes.
    // todo we assume here directly back into varmulet code, and that we don't have stack to unwind,
    //      so this is not very general code atm
    li          r_next_inst, 0x80
    csrc        mstatus, r_next_inst // set interrupt enable for mret to 0
    la          r_next_inst, vtake_exception
    mret

metal_take_exception:
    // create an ARMv6M exception stack frame
    lw          r_tmp0, CPU_OFFSET_SP(r_cpu)
    addi        r_tmp0, r_tmp0, -32
    sw          r_tmp0, CPU_OFFSET_SP(r_cpu)

    lw          r_tmp1, CPU_OFFSET_R0(r_cpu)
    sw          r_tmp1, 0(r_tmp0)
    lw          r_tmp1, CPU_OFFSET_R1(r_cpu)
    sw          r_tmp1, 4(r_tmp0)
    lw          r_tmp1, CPU_OFFSET_R2(r_cpu)
    sw          r_tmp1, 8(r_tmp0)
    lw          r_tmp1, CPU_OFFSET_R3(r_cpu)
    sw          r_tmp1, 12(r_tmp0)
    lw          r_tmp1, CPU_OFFSET_R12(r_cpu)
    sw          r_tmp1, 16(r_tmp0)
    lw          r_tmp1, CPU_OFFSET_LR(r_cpu)
    sw          r_tmp1, 20(r_tmp0)
    sw          r_pc, 24(r_tmp0)
    flags_to_apsr r_tmp1, r_work0, r_work1
    sw          r_tmp1, 28(r_tmp0) // todo save flags and such

    li          r_tmp1, 0xfffffff1
    sw          r_tmp1, CPU_OFFSET_LR(r_cpu)

    // load our single ARM irq_handler
    la          r_tmp1, metal_arm_irq_handler
    lw          r_pc, (r_tmp1)
    andi        r_pc, r_pc, ~1

    // continue executing.
    lw          r_next_inst, VASM_HOOKS_OFFSET_NEXT_INSTRUCTION(r_asm_hooks)
    next_instruction

metal_irq_hook_exc_return:
#if ARMULET_FEATURE_CALL
    li          r_tmp0, ARMULET_CALL_RETURN_ADDRESS
    // todo this should be user specified address
    beq         r_tmp0, r_work0, varmulet_do_call_return
#endif
    // pop the exception frame
    lw          r_tmp0, CPU_OFFSET_SP(r_cpu)
    lw          r_tmp1, 0(r_tmp0)
    sw          r_tmp1, CPU_OFFSET_R0(r_cpu)
    lw          r_tmp1, 4(r_tmp0)
    sw          r_tmp1, CPU_OFFSET_R1(r_cpu)
    lw          r_tmp1, 8(r_tmp0)
    sw          r_tmp1, CPU_OFFSET_R2(r_cpu)
    lw          r_tmp1, 12(r_tmp0)
    sw          r_tmp1, CPU_OFFSET_R3(r_cpu)
    lw          r_tmp1, 16(r_tmp0)
    sw          r_tmp1, CPU_OFFSET_R12(r_cpu)
    lw          r_tmp1, 20(r_tmp0)
    sw          r_tmp1, CPU_OFFSET_LR(r_cpu)
    lw          r_pc, 24(r_tmp0)
    lw          r_tmp1, 28(r_tmp0)
    apsr_to_flags   r_tmp1, r_tmp2

    addi        r_tmp0, r_tmp0, 32
    sw          r_tmp0, CPU_OFFSET_SP(r_cpu)
    // todo note we reenable IRQs, but we s
    csrsi       mstatus, 0x8
    next_instruction
