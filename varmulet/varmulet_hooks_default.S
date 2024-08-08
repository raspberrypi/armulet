/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if PICO_RP2350
#include "pico.h"
#endif

#include "varmulet_macros.inc.S"

.macro j_possibly_short sym
#if VARMULET_FORCE_SHORT_JUMPS_FROM_DEFAULT_HOOKS
    .reloc ., R_RISCV_RVC_JUMP, \sym
.hword 0xa001
#else
    j \sym
#endif
.endm

.global varmulet_default_asm_hooks
.global varmulet_halt

// for use in static asm_hooks compiled against the code
.macro public_hook name
.global \name
.word   \name
.endm

code_label_with_section varmulet_halt
    ebreak
    j           varmulet_halt

.section .rodata.varmulet_default_asm_hooks
.p2align 2
varmulet_default_asm_hooks:
    public_hook varmulet_hook_default_enter_fn
    public_hook varmulet_hook_default_exit_fn
    public_hook varmulet_hook_default_save_regs_fn
    public_hook varmulet_hook_default_restore_regs_fn
#if !VARMULET_USE_ENTER_HOOK_TO_OVERRIDE_REGISTER_STORED_HOOKS
    public_hook varmulet_hook_default_execute_instruction
    public_hook varmulet_main_decode_table
    public_hook varmulet_dp_decode_table
#endif
    .word       varmulet_halt                    // undefined16
    .word       varmulet_halt                    // undefined32
    public_hook varmulet_hook_default_bkpt_instr
    public_hook varmulet_hook_default_hint_instr
    .word       varmulet_halt // public_hook varmulet_hook_default_svc_instr
    public_hook varmulet_hook_default_cps_instr
    public_hook varmulet_hook_default_mrs_instr
    public_hook varmulet_hook_default_msr_instr
    public_hook varmulet_hook_default_misc_control_instr
    public_hook varmulet_hook_default_update_primask_fn
    public_hook varmulet_hook_default_exc_return
#if ARMULET_FEATURE_CALL
    public_hook varmulet_hook_default_call_return
#else
    .word       varmulet_halt
#endif

code_label_with_section varmulet_hook_default_save_regs_fn
.global varmulet_save_a0_a7_t0_t5
varmulet_save_a0_a7_t0_t5:
    addi        sp, sp, -56
    sw          a0, 0(sp)
    sw          a1, 4(sp)
    sw          a2, 8(sp)
    sw          a3, 12(sp)
    sw          a4, 16(sp)
    sw          a5, 20(sp)
    sw          a6, 24(sp)
    sw          a7, 28(sp)
    sw          t0, 32(sp)
    sw          t1, 36(sp)
    sw          t2, 40(sp)
    sw          t3, 44(sp)
    sw          t4, 48(sp)
    sw          t5, 52(sp)
    // note we don't save/restore t6 which is r_asm_hooks because we need it to call this method so it
    // is saved after this function
    // sw          t6, 56(sp)
    ret

code_label_with_section  varmulet_hook_default_restore_regs_fn
.global varmulet_restore_a0_a7_t0_t5
varmulet_restore_a0_a7_t0_t5:
    lw          a0, 0(sp)
    lw          a1, 4(sp)
    lw          a2, 8(sp)
    lw          a3, 12(sp)
    lw          a4, 16(sp)
    lw          a5, 20(sp)
    lw          a6, 24(sp)
    lw          a7, 28(sp)
    lw          t0, 32(sp)
    lw          t1, 36(sp)
    lw          t2, 40(sp)
    lw          t3, 44(sp)
    lw          t4, 48(sp)
    lw          t5, 52(sp)
    // note we don't save/restore t6 which is r_asm_hooks because we need it to call this method so it
    // is restored before this function
    //lw          t6, 56(sp)
    addi        sp, sp, 56
    ret

.macro la_codeaddr rd, sym
    // lui is expected to be 16-bit if we're using 16-bit pointers, so this should be smaller
    // (note `la` expands to `auipc; addi` even with medlow code model)
#if VARMULET_USE_HWORD_MAIN_DECODE && defined(VARMULET_HWORD_MAIN_DECODE_ABSOLUTE_ADDR_OFFSET)
    lui \rd, %hi(\sym)
    addi \rd, \rd, %lo(\sym)
#else
    la \rd, \sym
#endif
.endm

code_label_with_section varmulet_hook_default_enter_fn
#if __riscv_zcmp
    cm.push     {ra, s0-s2}, -16
#else
    addi        sp, sp, -12
    sw          s0, (sp)
    sw          s1, 4(sp)
    sw          s2, 8(sp)
#endif
#if VARMULET_USE_WATERMARK
    li          r_watermark, 0xe0000000
#endif
    li          r_f0000000, 0xf0000000
#if !VARMULET_USE_ENTER_HOOK_TO_OVERRIDE_REGISTER_STORED_HOOKS
#if ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
#error ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION not supported without VARMULET_USE_ENTER_HOOK_TO_OVERRIDE_REGISTER_STORED_HOOKS
#endif
    lw          r_main_decode, (4*VASM_HOOKS_INDEX_MAIN_DECODE_TABLE)(r_asm_hooks)
    lw          r_dp_decode, (4*VASM_HOOKS_INDEX_DP_DECODE_TABLE)(r_asm_hooks)
    lw          r_next_inst, (4*VASM_HOOKS_INDEX_NEXT_INSTRUCTION)(r_asm_hooks)
#else
    la_codeaddr r_main_decode, varmulet_main_decode_table
    la_codeaddr r_dp_decode, varmulet_dp_decode_table
    la_codeaddr r_next_inst, varmulet_hook_default_execute_instruction
#endif
    lb          r_c, CPU_OFFSET_C(r_cpu)
    lw          r_pc, CPU_OFFSET_PC(r_cpu)
    lb          r_topbit_v, CPU_OFFSET_V(r_cpu)
    slli        r_topbit_v, r_topbit_v, 31
    lw          r_lazy_nz, CPU_OFFSET_LAZY_NZ(r_cpu)
    ret

code_label_with_section varmulet_hook_default_exit_fn
    sw          r_pc, CPU_OFFSET_PC(r_cpu)
    sb          r_c, CPU_OFFSET_C(r_cpu)
    sw          r_lazy_nz, CPU_OFFSET_LAZY_NZ(r_cpu)
    srli        r_tmp0, r_topbit_v, 31
    sb          r_tmp0, CPU_OFFSET_V(r_cpu)
#if __riscv_zcmp
    sw          ra, 0(sp) // preserve ra on pop
    cm.pop      {ra, s0-s2}, +16
#else
    lw          s0, (sp)
    lw          s1, 4(sp)
    lw          s2, 8(sp)
    addi        sp, sp, 12
#endif
    ret

code_label_with_section varmulet_hook_default_execute_instruction
.p2align 2
#if !VARMULET_USE_EARLY_INSTR_READ
    // don't think there is any benefit in checking here - if people care they can replace this loop
    read_mem_u16_unchecked r_inst, r_pc
    add         r_pc, r_pc, 2
#endif
#if ARMULET_FEATURE_PROFILER
    la          r_tmp1, address_profile
    slli        r_tmp0, r_pc, 17
    srli        r_tmp0, r_tmp0, 16
    add         r_tmp1, r_tmp1, r_tmp0
    lw          r_tmp0, (r_tmp1)
    addi        r_tmp0, r_tmp0, 1
    sw          r_tmp0, (r_tmp1)
#endif
    // get handler address based on the top 6 bits
    srli        r_tmp0, r_inst, 10
#if VARMULET_USE_HWORD_MAIN_DECODE
    sh1add      r_tmp1, r_tmp0, r_main_decode
    lhu         r_tmp1, (r_tmp1)
#if !defined(VARMULET_HWORD_MAIN_DECODE_ABSOLUTE_ADDR_OFFSET)
    // 16-bit table entry is relative address
    add         r_tmp1, r_tmp1, r_main_decode
#endif
#else
    sh2add      r_tmp1, r_tmp0, r_main_decode
    lw          r_tmp1, (r_tmp1)
#endif
#if VARMULET_USE_EARLY_INSTR_READ
    add         r_pc, r_pc, 2
#else
    // Hoisted from rlo_ptr_2_0 to save a cycle in some common instructions (plus some code size)
    andi        r_work0, r_inst, 0x7
#endif
    // jump to handler
    jr          r_tmp1

code_label_with_section varmulet_hook_default_bkpt_instr
    ebreak
    // fall thru
varmulet_hook_default_hint_instr: // hint defaults to nop
    next_instruction

code_label_with_section varmulet_hook_default_cps_instr
    andi        r_tmp0, r_inst, 15
    li          r_tmp1, 2
    beq         r_tmp0, r_tmp1, 1f
    j_possibly_short vexecute_undefined
1:
    bexti       r_work0, r_inst, 4
    call_hook_update_primask_fn_work0_trash_tmp2_work1
    next_instruction

code_label_with_section varmulet_hook_default_update_primask_fn
    sb          r_work0, CPU_OFFSET_PRIMASK(r_cpu)
    ret

#if 0
code_label_with_section varmulet_hook_default_svc_instr
    call_hook_save_regs_fn
    addi        sp, sp, -4
    sw          r_cpu, 0(sp)
    mv          a4, r_cpu // not a0-a4
    zext.b      a0, r_inst
    lw          a1, 0(a4) // r0
    lw          a2, 4(a4) // r1
    lw          a3, 8(a4) // r2
    lw          a5, CPU_OFFSET_SVC_HANDLER(a4)
    beqz        a5, varmulet_halt
1:
    jalr        a5
    lw          a4, 0(sp)
    addi        sp, sp, 4
    sw          a0, 0(a4) // r0
    call_hook_restore_regs_fn
    next_instruction
#endif

code_label_with_section varmulet_hook_default_mrs_instr
    // uint32_t rd = (op16_2 >> 8u) & 0xfu;
    h3.bextmi   r_tmp0, r_work2, 8, 4
    sh2add      r_work0, r_tmp0, r_cpu
    // uint32_t SYSm = op16_2 & 0xffu;
    zext.b      r_work1, r_work2
    addi        r_work1, r_work1, -0b00001000
    bltz        r_work1, vexecute_mrs_psr     // < 0b00001000
    beqz        r_work1, vexecute_mrs_msp     // == 0b00001000
    addi        r_work1, r_work1, 0b00001000 - 0b00001010
#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
    beqz        r_work1, v_execute_mrs_msplim // == 0b00001010
#endif
    addi        r_work1, r_work1, 0b00001010 - 0b00010000
    beqz        r_work1, vexecute_mrs_primask // == 0b00010000
    addi        r_work1, r_work1, 0b00010000 - 0b00010100
    // manual branch relaxation to allow rvc branch
    beqz        r_work1, 1f // == 0b00010100
    j_possibly_short vexecute32_undefined
1:
    // would be `zero`, but using known-zero r_work1 gets us a 16-bit encoding:
    sw          r_work1, (r_work0)
    next_instruction

vexecute_mrs_msp:
    lw          r_tmp0, CPU_OFFSET_SP(r_cpu)
    j           mrs_finish
#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
v_execute_mrs_msplim:
    lw          r_tmp0, CPU_OFFSET_SPLIM(r_cpu)
    j           mrs_finish
#endif
vexecute_mrs_primask:
    lb          r_tmp0, CPU_OFFSET_PRIMASK(r_cpu)

mrs_finish:
    // note storing to SP, PC is unpredictable
    sw          r_tmp0, (r_work0)
    next_instruction

vexecute_mrs_psr:
    li          r_tmp0, 0
    andi        r_tmp1, r_work1, 1
    beqz        r_tmp1, 1f
    lb          r_tmp0, CPU_OFFSET_IPSR(r_cpu)
1:
    andi        r_tmp1, r_work1, 4
    bnez        r_tmp1, mrs_finish
    flags_to_apsr r_work1, r_tmp1, r_tmp2
    or          r_tmp0, r_tmp0, r_work1
    j           mrs_finish

code_label_with_section varmulet_hook_default_msr_instr
    // uint32_t rd = op16 & 0xfu;
    andi        r_tmp0, r_inst, 15
    sh2add      r_tmp0, r_tmp0, r_cpu
    lw          r_tmp0, (r_tmp0)
    // uint32_t SYSm = op16_2 & 0xffu;
    zext.b      r_work1, r_work2
    addi        r_work1, r_work1, -0b00001000
    bltz        r_work1, vexecute_msr_psr     // < 0b00001000
    beqz        r_work1, vexecute_msr_msp     // == 0b00001000
    addi        r_work1, r_work1, 0b00001000 - 0b00001010
#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
    beqz        r_work1, vexecute_msr_msplim  // == 0b00001010
#endif
    addi        r_work1, r_work1, 0b00001010 - 0b00010000
    beqz        r_work1, vexecute_msr_primask // == 0b00010000
    addi        r_work1, r_work1, 0b00010000 - 0b00010100
    beqz        r_work1, 1f                   // == 0b00010100 (nop - control)
    j_possibly_short vexecute32_undefined
vexecute_msr_msp:
    andi        r_tmp0, r_tmp0, ~3
    sw          r_tmp0, CPU_OFFSET_SP(r_cpu)
1:
    next_instruction

vexecute_msr_msplim:
    sw          r_tmp0, CPU_OFFSET_SPLIM(r_cpu)
    next_instruction

vexecute_msr_primask:
    andi        r_work0, r_tmp0, 1
    call_hook_update_primask_fn_work0_trash_tmp2_work1
    next_instruction

vexecute_msr_psr:
    andi        r_tmp1, r_work1, 4
    bnez        r_tmp1, 1b
    apsr_to_flags r_tmp0, r_tmp2
    next_instruction

code_label_with_section varmulet_hook_default_misc_control_instr
    h3.bextmi   r_tmp0, r_work2, 4, 4
    addi        r_tmp0, r_tmp0, -4
    li          r_tmp1, 3
    bltu        r_tmp0, r_tmp1, 1f
    j_possibly_short vexecute32_undefined
1:
    // dsb, dmb, isb
    next_instruction

#if ARMULET_FEATURE_CALL
code_label_with_section varmulet_hook_default_call_return
    lw          r_tmp0, (sp)
    addi        sp, sp, 4
    jr          r_tmp0
#endif

code_label_with_section varmulet_hook_default_exc_return
#if ARMULET_FEATURE_CALL
    li          r_tmp0, ARMULET_CALL_RETURN_ADDRESS
    beq         r_tmp0, r_work0, varmulet_do_call_return
#endif
    ebreak
    next_instruction

#if ARMULET_FEATURE_CALL
varmulet_do_call_return:
    tail_call_asm_hook_trash_tmp2_work1 VASM_HOOKS_INDEX_CALL_RETURN
#endif
