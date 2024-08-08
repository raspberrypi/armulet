#include "varmulet.h"

#if !ARMULET_USE_LAZY_NZ || ARMULET_USE_LAZY_Z
#error varmulet requires ARMULET_USE_LAZY_NZ and not ARMULET_USE_LAZY_Z
#endif


// ===================================================
//
// s0
// s1
//
// a0
// a1
// a2
// a3
// a4
// a5
// a6
// a7

// t1
// t2
// t3
// t4
// t5
// t6
// t7

// r_tmp0->r_tmp2 are for local temproaries (they are not commonly set in setup code common to multiple instruction implementations)
// r_work0->r_work2 are also temporary, but are used also for pre-instruction setup values (often register pointers/values)

#define r_tmp0          s0
#define r_tmp1          s1
#define r_f0000000      s2

#define r_pc            a0 // ARM pc points at following instruction during instruction execution
#define r_tmp2          a1 // note this is used as a temporary in function enter/exit so needs to be callee saved (and not any of the things we use below)
#define r_work0         a2
#define r_work1         a3 // note this is used as a temporary in function enter/exit so needs to be callee saved (and not any of the things we use below)
#define r_lazy_nz       a4
#define r_inst          a5 // the current 16 bit instruction
#define r_next_inst     a6 // need to jump to this after each instruction
#define r_c             a7 // the carry 1/0

#define r_main_decode   t1 // main decoder table
#define r_cpu           t2
#define r_work2         t3

#define r_topbit_v      t4 // 0b Vxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx
#define r_dp_decode     t5
// this is the last one by definition because of save/restore regs (we must save it separately,
// and don't want to leave a gap in registers the hook saves/restores)
#define r_asm_hooks     t6

#if VARMULET_USE_WATERMARK
#error todo need assign a r_watermark register for this
#endif

// ----------------------------------------------------------------------------
// Hazard3 custom instructions

// rd = (rs1 >> rs2[4:0]) & ~(-1 << nbits)
.macro h3.bextm rd rs1 rs2 nbits
.if (\nbits < 1) || (\nbits > 8)
.err
.endif
#if VARMULET_USE_HAZARD3_CUSTOM
    .insn r 0x0b, 0x4, (((\nbits - 1) & 0x7 ) << 1), \rd, \rs1, \rs2
#else
    srl  \rd, \rs1, \rs2
    andi \rd, \rd, ((1 << \nbits) - 1)
#endif
.endm

// rd = (rs1 >> shamt) & ~(-1 << nbits)
.macro h3.bextmi rd rs1 shamt nbits
.if (\nbits < 1) || (\nbits > 8)
.err
.endif
.if (\shamt < 0) || (\shamt > 31)
.err
.endif
#if VARMULET_USE_HAZARD3_CUSTOM
    .insn i 0x0b, 0x4, \rd, \rs1, (\shamt & 0x1f) | (((\nbits - 1) & 0x7 ) << 6)
#else
    srli \rd, \rs1, \shamt
    andi \rd, \rd, ((1 << \nbits) - 1)
#endif
.endm

// ----------------------------------------------------------------------------

.macro code_label_with_section l
.section .text.\l
\l:
.endm

// ----------------------------------------------------------------------------
// MEMORY access macros for the "ARM" address space
// ----------------------------------------------------------------------------

// direct use of _unchecked macro means should only be for memory access that aren't done on behalf of an
// ARM load/store/push/pop instruction (other than instruction fetch); for everything else use the non-"_unchecked"
// macros which use fixed address and data RISC-V registers
.macro read_mem_u16_unchecked target_reg, address_reg
    lhu     \target_reg, 0(\address_reg)
.endm

.macro read_mem_32_unchecked target_reg, address_reg
    lw      \target_reg, 0(\address_reg)
.endm

.macro write_mem_32_unchecked out_reg, address_reg
    sw      \out_reg, (\address_reg)
.endm

// ----------------------------------------------------------------------------
// These remaining memory access macros have ARM address in r_work1 and data to/from r_tmp0
// this allos read/writeN_special to know what registers to use when watermarking, and also potentially
// makes it easy for an exception handler to virtualize an ARM address range based on memory protection
.macro read_mem_u8_at_work1_to_tmp0
#if VARMULET_USE_WATERMARK
    bgeu    r_work1, r_watermark, read8_special
#endif
    lbu     r_tmp0, (r_work1)
.endm

.macro read_mem_s8_at_work1_to_tmp0
#if VARMULET_USE_WATERMARK
    bgeu    r_work1, r_watermark, read8_special
#endif
    lb      r_tmp0, (r_work1)
.endm

.macro read_mem_u16_at_work1_to_tmp0
#if VARMULET_USE_WATERMARK
    bgeu    r_work1, r_watermark, read16_special
#endif
    lhu     r_tmp0, (r_work1)
.endm

.macro read_mem_s16_at_work1_to_tmp0
#if VARMULET_USE_WATERMARK
    bgeu    r_work1, r_watermark, read16_special
#endif
    lh      r_tmp0, (r_work1)
.endm

.macro read_mem_32_at_work1_to_tmp0
#if VARMULET_USE_WATERMARK
    bgeu    r_work1, r_watermark, read32_special
#endif
    lw      r_tmp0, (r_work1)
.endm

.macro read_stack_32_at_work1_to_tmp0
#if ASSUME_STACK_SAFE
    read_u32_no_check r_tmp0, r_work1
#else
    read_mem_32_at_work1_to_tmp0
#endif
.endm

.macro write_mem_32_at_work1_from_tmp0
#if VARMULET_USE_WATERMARK
    bgeu    r_work1, r_watermark, write32_special
#endif
    sw      r_tmp0, (r_work1)
.endm

.macro write_stack_32_at_work1_from_tmp0
#if ASSUME_STACK_SAFE
    write_mem_32_unchecked r_tmp0, r_work1
#else
    write_mem_32_at_work1_from_tmp0
#endif
.endm

.macro write_mem_16_at_work1_from_tmp0
#if VARMULET_USE_WATERMARK
    bgeu    r_work1, r_watermark, write16_special
#endif
    sh      r_tmp0, (r_work1)
.endm

.macro write_mem_8_at_work1_from_tmp0
#if VARMULET_USE_WATERMARK
    bgeu    r_work1, r_watermark, write8_special
#endif
    sb      r_tmp0, (r_work1)
.endm

.macro rlo_ptr_10_8 reg
    h3.bextmi   \reg, r_inst, 8, 3
    sh2add      \reg, \reg, r_cpu
.endm

.macro rlo_ptr_8_6 reg
    h3.bextmi    \reg, r_inst, 6, 3
    sh2add       \reg, \reg, r_cpu
.endm


.macro rlo_ptr_2_0_should_use_a2_not_a2
.endm
.macro rlo_ptr_2_0 reg
#if !VARMULET_USE_EARLY_INSTR_READ
    // Assert that only r_work0 (assumed to be a2) is used, to match the
    // hoisted `andi` in varmulet_hook_default_execute_instruction:
    rlo_ptr_2_0_should_use_a2_not_\reg
    // (note this assertion will need updating if you reallocate r_work0)
#else
    andi        \reg, r_inst, 0x7
#endif
    sh2add      \reg, \reg, r_cpu
.endm

.macro rlo_ptr_5_3 reg
    h3.bextmi      \reg, r_inst, 3, 3
    sh2add         \reg, \reg, r_cpu
.endm

#if ARMULET_FEATURE_ARMV8M_BASELINE
.macro r_ptr32_11_8 reg
    h3.bextmi      \reg, r_work2, 8, 4
    sh2add         \reg, \reg, r_cpu
.endm

.macro r_ptr32_3_0 reg
    andi           \reg, r_work2, 0xf
    sh2add         \reg, \reg, r_cpu
.endm

.macro r_ptr32_19_16 reg
    andi           \reg, r_inst, 0xf
    sh2add         \reg, \reg, r_cpu
.endm
#endif

// todo make a function
// out and reg can be the same
.macro r_lo_hi_value out, reg, temp1, temp2
    sh2add      \temp1, \reg, r_cpu
    addi        \temp2, \reg, -15
    lw          \out, (\temp1)
    // Do not use \reg now that \out is trashed
    bnez        \temp2, 1f
    addi        \out, r_pc, 2
1:
.endm

.macro check_splim value, temp
#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
    lw          \temp, CPU_OFFSET_SPLIM(r_cpu)
#if 1
    bltu        \value, \temp, vexecute_dummy_bkpt
#else
    bgeu        \value, \temp, 6f
    ebreak
    j vexecute_dummy_bkpt
6:
#endif
#endif

.endm
.macro w_lo_hi_value_sp_pc reg, value, temp1, temp2
    sh2add      \temp1, \reg, r_cpu
    addi        \temp2, \reg, -13
    bltz        \temp2, 8f
    beqz        \temp2, 7f
    addi        \temp2, \temp2, -2
    bnez        \temp2, 8f
    andi        r_pc, \value, ~1
    j           9f
7:
    andi        \value, \value, ~3
    sw          \value, (\temp1)
    check_splim \value, \temp2
8:
    sw          \value, (\temp1)
9:
.endm

.macro get_va_imm5_ra_rb out, a, b, imm
    rlo_ptr_5_3 \a
    rlo_ptr_2_0 \b
    lw          \out, (\a)
    h3.bextmi   \imm, r_inst, 6, 5
.endm

.macro get_z reg
    seqz        \reg, r_lazy_nz
.endm

.macro get_not_z reg
    snez        \reg, r_lazy_nz
.endm

.macro get_n reg
    sltz        \reg, r_lazy_nz
.endm

.macro get_c reg
    mv          \reg, r_c
.endm

.macro get_v reg
    srli    \reg, r_topbit_v, 31
.endm

.macro flags_to_apsr apsr, tmpa, tmpb
    get_N       \tmpa
    get_Z       \tmpb
    sh1add      \apsr, \tmpa, \tmpb
    get_V       \tmpb
    sh1add      \tmpa, r_c, \tmpb
    sh2add      \apsr, \apsr, \tmpa
    slli        \apsr, \apsr, 28
.endm

.macro apsr_to_flags apsr, tmp
    // N Z LNZ
    // -------
    // 0 0 1
    // 0 1 0
    // 1 0 -1
    // 1 1 -2 (not ideal, but we don't have a valid answer)
    srli        r_lazy_nz, \apsr, 30
    li          \tmp, 1
    sub         r_lazy_nz, \tmp, r_lazy_nz

    sll         r_topbit_v, \apsr, 3
    bexti       r_c, \apsr, 29
.endm

.macro next_instruction
#if VARMULET_USE_EARLY_INSTR_READ
    read_mem_u16_unchecked r_inst, r_pc
#endif
    jr      r_next_inst
.endm

// have this in case we want to replace with a single jmp to common code
.macro store_nz_at_work0_next_instruction
    sw      r_lazy_nz, (r_work0)
    next_instruction
.endm

.macro add_update_flags r_a r_b
    add     r_lazy_nz, \r_a, \r_b
    // calulate v
    xor     r_topbit_v, r_lazy_nz, \r_a
    xnor    r_c, \r_a, \r_b
    and     r_topbit_v, r_topbit_v, r_c
    // calulate c
    sltu    r_c, r_lazy_nz, \r_a
.endm

.macro sub_update_flags r_a r_b
    sub     r_lazy_nz, \r_a, \r_b
    // calulate v
    xor     r_topbit_v, \r_a, r_lazy_nz
    xnor    r_c, r_lazy_nz, \r_b
    and     r_topbit_v, r_topbit_v, r_c
    // calculate c
    sltu    r_c, \r_a, \r_b
    xori    r_c, r_c, 1
.endm

.macro call_asm_hook_fn_trash_tmp2_work1 FN_INDEX
#if !ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
    lw          r_tmp2, (4 * \FN_INDEX)(r_asm_hooks)
    jalr        r_tmp2
#else
    li          r_tmp2, \FN_INDEX
    jalr        r_asm_hooks
#endif
.endm

.macro tail_call_asm_hook_trash_tmp2_work1 HOOK_INDEX
#if !ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
    lw          r_tmp2, (4 * \HOOK_INDEX)(r_asm_hooks)
    jr          r_tmp2
#else
    li          r_tmp2, \HOOK_INDEX
    jalr        r_asm_hooks
#endif
.endm

.macro tail_call_asm_hook_in_tmp2_trash_work1
#if !ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
    sh2add      r_tmp2, r_tmp2, r_asm_hooks
    lw          r_tmp2, (r_tmp2)
    jr          r_tmp2
#else
    jr          r_asm_hooks
#endif
.endm

.macro call_hook_enter_fn_trash_tmp2_work1
    mv          r_cpu, a0
    mv          r_asm_hooks, a1
    // use tmp2 (a1) since we know it is unused atm
    call_asm_hook_fn_trash_tmp2_work1 VASM_HOOKS_INDEX_ENTER_FN
.endm

.macro call_hook_exit_fn_trash_tmp2_work1
    // use tmp2 (a1) since we know we don't care about it's value (we might about t0)
    call_asm_hook_fn_trash_tmp2_work1 VASM_HOOKS_INDEX_EXIT_FN
.endm

.macro call_hook_save_regs_fn_trash_tmp2_work1
    call_asm_hook_fn_trash_tmp0_work1 VASM_HOOKS_INDEX_SAVE_REGS_FN
    // need to store r_asm_hooks at the end of the stack, so we can pop it before calling restore_regs_fn
    addi        sp, sp, -4
    sw          r_asm_hooks, (sp)
.endm

.macro call_hook_restore_regs_fn_trash_tmp2_work1
    lw          r_asm_hooks, (sp)
    addi        sp, sp, 4
    call_asm_hook_trash_tmp2_work1 VASM_HOOKS_INDEX_RESTORE_REGS_FN
.endm

.macro call_hook_update_primask_fn_work0_trash_tmp2_work1
    call_asm_hook_fn_trash_tmp2_work1 VASM_HOOKS_INDEX_UPDATE_PRIMASK_FN
.endm

.macro jmp_hook_undefined16_trash_tmp2_work1
    tail_call_asm_hook_trash_tmp2_work1 VASM_HOOKS_INDEX_UNDEFINED16
.endm

.macro jmp_hook_undefined32_trash_tmp2_work1
    tail_call_asm_hook_trash_tmp2_work1 VASM_HOOKS_INDEX_UNDEFINED32
.endm

.macro jmp_hook_exc_return_trash_tmp2_work1
    tail_call_asm_hook_trash_tmp2_work1 VASM_HOOKS_INDEX_EXC_RETURN
.endm

.macro check_exc_return_to_work0_trash_tmp2_work1
    bgeu r_work0, r_f0000000, do_jmp_hook_exc_return
.endm

.macro check_exc_return_to_tmp0_trash_tmp2_work1
    bgeu r_tmp0, r_f0000000, do_jmp_hook_exc_return_tmp0
.endm
