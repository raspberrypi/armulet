/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if PICO_RP2350
#include "pico.h"
#endif

#include "varmulet_macros.inc.S"

// speed vs space
.macro main_decode_align
.p2align 2
.endm

.global varmulet_main_decode_table
.global varmulet_dp_decode_table
.global vexecute_undefined // saves duplicating it elsewhere
.global vexecute32_undefined // saves duplicating it elsewhere
.global do_jmp_hook_exc_return
.global do_jmp_hook_exc_return_tmp0

#if VARMULET_USE_HWORD_MAIN_DECODE
.section .text.varmulet_armv6m_core
.macro main_decode_entry sym
#if VARMULET_HWORD_MAIN_DECODE_ABSOLUTE_ADDR_OFFSET
// Workaround for lack of 16-bit data relocations: put decode table at known
// address so that we can generate the pointers using assembler arithmetic.
.hword \sym - varmulet_main_decode_table + VARMULET_HWORD_MAIN_DECODE_ABSOLUTE_ADDR_OFFSET
#else
.hword \sym - varmulet_main_decode_table
#endif
.endm
#else
// Regular 32-bit function pointer with data relocation
.section .rodata.varmulet_main_decode_table
.macro main_decode_entry sym
.word \sym
.endm
#endif

.macro table_branch name
    jal v_table_branch
.set \name\()_table_base,.
.endm

.macro table_branch_entry name,sym
.byte \sym-\name\()_table_base
.endm

.macro enter_asm_hook_shared_if_smaller
#if ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
    // in this case should be a single 16-bit instruction
    tail_call_asm_hook_in_tmp2_trash_work1
#else
    j shared_enter_asm_hook
#endif
.endm

.p2align 2
varmulet_main_decode_table:
    // Shift (immediate), add, subtract, move, and compare
    main_decode_entry vexecute_lslsi        // 0000 00
    main_decode_entry vexecute_lslsi        // 0000 01
    main_decode_entry vexecute_lsrsi        // 0000 10
    main_decode_entry vexecute_lsrsi        // 0000 11
    main_decode_entry vexecute_asrsi        // 0001 00
    main_decode_entry vexecute_asrsi        // 0001 01
    main_decode_entry vexecute_adds_subs    // 0001 10
    main_decode_entry vexecute_addsi_subsi  // 0001 11
    main_decode_entry vexecute_movsi        // 0010 00
    main_decode_entry vexecute_movsi        // 0010 01
    main_decode_entry vexecute_cmpi         // 0010 10
    main_decode_entry vexecute_cmpi         // 0010 11
    main_decode_entry vexecute_adds8i       // 0011 00
    main_decode_entry vexecute_adds8i       // 0011 01
    main_decode_entry vexecute_subs8i       // 0011 10
    main_decode_entry vexecute_subs8i       // 0011 11
    // Data processing
    main_decode_entry vexecute_dp           // 0100 00
    // Special data instructions and branch and exchange
    main_decode_entry vexecute_010001       // 0100 01
    // Load from Literal Pool
    main_decode_entry vexecute_ldr_lit      // 0100 10
    main_decode_entry vexecute_ldr_lit      // 0100 11
    // Load/store single data item
    main_decode_entry vexecute_010100       // 0101 00 // str, strh
    main_decode_entry vexecute_010101       // 0101 01 // strb ldrsb
    main_decode_entry vexecute_010110       // 0101 10 // ldr, ldrh
    main_decode_entry vexecute_010111       // 0101 11 // ldrb, lrdsh

    main_decode_entry vexecute_stri         // 0110 00
    main_decode_entry vexecute_stri         // 0110 01
    main_decode_entry vexecute_ldri         // 0110 10
    main_decode_entry vexecute_ldri         // 0110 11

    main_decode_entry vexecute_strbi        // 0111 00
    main_decode_entry vexecute_strbi        // 0111 01
    main_decode_entry vexecute_ldrbi        // 0111 10
    main_decode_entry vexecute_ldrbi        // 0111 11

    main_decode_entry vexecute_strhi        // 1000 00
    main_decode_entry vexecute_strhi        // 1000 01
    main_decode_entry vexecute_ldrhi        // 1000 10
    main_decode_entry vexecute_ldrhi        // 1000 11

    main_decode_entry vexecute_strspi       // 1001 00
    main_decode_entry vexecute_strspi       // 1001 01
    main_decode_entry vexecute_ldrspi       // 1001 10
    main_decode_entry vexecute_ldrspi       // 1001 11

    // Generate PC-relative address
    main_decode_entry vexecute_adr          // 1010 00
    main_decode_entry vexecute_adr          // 1010 01

    // Generate SP-relative address
    main_decode_entry vexecute_add_sp       // 1010 10
    main_decode_entry vexecute_add_sp       // 1010 11

    // Miscellaneous 16-bit instructions
    main_decode_entry vexecute_101100       // 1011 00
    main_decode_entry vexecute_101101       // 1011 01
    main_decode_entry vexecute_101110       // 1011 10
    main_decode_entry vexecute_101111       // 1011 11

    // Store multiple registers
    main_decode_entry vexecute_stm          // 1100 00
    main_decode_entry vexecute_stm          // 1100 01

    // Load multiple registers
    main_decode_entry vexecute_ldm          // 1101 00
    main_decode_entry vexecute_ldm          // 1101 01

    // Conditional branch, and Supervisor Call
    main_decode_entry vexecute_1101xx       // 1101 00
    main_decode_entry vexecute_1101xx       // 1101 01
    main_decode_entry vexecute_1101xx       // 1101 10
    main_decode_entry vexecute_1101xx       // 1101 11

    // Unconditional Branch
    main_decode_entry vexecute_branch       // 1110 00
    main_decode_entry vexecute_branch       // 1110 01

    // 32 bit instructions from here on

    main_decode_entry vexecute32_prefix_undefined    // 1110 10
    main_decode_entry vexecute32_prefix_undefined    // 1110 11

    main_decode_entry vexecute32_11110  // 1111 00
    main_decode_entry vexecute32_11110  // 1111 01

#if ARMULET_FEATURE_ARMV8M_BASELINE_SDIV_UDIV
    main_decode_entry vexecute32_111110    // 1111 10
#else
    main_decode_entry vexecute32_prefix_undefined
#endif
    main_decode_entry vexecute32_prefix_undefined    // 1111 11
main_decode_end:

// These tables must be concatenated in order for the main_decode_entry
// address arithmetic to work with 16-bit pointers across multiple tables:
varmulet_dp_decode_table:
    main_decode_entry vdp_ands     // 0000
    main_decode_entry vdp_eors     // 0001
    main_decode_entry vdp_lsls     // 0010
    main_decode_entry vdp_lsrs     // 0011
    main_decode_entry vdp_asrs     // 0100
    main_decode_entry vdp_adcs     // 0101
    main_decode_entry vdp_sbcs     // 0110
    main_decode_entry vdp_rors     // 0111
    main_decode_entry vdp_tsts     // 1000
    main_decode_entry vdp_rsbs     // 1001
    main_decode_entry vdp_cmp      // 1010
    main_decode_entry vdp_cmn      // 1011
    main_decode_entry vdp_orrs     // 1100
    main_decode_entry vdp_muls     // 1101
    main_decode_entry vdp_bics     // 1110
    main_decode_entry vdp_mvns     // 1111

.section .text.varmulet_armv6m_core

v_table_branch:
    add         r_tmp0, r_tmp0, ra
    lbu         r_tmp0, (r_tmp0)
    bexti       r_work2, r_inst, 8 // spare cycle so calculate for vcb_bxx
    add         ra, ra, r_tmp0
    ret

// in some cases the source register is in tmp0, and it is better to move it to work0 once we've detected an exception!
do_jmp_hook_exc_return_tmp0:
    mv          r_work0, r_tmp0
do_jmp_hook_exc_return:
    jmp_hook_exc_return_trash_tmp2_work1

// === BEGINNING OF INSTRUCTIONS
// odd length
vexecute_asrsi: //  0001 0x
    jal         v_ldr_str_r2i_setup // work0=&Rd, work1=Rm, work2=immediate
v_asr_entry:
    addi        r_tmp1, r_work2, -1
    bext        r_c, r_work1, r_tmp1
    beqz        r_work2, 2f
1:
    sra         r_lazy_nz, r_work1, r_work2
    store_nz_at_work0_next_instruction
2:
    // immediate of 0 is shift by 32
    li          r_work2,31
    j 1b

// === Shift (immediate), add, subtract, move, and compare
main_decode_align
vexecute_lslsi: //  0000 0x
    get_va_imm5_ra_rb r_lazy_nz, r_work1, r_work0, r_tmp0 // lazy_nz=operand/result, work1=temp, work0=dest, tmp0=shift amount
v_lsl_entry:
    beqz        r_tmp0, 1f
    neg         r_tmp1, r_tmp0
    bext        r_c, r_lazy_nz, r_tmp1
    sll         r_lazy_nz, r_lazy_nz, r_tmp0
1:
    // immediate of 0 is movs rd, rm
    store_nz_at_work0_next_instruction

main_decode_align
vexecute_lsrsi: //  0000 1x
    get_va_imm5_ra_rb r_lazy_nz, r_work1, r_work0, r_tmp0
v_lsr_entry:
    addi        r_tmp1, r_tmp0, -1
    bext        r_c, r_lazy_nz, r_tmp1
    beqz        r_tmp0, 2f
    srl         r_lazy_nz, r_lazy_nz, r_tmp0
1:
    store_nz_at_work0_next_instruction
2:
    // immediate of 0 is shift by 32
    li          r_lazy_nz, 0
    j 1b

// static counts in ROM text:
//   3-reg adds:  168
//   3-reg subs:  62
//   2-reg addsi: 41
//   2-reg subsi: 24
//   1-reg addsi: 189
//   1-reg subsi: 58
// conclusion: 2-reg immediate instructions don't get the fast fall through

main_decode_align
vexecute_adds_subs: 	//  0001 10
    // adds/subs rd, rn, rm
    rlo_ptr_8_6 r_work2 // rm
    rlo_ptr_5_3 r_work1 // rn
    rlo_ptr_2_0 r_work0 // rd
    lw          r_tmp0, (r_work2) // tmp0 = regs[rm]
    lw          r_work2, (r_work1) // work2 = regs[rn]
    bexti       r_tmp1, r_inst, 9
    bnez        r_tmp1, vexecute_subs_common
vexecute_adds_common:
    add_update_flags r_work2, r_tmp0
    store_nz_at_work0_next_instruction

// .p2align 2 (uncommon) odd length
vdp_muls:
    // cpu->regs[rdn] *= cpu->regs[rm];
    mul         r_lazy_nz, r_work1, r_work2
    j vdp_shared_store_nz_at_work0_next_instruction

main_decode_align
vexecute_addsi_subsi: //  0001 11
    // adds/subs rd, rn, #imm3
    rlo_ptr_5_3 r_work1 // rn
    rlo_ptr_2_0 r_work0 // rd
    lw          r_work2, (r_work1) // work2 = regs[rn]
    h3.bextmi   r_tmp0, r_inst, 6, 3
    bexti       r_tmp1, r_inst, 9
    beqz        r_tmp1, vexecute_adds_common
    j vexecute_subs_common

main_decode_align
vexecute_movsi: //  0010 0x
    rlo_ptr_10_8 r_work0 // rd
    zext.b       r_lazy_nz, r_inst
    store_nz_at_work0_next_instruction

main_decode_align
vexecute_adds8i: //  0011 0x
    rlo_ptr_10_8 r_work0 // rdn
    lw          r_work2, (r_work0) // work2 = regs[rdn]
    // Use destructive zext.b to get 16-bit encoding:
    zext.b      r_inst, r_inst
    add_update_flags r_work2, r_inst
    store_nz_at_work0_next_instruction

// odd length
vdp_tsts:
    and     r_lazy_nz, r_work1, r_work2
    next_instruction

main_decode_align
vexecute_subs8i: //  0011 1x
    rlo_ptr_10_8 r_work0 // rdn
    lw          r_work2, (r_work0) // work2 = regs[rdn]
    zext.b      r_tmp0, r_inst
vexecute_subs_common:
    sub_update_flags r_work2, r_tmp0
    store_nz_at_work0_next_instruction

// === Data processing
main_decode_align
vexecute_dp: //  0100 00
    rlo_ptr_2_0 r_work0 // rdn
    rlo_ptr_5_3 r_work1 // rm
    h3.bextmi   r_tmp0, r_inst, 6, 4
#if VARMULET_USE_HWORD_MAIN_DECODE
    sh1add      r_tmp0, r_tmp0, r_dp_decode
    lhu         r_tmp0, (r_tmp0)
#if !defined(VARMULET_HWORD_MAIN_DECODE_ABSOLUTE_ADDR_OFFSET)
    // 16-bit table entry is relative to first main decode entry
    add         r_tmp0, r_tmp0, r_main_decode
#endif
#else
    sh2add      r_tmp0, r_tmp0, r_dp_decode
    lw          r_tmp0, (r_tmp0)
#endif
    lw          r_work2, (r_work1) // work2 = regs[rm]
    lw          r_work1, (r_work0) // work1 = regs[rdn]
    jr          r_tmp0

// .p2align 2 (uncommon)
// odd length, tuck it in here
vdp_rsbs:
    neg     r_lazy_nz,r_work2
    seqz    r_c,r_lazy_nz // C is set for 0 result, clear otherwise
    and     r_topbit_v,r_lazy_nz,r_work2 // V is set for 0x80000000 (only case where both argument and result are negative)
    j vdp_shared_store_nz_at_work0_next_instruction

.p2align 2
vdp_ands:
    // cpu->regs[rdn] &= cpu->regs[rm];
    and         r_lazy_nz, r_work1, r_work2
    store_nz_at_work0_next_instruction

.p2align 2
vdp_eors:
    // cpu->regs[rdn] ^= cpu->regs[rm];
    xor         r_lazy_nz, r_work1, r_work2
    store_nz_at_work0_next_instruction

// .p2align 2 // the mv is a compressed instruction
vdp_lsls:
    mv          r_lazy_nz,r_work1
    zext.b      r_tmp0, r_work2 // shift amount
    li          r_tmp1,32
    blt         r_tmp0,r_tmp1,v_lsl_entry
v_lsls_over_ret:
    slti        r_c,r_tmp0,33
    and         r_c,r_c,r_work1
    mv          r_lazy_nz,zero
    j           vdp_shared_store_nz_at_work0_next_instruction

// .p2align 2 // the mv is a compressed instruction
vdp_lsrs:
    mv          r_lazy_nz,r_work1
    zext.b      r_tmp0, r_work2 // shift amount
    beqz        r_tmp0,vdp_shared_store_nz_at_work0_next_instruction // zero shift
    li          r_tmp1,32
    blt         r_tmp0,r_tmp1,v_lsr_entry
    srli        r_work1,r_work1,31
    j           v_lsls_over_ret

// .p2align 2 // the mv is a compressed instruction
vdp_asrs:
    mv          r_lazy_nz,r_work1
    zext.b      r_work2, r_work2 // shift amount
    beqz        r_work2,vdp_shared_store_nz_at_work0_next_instruction // zero shift means zero for shift-by-register
    li          r_tmp1,31
    ble         r_work2,r_tmp1,1f
    li          r_work2,0 // a shift by >31 places is the same as a shift by 32, which is done by immediate shift operand of zero
1:
    j           v_asr_entry

.p2align 2
vdp_sbcs:
    not     r_work2, r_work2
    // fall thru
vdp_adcs:
    add         r_tmp0, r_work2, r_c        // tmp1(bc) = b + c
    add         r_lazy_nz, r_work1, r_tmp0  // result = a + bc

    // calculate C
    seqz        r_tmp1, r_tmp0              // !bc
    and         r_tmp1, r_tmp1, r_c
    sltu        r_tmp0, r_lazy_nz, r_work1
    or          r_c, r_tmp0, r_tmp1

    // calculate V
    xor         r_tmp0, r_lazy_nz, r_work2
    xnor        r_tmp1, r_work2, r_work1
    and         r_topbit_v, r_tmp0, r_tmp1

    store_nz_at_work0_next_instruction

// .p2align 2 (uncommon)
vdp_rors:
    zext.b      r_tmp0, r_work2
    ror         r_lazy_nz, r_work1, r_tmp0
    // carry is unchanged if 8-bit shift count is 0:
    beqz        r_tmp0, 1f
    srli        r_c, r_lazy_nz, 31
1:
    j vdp_shared_store_nz_at_work0_next_instruction

// .p2align 2 (uncommon)
vdp_cmn:
    addi    r_work0,r_cpu,CPU_OFFSET_LAZY_NZ // set result pointer to safe-to-write-to memory location !!! check
    mv      r_tmp0, r_work1
    j       vexecute_adds_common

main_decode_align
vexecute_cmpi: //  0010 1x
    rlo_ptr_10_8 r_work0 // rdn
#if 0
    // Maintain alignment to fall into vdp_cmp without an alignment nop
.option push
.option norvc
    lw          r_work1, (r_work0) // work2 = regs[rdn]
.option pop
#else
    // Seems the C vs Zca confusion has broken .option norvc -- this is the
    // same instruction as above but has the requested size.
    .insn i 0x03, 0x2, r_work1, r_work0, 0
#endif
    zext.b      r_work2, r_inst
    // fall through
vdp_cmp:
    // this is also entered from vexecute_add_cmp_mov_hi
    sub_update_flags r_work1, r_work2
    next_instruction

// .p2align 2 (uncommon)
vdp_mvns:
    // cpu->regs[rdn] = ~cpu->regs[rm];
    not         r_lazy_nz, r_work2
    j vdp_shared_store_nz_at_work0_next_instruction

.p2align 2
vdp_orrs:
    // cpu->regs[rdn] |= cpu->regs[rm];
    or          r_lazy_nz, r_work1, r_work2
    // uncommon instructions share this tail:
vdp_shared_store_nz_at_work0_next_instruction:
    store_nz_at_work0_next_instruction

.p2align 2
vdp_bics:
    // cpu->regs[rdn] &= ~cpu->regs[rm];
    andn        r_lazy_nz, r_work1, r_work2
    store_nz_at_work0_next_instruction


// === Special data instructions and branch and exchange
main_decode_align
vexecute_010001: //  0100 01
    h3.bextmi   r_tmp0, r_inst, 8, 2
    addi        r_tmp0, r_tmp0, -3
    beqz        r_tmp0, vexecute_010001_11

vexecute_add_cmp_mov_hi:
    // uint32_t rdn = (op16 & 0x7u) + ((op16 >> 4u & 0x8u));
    bexti       r_tmp0, r_inst, 7
    andi        r_tmp1, r_inst, 7
    sh3add      r_work0, r_tmp0, r_tmp1
    // uint32_t rm = (op16 >> 3u) & 0xfu;
    h3.bextmi   r_work1, r_inst, 3, 4

    // uint32_t vm = get_lo_hi_reg(cpu, rm);
    r_lo_hi_value r_work2, r_work1, r_tmp0, r_tmp1
    // mov only requires value of rm
    bexti       r_tmp0, r_inst, 9
    bnez        r_tmp0, vexecute_mov_hi
    // uint32_t vdn = get_lo_hi_reg(cpu, rdn);
    r_lo_hi_value r_work1, r_work0, r_tmp0, r_tmp1

    bexti       r_tmp0, r_inst, 8
    bnez        r_tmp0, vdp_cmp
vexecute_add_hi:
    add         r_work2, r_work2, r_work1
    // fall through
vexecute_mov_hi:
    // no flags, may update pc
    // adapted from written out version of: w_lo_hi_value_sp_pc r_work0, r_work2, r_tmp0, r_tmp1
    // r_work0: register to write to 0..15; r_work2: value to write
    sh2add      r_tmp0, r_work0, r_cpu
    addi        r_tmp1, r_work0, -13
    beqz        r_tmp1, v_write_sp
    addi        r_tmp1, r_tmp1, -2
    mv          r_work0, r_work2
    beqz        r_tmp1, v_write_pc_work0
    sw          r_work2, (r_tmp0)
    next_instruction

vexecute_010001_11:
    // branch to PC is undefined, so we don't care about getting it right
    // this is an unrolled register read
    h3.bextmi   r_tmp0, r_inst, 3, 4
    sh2add      r_tmp0, r_tmp0, r_cpu
    lw          r_work0, (r_tmp0)

    bexti       r_tmp0, r_inst, 7
    bnez        r_tmp0, vexecute_blx
vexecute_bx:
    check_exc_return_to_work0_trash_tmp2_work1
    j v_write_pc_work0

// .p2align 2 odd length, unlikely to be in a tight loop
vexecute_blx:
    add         r_tmp1, r_pc, 1 // thumb bit
    sw          r_tmp1, CPU_OFFSET_LR(r_cpu)
v_write_pc_work0:
    andi        r_pc, r_work0, ~1
    next_instruction

// === Load from Literal Pool
main_decode_align
vexecute_ldr_lit:  		//  0100 1x
//            // ldr (literal)
//            uint32_t rt = (op16 >> 8u) & 0x7u;
//            uint32_t imm8 = (op16 & 0xffu);
//            uint32_t addr = ((cpu->regs[PC] >> 2u) + 1u + imm8) << 2u;
//            cpu->regs[rt] = read_u32(addr);
    rlo_ptr_10_8 r_tmp2
    zext.b      r_inst, r_inst

    addi        r_tmp1, r_pc, 2 // + 2 to get (pc + 4)
    andi        r_tmp1, r_tmp1, ~3
    sh2add      r_inst, r_inst, r_tmp1
    read_mem_32_unchecked r_tmp1, r_inst // this is pc relative so no real reason to check
    sw          r_tmp1, (r_tmp2)
    next_instruction

v_ldr_str_r2i_setup:
    h3.bextmi   r_work2, r_inst, 6, 5
    j           1f
v_ldr_str_r3_setup:
    rlo_ptr_8_6 r_work2 // rm
    lw          r_work2, (r_work2)
1:
    rlo_ptr_5_3 r_work1 // rn
    lw          r_work1, (r_work1)
    rlo_ptr_2_0 r_work0 // rt
    ret

// work0=&Rt, work1=effective address
.macro ldrh_strh_r2i_setup
    jal         v_ldr_str_r2i_setup
    sh1add      r_work1, r_work2, r_work1
.endm

// work0=&Rt, work1=effective address
.macro ldr_str_r3_setup
    jal         v_ldr_str_r3_setup
    add         r_work1, r_work1, r_work2
.endm

// === Load/store single data item

main_decode_align
vexecute_010100: //  0101 00 // str, strh
    ldr_str_r3_setup
    bexti       r_tmp0, r_inst, 9
    bnez        r_tmp0, vexecute_strh
vexecute_str:
    lw          r_tmp0, (r_work0)
    write_mem_32_at_work1_from_tmp0
    next_instruction
vexecute_strh:
    lw          r_tmp0, (r_work0)
    write_mem_16_at_work1_from_tmp0
    next_instruction
vexecute_ldrsb:
    read_mem_s8_at_work1_to_tmp0
    // shared as uncommon
    j ld3_shared_write_tmp0_to_work0_next_instruction

main_decode_align
vexecute_010101: //  0101 01 // strb ldrsb
    ldr_str_r3_setup
    bexti       r_tmp0, r_inst, 9
    bnez        r_tmp0, vexecute_ldrsb
vexecute_strb:
    lw          r_tmp0, (r_work0)
    write_mem_8_at_work1_from_tmp0
    next_instruction

main_decode_align
vexecute_010110: // 0101 10 // ldr, ldrh
    ldr_str_r3_setup
    bexti       r_tmp0, r_inst, 9
    bnez        r_tmp0, vexecute_ldrh
vexecute_ldr:
    read_mem_32_at_work1_to_tmp0
    sw          r_tmp0, (r_work0)
    next_instruction

// === Generate SP-relative address
// main_decode_align: unlikely to be in a tight loop
vexecute_add_sp: //  1010 1x
    lw          r_tmp1, CPU_OFFSET_SP(r_cpu)
    j           vexecute_address_gen

main_decode_align
vexecute_010111: //  0101 11 // ldrb, lrdsh
    ldr_str_r3_setup
    bexti       r_tmp0, r_inst, 9
    bnez        r_tmp0, vexecute_ldrsh
vexecute_ldrb:
    read_mem_u8_at_work1_to_tmp0
ld3_shared_write_tmp0_to_work0_next_instruction:
    sw          r_tmp0, (r_work0)
    next_instruction
vexecute_ldrsh:
    read_mem_s16_at_work1_to_tmp0
    // shared as uncommon
    j ld3_shared_write_tmp0_to_work0_next_instruction

main_decode_align
vexecute_stri: //  0110 0x
   get_va_imm5_ra_rb r_tmp0, r_work1, r_work0, r_tmp1
   sh2add       r_work1, r_tmp1, r_tmp0
   lw           r_tmp0, (r_work0)
   write_mem_32_at_work1_from_tmp0
   next_instruction

main_decode_align
vexecute_ldri: //  0110 1x
   get_va_imm5_ra_rb r_tmp1, r_work1, r_work0, r_tmp0
   sh2add       r_work1, r_tmp0, r_tmp1
   read_mem_32_at_work1_to_tmp0
   sw           r_tmp0, (r_work0)
   next_instruction

main_decode_align
vexecute_strbi: //  0111 0x
   get_va_imm5_ra_rb r_tmp0, r_work1, r_work0, r_tmp1
   add          r_work1, r_tmp1, r_tmp0
   lw           r_tmp0, (r_work0)
   write_mem_8_at_work1_from_tmp0
   next_instruction

main_decode_align
vexecute_ldrbi: //  0111 1x
   get_va_imm5_ra_rb r_tmp1, r_work1, r_work0, r_tmp0
   add          r_work1, r_tmp0, r_tmp1
   read_mem_u8_at_work1_to_tmp0
   sw           r_tmp0, (r_work0)
   next_instruction


main_decode_align
vexecute_strhi: //  1000 0x
   ldrh_strh_r2i_setup
   lw           r_tmp0, (r_work0)
   write_mem_16_at_work1_from_tmp0
   next_instruction

main_decode_align
vexecute_ldrhi: //  1000 1x
   ldrh_strh_r2i_setup
   read_mem_u16_at_work1_to_tmp0
   sw           r_tmp0, (r_work0)
   next_instruction


// No .align required, starts with 16-bit
//main_decode_align
vexecute_strspi: //  1001 0x
    lw          r_tmp1, CPU_OFFSET_SP(r_cpu)
    rlo_ptr_10_8 r_tmp2
    zext.b      r_inst, r_inst
    sh2add      r_work1, r_inst, r_tmp1
    lw          r_tmp0, (r_tmp2)
    write_stack_32_at_work1_from_tmp0
    next_instruction

// No .align required, starts with 16-bit
//main_decode_align
vexecute_ldrspi: //  1001 1x
    lw          r_tmp1, CPU_OFFSET_SP(r_cpu)
    rlo_ptr_10_8 r_tmp2
    zext.b      r_inst, r_inst
    sh2add      r_work1, r_inst, r_tmp1
    read_stack_32_at_work1_to_tmp0
    sw          r_tmp0, (r_tmp2)
    next_instruction

// === Generate PC-relative address
main_decode_align
vexecute_adr: //  1010 0x
    addi        r_tmp1, r_pc, 2
    andi        r_tmp1, r_tmp1, ~3
vexecute_address_gen:
    rlo_ptr_10_8 r_work0 // rd
    zext.b      r_tmp0, r_inst
    sh2add      r_tmp1, r_tmp0, r_tmp1
    sw          r_tmp1, (r_work0)
    next_instruction

// odd length
vexecute_ldrh:
    read_mem_u16_at_work1_to_tmp0
    sw          r_tmp0, (r_work0)
    next_instruction


// === Miscellaneous 16-bit instructions
main_decode_align
vexecute_101100: //  1011 00
    bexti       r_tmp0, r_inst, 8
#if ARMULET_FEATURE_ARMV8M_BASELINE_CBZ_CBNZ
    bnez        r_tmp0, vexecute_cbz_cbnz
#else
    bnez        r_tmp0, vexecute_undefined // todo more specific?
#endif
    bexti       r_tmp0, r_inst, 9
    bnez        r_tmp0, vexecute_extend
vexecute_add_sub_sp:
    andi        r_tmp0, r_inst, 0x7f
    lw          r_work2, CPU_OFFSET_SP(r_cpu)

    bexti       r_tmp1, r_inst, 7      // 1 for sub, 0 for add
    beqz        r_tmp1, 1f
    neg         r_tmp0, r_tmp0
1:
    sh2add      r_work2, r_tmp0, r_work2
v_write_sp:
    andi        r_work2, r_work2, ~3
    sw          r_work2, CPU_OFFSET_SP(r_cpu)
    check_splim r_work2, r_tmp0
    next_instruction

vexecute_extend:
    rlo_ptr_5_3 r_work1 // rm
    rlo_ptr_2_0 r_work0 // rd
    lw          r_work1, (r_work1)
    h3.bextmi   r_tmp0, r_inst, 6, 2
    table_branch extend
    table_branch_entry extend,vexecute_extend_sxth
    table_branch_entry extend,vexecute_extend_sxtb
    table_branch_entry extend,vexecute_extend_uxth
    table_branch_entry extend,vexecute_extend_uxtb

vexecute_extend_sxtb: // 1
    sext.b      r_work1, r_work1
vexecute_extend_sxth: // 0
    sext.h      r_work1, r_work1
    j           vexecute_extend_done
vexecute_extend_uxtb: // 3 (most common)
    zext.b      r_work1, r_work1
    j           vexecute_extend_done // this is the commonest one, and this block is otherwise odd length anyway
vexecute_extend_uxth: // 2
    zext.h      r_work1, r_work1
vexecute_extend_done:
    sw          r_work1, (r_work0)
v_next_instruction_0:
    next_instruction

main_decode_align
vexecute_101101: //  1011 01
    bexti       r_tmp0, r_inst, 9
    bnez        r_tmp0, vexecute_101101_1
vexecute_push:
    // push
    // pre-decrement SP as a thought towards interrupts (if they share the same stack)
    // cpu->regs[SP] -= (uint32_t)__builtin_popcount(op16 & 0x1ffu) << 2u;
    // __compiler_memory_barrier();
    lw          r_work1, CPU_OFFSET_SP(r_cpu)
    andi        r_tmp1, r_inst, 0x1ff
    cpop        r_tmp0, r_tmp1
    slli        r_tmp0, r_tmp0, 2
    sub         r_work0, r_work1, r_tmp0
    sw          r_work0, CPU_OFFSET_SP(r_cpu)
    check_splim r_work0, r_tmp0
    // uint32_t addr = cpu->regs[SP];
    // uint32_t lo_bitmap = op16 & 0xff;
    // while (lo_bitmap) {
    //     uint regnum = __builtin_ctz(lo_bitmap);
    //     lo_bitmap &= ~(1u << regnum);
    //     write_u32(addr, cpu_regs[i]);
    //     addr += 4
    // }

#if !ASSUME_STACK_SAFE // then write_stack_32_at_work1_from_tmp0 is the same as write_mem_32_at_work1_from_tmp0 and we can tail into stmia
    andi        r_tmp2, r_inst, 0xff
    beq         r_tmp1,r_tmp2,1f // r14 included?
    lw          r_tmp0, CPU_OFFSET_LR(r_cpu)
    addi        r_work1,r_work1,-4
    write_mem_32_at_work1_from_tmp0
1:
    mv          r_work1,r_work0
    beqz        r_tmp2, v_next_instruction_0 // early skip for push {lr}
    li          r_work2,0 // zero pointer to avoid writeback
    j           v_stmia_entry

#else
    andi        r_tmp2, r_inst, 0xff
    beqz        r_tmp2, 2f // early skip for push {lr}
1:
    ctz         r_tmp1, r_tmp2
    bclr        r_tmp2, r_tmp2, r_tmp1
    sh2add      r_work0, r_tmp1, r_cpu
    lw          r_tmp0, (r_work0)
    write_stack_32_at_work1_from_tmp0
    addi        r_work1, r_work1, 4
    bnez        r_tmp2, 1b
2:
    bexti       r_tmp0, r_inst, 8
    beqz        r_tmp0, 1f
    // if (prefix & 1) {
    //     write_u32(addr, cpu->regs[LR]);
    // }
    lw          r_tmp0, CPU_OFFSET_LR(r_cpu)
    write_stack_32_at_work1_from_tmp0
1:
    next_instruction
#endif

main_decode_align
vexecute_101110: //  1011 10
#if ARMULET_FEATURE_ARMV8M_BASELINE_CBZ_CBNZ
    bexti       r_tmp0, r_inst, 8
    // Note cbz/cbnz are overwhelmingly more common than rev/rev16/rev16sh
    beqz        r_tmp0, vexecute_rev16_rev_revsh
vexecute_cbz_cbnz:
	rlo_ptr_2_0 r_work0 // rn
	lw          r_work0, (r_work0)
	bexti       r_work1, r_inst, 11	//r_work1 = do_branch_on_nonzero
	seqz        r_work0, r_work0 	// r_work0 = reg_was_zero
	beq			r_work0, r_work1, 1f
	h3.bextmi   r_tmp0, r_inst, 3, 5
	bexti       r_tmp1, r_inst, 9
    slli		r_tmp1, r_tmp1, 6
	sh1add      r_pc, r_tmp0, r_pc
    add         r_pc, r_pc, r_tmp1
    addi        r_pc, r_pc, 2
1:
    next_instruction
#endif
vexecute_rev16_rev_revsh:
    rlo_ptr_5_3 r_work1 // rm
    rlo_ptr_2_0 r_work0 // rd
    lw          r_tmp1, (r_work1)
    h3.bextmi   r_tmp0, r_inst, 6, 4
    addi        r_tmp0, r_tmp0, -8
    beqz        r_tmp0, vexecute_rev   // == 8
    addi        r_tmp0, r_tmp0, -1
    beqz        r_tmp0, vexecute_rev16 // == 9
    addi        r_tmp0, r_tmp0, -2
    bnez        r_tmp0, vexecute_undefined // != 11

vexecute_revsh:
    rev8        r_tmp1, r_tmp1
    srai        r_tmp1, r_tmp1, 16
    // Share tails as this is an uncommon instruction
    j 1f

vexecute_rev16:
    rori        r_tmp1, r_tmp1, 16
    // fall through
vexecute_rev:
    rev8        r_tmp1, r_tmp1
1:
    sw          r_tmp1, (r_work0)
    next_instruction

main_decode_align
vexecute_101111: //  1011 11
    bexti       r_tmp0, r_inst, 9
    bnez        r_tmp0, vexecute_101111_1
vexecute_pop:
    // uint32_t addr = cpu->regs[SP];
    addi        r_work2,r_cpu,CPU_OFFSET_SP
    lw          r_work1, (r_work2)
    // uint32_t lo_bitmap = op16 & 0xff;
    // while (lo_bitmap) {
    //     uint regnum = __builtin_clz(lo_bitmap);
    //     lo_bitmap &= ~(1u << regnum);
    //     cpu->regs[regnum] = read_u32(addr);
    //     addr += 4;
    // }
    andi        r_tmp2, r_inst, 0xff
    beqz        r_tmp2, 2f // early skip for pop {pc}
v_pop_entry:
1:
    ctz         r_tmp1, r_tmp2
    bclr        r_tmp2, r_tmp2, r_tmp1
    read_stack_32_at_work1_to_tmp0
    sh2add      r_work0, r_tmp1, r_cpu
    sw          r_tmp0, (r_work0)
    addi        r_work1, r_work1, 4
    bnez        r_tmp2, 1b
2:

    // if (prefix & 1) {
    //     uint32_t next_pc = read_u32(addr);
    //     check_exec_return(cpu, next_pc);
    //     update_pc(cpu, next_pc);
    //     addr += 4;
    // }
    bexti       r_tmp0, r_inst, 8
    beqz        r_tmp0, pop_no_pc
    read_stack_32_at_work1_to_tmp0
    addi        r_work1, r_work1, 4
    sw          r_work1, (r_work2)
    check_exc_return_to_tmp0_trash_tmp2_work1
    andi        r_pc, r_tmp0, ~1
    next_instruction

    // __compiler_memory_barrier();
    // cpu->regs[SP] = addr;
pop_no_pc:
    beqz        r_work2,1f
    sw          r_work1, (r_work2)
1:
    next_instruction

vexecute_dummy_bkpt:
    // Set r_inst=0xb000 in the handler (often a native ebreak), useful
    // telltale -- we enter this path on an splim check fail
    lui r_inst, 0xb
// should already be aligned, but just make sure:
main_decode_align
vexecute_101111_1:
#if VASM_HOOKS_INDEX_HINT_INSTR != VASM_HOOKS_INDEX_BKPT_INSTR + 1
#error "Uh oh"
#endif
    bexti       r_tmp2, r_inst, 8
    addi        r_tmp2, r_tmp2, VASM_HOOKS_INDEX_BKPT_INSTR
    enter_asm_hook_shared_if_smaller

// === Store multiple registers
main_decode_align
vexecute_stm: //  1100 0x
    // uint32_t rn = (op16 >> 8u) & 0x7u;
    rlo_ptr_10_8 r_work2 // rn
    // uint32_t addr = cpu->regs[rn];
    lw          r_work1, (r_work2)
    // uint32_t lo_bitmap = op16 & 0xffu;
    // while (lo_bitmap) {
    //     uint regnum = __builtin_ctz(lo_bitmap);
    //     lo_bitmap &= ~(1u << regnum);
    //     write_u32(addr, cpu->regs[regnum]);
    //     addr += 4;
    // }
    andi        r_tmp2, r_inst, 0xff
    // UNPREDICTABLE if bitmap is all-zeroes: treated as {r0} (save a beqz)
v_stmia_entry:
1:
    ctz         r_tmp1, r_tmp2
    bclr        r_tmp2, r_tmp2, r_tmp1
    sh2add      r_work0, r_tmp1, r_cpu
    lw          r_tmp0, (r_work0)
    write_mem_32_at_work1_from_tmp0
    addi        r_work1, r_work1, 4
    bnez        r_tmp2, 1b
2:
    beqz        r_work2,1f // writeback pointer==0? don't write to it
    sw          r_work1, (r_work2)
1:
    next_instruction

// odd length
vexecute_101101_1:
    li          r_tmp0, 3
    h3.bextmi   r_tmp1, r_inst, 5, 4
    bne         r_tmp0, r_tmp1, vexecute_undefined

    // 101101_10011
vexecute_cps:
    li          r_tmp2, VASM_HOOKS_INDEX_CPS_INSTR
    enter_asm_hook_shared_if_smaller

// === Load multiple registers
main_decode_align
vexecute_ldm: //  1101 0x // ldmia
    // uint32_t rn = (op16 >> 8u) & 0x7u;
    // This would be rlo_ptr_10_8 r_work2 but we also need to remember rn for later
    h3.bextmi   r_work0, r_inst, 8, 3
    sh2add      r_work2, r_work0, r_cpu
    // uint32_t addr = cpu->regs[rn];
    lw          r_work1, (r_work2)
    // uint32_t lo_bitmap = op16 & 0xffu;
    // while (lo_bitmap) {
    //     uint regnum = __builtin_ctz(lo_bitmap);
    //     lo_bitmap &= ~(1u << regnum);
    //     cpu->regs[regnum] = read_u32(addr);
    //     addr += 4;
    // }
#if !ASSUME_STACK_SAFE // then write_stack_32_at_work1_from_tmp0 is the same as write_mem_32_at_work1_from_tmp0 and we can tail into pop
    zext.b      r_inst, r_inst
    mv          r_tmp2,r_inst
    bext        r_tmp0, r_inst, r_work0
    beqz        r_tmp0, 1f
    li          r_work2,0 // in the list, so no writeback
1:
    j           v_pop_entry
#else
    // UNPREDICTABLE if bitmap is all-zeroes: treated as {r0}, except that rn
    // is written back even if rn is r0 (save a beqz)
    zext.b      r_tmp2, r_inst
1:
    ctz         r_tmp1, r_tmp2
    bclr        r_tmp2, r_tmp2, r_tmp1
    read_mem_32_at_work1_to_tmp0
    sh2add      r_tmp1, r_tmp1, r_cpu
    sw          r_tmp0, (r_tmp1)
    addi        r_work1, r_work1, 4
    bnez        r_tmp2, 1b
2:
    // if (!(op16 & (1u << rn))) cpu->regs[rn] = addr;
    bext        r_tmp0, r_inst, r_work0
    bnez        r_tmp0, 1f
    sw          r_work1, (r_work2)
1:
    next_instruction
#endif

// === Conditional branch, and Supervisor Call
main_decode_align
vexecute_1101xx: //  1101 xx
    h3.bextmi   r_tmp0, r_inst, 9, 3
    // eq/ne is the most common, make it faster
#define VCB_EQ_NE_FASTER 1
#if VCB_EQ_NE_FASTER
    beqz r_tmp0,vcb_eq_ne
#endif
    table_branch bxx
    table_branch_entry bxx,vcb_eq_ne    // 0000
    table_branch_entry bxx,vcb_cs_cc    // 0001
    table_branch_entry bxx,vcb_mi_pl    // 0010
    table_branch_entry bxx,vcb_vs_vc    // 0011
    table_branch_entry bxx,vcb_hi_ls    // 0100
    table_branch_entry bxx,vcb_ge_lt    // 0101
    table_branch_entry bxx,vcb_gt_le    // 0110
    table_branch_entry bxx,vcb_svc_udf  // 0111

// NOTE r_work2 is reserved in all vcb_x routines

// align both this common option, and the shared vcb_bxx_check
.p2align 2
vcb_eq_ne:    // 0000
#if VCB_EQ_NE_FASTER
    bexti       r_work2, r_inst, 8
#endif
    bnez        r_lazy_nz,vcb_bxx_no
    // fall through
vcb_bxx_yes:
    // This functions as a logical not (assuming input is 0/1) with a 16-bit
    // opcode, which keeps the next instruction aligned.
    addi        r_work2, r_work2, -1
vcb_bxx_no:
    beqz        r_work2, 1f
    sext.b      r_inst, r_inst
    sh1add      r_pc, r_inst, r_pc
    addi        r_pc, r_pc, 2
1:
    next_instruction

// packed to give aligned entry points for more-common branch types when first
// instruction is 32-bit -- in particular bvs/bvc are incredibly rare, as is svc

.p2align 2
vcb_cs_cc:    // 0001
    beqz        r_c,vcb_bxx_no
    j vcb_bxx_yes

vcb_vs_vc:    // 0011
    bltz        r_topbit_v,vcb_bxx_yes
    j vcb_bxx_no

vcb_hi_ls:    // 0100
    // take_branch = cpu->C && !get_Z(cpu);
    beqz        r_c,vcb_bxx_no
    beqz        r_lazy_nz,vcb_bxx_no
    j vcb_bxx_yes

vcb_mi_pl:    // 0010
    bltz        r_lazy_nz,vcb_bxx_yes
    j vcb_bxx_no

vcb_gt_le:    // 0110
    // take_branch = !get_Z(cpu) && get_N(cpu) == get_V(cpu);
    beqz        r_lazy_nz,vcb_bxx_no
    // fall thru
vcb_ge_lt:    // 0101
    // take_branch = get_N(cpu) == get_V(cpu);
    xor         r_work0,r_lazy_nz,r_topbit_v
    bltz        r_work0,vcb_bxx_no
    j vcb_bxx_yes

vcb_svc_udf:  // 0111
    beqz        r_work2, vexecute_undefined

vcb_svc:
    li          r_tmp2, VASM_HOOKS_INDEX_SVC_INSTR
    enter_asm_hook_shared_if_smaller

// === Unconditional Branch
// main_decode_align don't care, 16-bit
vexecute_branch: //  1110 0x
    // uint32_t delta = (uint32_t)((((int32_t) op16) << 21u) >> 20u);
    slli        r_inst, r_inst, 21
    srai        r_inst, r_inst, 20
    add         r_pc, r_pc, r_inst
    addi        r_pc, r_pc, 2
    next_instruction

// === 32 bit instructions from here on

main_decode_align
vexecute32_11110: //  1111 0x
    read_mem_u16_unchecked r_work2, r_pc
    add         r_pc, r_pc, 2
#if ARMULET_FEATURE_ARMV8M_BASELINE_MOVW_MOVT
    bexti       r_tmp0, r_work2, 15
    beqz        r_tmp0, vexecute32_dp
#endif
    bexti       r_tmp0, r_work2, 14
    beqz        r_tmp0, vexecute32_misc

vexecute32_11110x_and_11:
    bexti       r_tmp0, r_work2, 12
    beqz        r_tmp0, vexecute32_undefined
vexecute32_bl: // 11110x and 11x1
    add         r_tmp1, r_pc, 1 // thumb bit
    sw          r_tmp1, CPU_OFFSET_LR(r_cpu)
    //fallthrough

vexecute32_bw:
                                          // r_inst[12:0] = 1  0  S  imm10
    addi        r_tmp2, r_inst, -1024     // r_tmp2[12:0] = S !S !S  imm10
    slli        r_tmp0, r_work2, 5        // concatenate imm11 to end (plus 5 incidental zeroes)
    pack        r_tmp2, r_tmp0, r_tmp2    // (it's called pack but I use it to unpack things???)
    slli        r_tmp2, r_tmp2, 3         // Sign-extend and scale by 2 overall (sll 5 + 3 - 7)
    srai        r_tmp2, r_tmp2, 7         // {{8{S}}, !S, !S, imm10, imm11, 1'b0}

    bexti       r_tmp0, r_work2, 13       // J1
    bexti       r_tmp1, r_work2, 11       // J2
    sh1add      r_tmp0, r_tmp0, r_tmp1    // {J1, J2}
    slli        r_tmp0, r_tmp0, 22

    xor         r_tmp0, r_tmp0, r_tmp2    // Mix the pasta and the sauce
    add         r_pc, r_pc, r_tmp0
    next_instruction

#if ARMULET_FEATURE_ARMV8M_BASELINE_MOVW_MOVT
main_decode_align
vexecute32_dp:
    pack         r_tmp1, r_work2, r_inst // xxx:imm4:0:imm3:xxx
    srli         r_tmp1, r_tmp1, 12      //     xxx:imm4:0:imm3
    packh        r_tmp1, r_work2, r_tmp1 //    imm4:0:imm3:imm8
    andi         r_tmp2, r_inst, 1 << 10 //          i:  zeroes
    sh1add       r_tmp1, r_tmp2, r_tmp1  //    imm4:i:imm3:imm8 (imm16 complete)
    r_ptr32_11_8  r_tmp0         //Rd
    h3.bextmi    r_tmp2, r_inst, 4, 6    //was it even an instr we support?
    addi         r_tmp2, r_tmp2, -0x24
    bnez         r_tmp2, 1f
    // fallthrough to common case of movw
vexecute32_movw:
    sw           r_tmp1, (r_tmp0)
    next_instruction
1:
    addi         r_tmp2, r_tmp2, -0x08
    bnez         r_tmp2, vexecute32_undefined
    // fallthrough
vexecute32_movt:
    sh           r_tmp1, 2(r_tmp0)
    next_instruction

#endif

vexecute32_misc: // 11110x and 10
    bexti       r_tmp0, r_work2, 12
#if ARMULET_FEATURE_ARMV8M_BASELINE_BW
    bnez        r_tmp0, vexecute32_bw // op2 == 1 or op2 == 3
#else
    bnez        r_tmp0, vexecute32_undefined // op2 == 1 or op2 == 3
#endif
    bexti       r_tmp0, r_inst, 10
    bnez        r_tmp0, vexecute32_undefined // includes udf
    andi        r_tmp0, r_tmp0, 3 // op1 == 1xxxxxx
    bnez        r_tmp0, vexecute32_undefined // includes udf
    // 11110x and 10x0 (op2 == 0)
    h3.bextmi   r_tmp0, r_inst, 5, 7 // op1 >> 1
    addi        r_tmp0, r_tmp0, -0x1c
    beqz        r_tmp0, vexecute32_msr // == 0x1c
    addi        r_tmp0, r_tmp0, -1
    beqz        r_tmp0, vexecute32_misc_control // == 0x1d
    addi        r_tmp0, r_tmp0, -2
    bnez        r_tmp0, vexecute32_undefined // != 0x1f
    // fall thru
vexecute32_mrs:
    li          r_tmp2, VASM_HOOKS_INDEX_MRS_INSTR
    enter_asm_hook_shared_if_smaller
vexecute32_msr:
    li          r_tmp2, VASM_HOOKS_INDEX_MSR_INSTR
    enter_asm_hook_shared_if_smaller
vexecute32_misc_control:
    li          r_tmp2, VASM_HOOKS_INDEX_MISC_CONTROL_INSTR
    // fall through
shared_enter_asm_hook:
    tail_call_asm_hook_in_tmp2_trash_work1

#if ARMULET_FEATURE_ARMV8M_BASELINE_SDIV_UDIV
vexecute32_111110:
    read_mem_u16_unchecked r_work2, r_pc
    add         r_pc, r_pc, 2
    r_ptr32_19_16 r_tmp0
    lw          r_tmp0, (r_tmp0)         //Rn
    r_ptr32_3_0   r_tmp1
    lw          r_tmp1, (r_tmp1)         //Rm
    r_ptr32_11_8  r_work0                  //&Rd
    h3.bextmi   r_tmp2, r_inst, 4, 6    //was it even an instr we support?
    addi        r_tmp2, r_tmp2, -0x39
    beqz        r_tmp2, vexecute32_sdiv
    addi        r_tmp2, r_tmp2, -0x02
    bnez        r_tmp2, vexecute32_undefined
    //fallthrough

vexecute32_udiv:
    beqz        r_tmp1, div_by_zero
    divu        r_tmp0, r_tmp0, r_tmp1
    sw          r_tmp0, (r_work0)
    next_instruction

vexecute32_sdiv:
    beqz        r_tmp1, div_by_zero
    div         r_tmp0, r_tmp0, r_tmp1
    sw          r_tmp0, (r_work0)
    next_instruction

div_by_zero:
    sw          r_tmp1, (r_work0)
    next_instruction
#endif

vexecute_undefined:
    jmp_hook_undefined16_trash_tmp2_work1

vexecute32_prefix_undefined:
    read_mem_u16_unchecked r_work2, r_pc
    addi        r_pc, r_pc, 2
    // fall thru

vexecute32_undefined:
    jmp_hook_undefined32_trash_tmp2_work1

// === END OF INSTRUCTIONS
