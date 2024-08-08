#pragma once
#if PICO_RP2350
#include "pico.h"
#endif
#include "armulet.h"

#if !ARMULET_USE_REAL_MEMORY
#error VARMULET requires ARMULET_USE_REAL_MEMORY
#endif

#ifndef VARMULET_USE_WATERMARK
#define VARMULET_USE_WATERMARK ARMULET_FEATURE_CHECKED_MEMORY
#else
#if ARMULET_FEATURE_CHECKED_MEMORY & !VARMULET_USE_WATERMARK
#error ARMULET_FEATURE_CHECKED_MEMORY requires VARMULET_USE_WATERMARK
#endif
#endif

// Use multi-bit extract from Hazard3 for improved speed
#ifndef VARMULET_USE_HAZARD3_CUSTOM
#define VARMULET_USE_HAZARD3_CUSTOM 1
#endif

// the instruction implementations must read the next instruction into r_inst (note this
// is automatically handled by the 'next_instruction' macro
#ifndef VARMULET_USE_EARLY_INSTR_READ
#define VARMULET_USE_EARLY_INSTR_READ 1
#endif

#define CPU_OFFSET_R0          (0 * 4)
#define CPU_OFFSET_R1          (1 * 4)
#define CPU_OFFSET_R2          (2 * 4)
#define CPU_OFFSET_R3          (3 * 4)
#define CPU_OFFSET_R12         (12 * 4)
#define CPU_OFFSET_SP          (13 * 4)
#define CPU_OFFSET_LR          (14 * 4)
#define CPU_OFFSET_PC          (15 * 4)
#define CPU_OFFSET_LAZY_NZ     (16 * 4)
#define CPU_OFFSET_V           68
#define CPU_OFFSET_C           69
#define CPU_OFFSET_PRIMASK     70
#define CPU_OFFSET_IPSR        71
#define __CPU_BASE             72
#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
#define CPU_OFFSET_SPLIM       __CPU_BASE
#define CPU_SPLIM_SIZE         4
#else
#define CPU_SPLIM_SIZE         0
#endif
#if ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
#define CPU_OFFSET_ASM_HOOK_PARAM    (__CPU_BASE + CPU_SPLIM_SIZE)
#define CPU_ASM_HOOK_PARAM_SIZE         4
#else
#define CPU_ASM_HOOK_PARAM_SIZE         0
#endif
#if ARMULET_FEATURE_STEP_STATUS
#define CPU_OFFSET_STEP_STATUS    (__CPU_BASE + CPU_SPLIM_SIZE + CPU_ASM_HOOK_PARAM_SIZE)
#define CPU_STEP_STATUS_SIZE         4
#else
#define CPU_STEP_STATUS_SIZE         0
#endif

#ifndef __ASSEMBLER__
static_assert(offsetof(armulet_cpu_t, ipsr) == CPU_OFFSET_IPSR, "");
#if ARMULET_FEATURE_STEP_STATUS
static_assert(offsetof(armulet_cpu_t, step_status) == CPU_OFFSET_STEP_STATUS, "");
#endif
#endif

#define VASM_HOOKS_INDEX_ENTER_FN          0
#define VASM_HOOKS_INDEX_EXIT_FN           1
// save regs to stack prior to calling into ABI function
#define VASM_HOOKS_INDEX_SAVE_REGS_FN      2
// restore regs from stack after calling into ABI function
#define VASM_HOOKS_INDEX_RESTORE_REGS_FN   3
// start executing next instruction (at PC)... note that if
// VARMULET_USE_EARLY_INSTR_READ=1, then r_inst has been loaded from PC prior to jumping here
#if !VARMULET_USE_ENTER_HOOK_TO_OVERRIDE_REGISTER_STORED_HOOKS
#define VASM_HOOKS_INDEX_NEXT_INSTRUCTION  4
#define VASM_HOOKS_INDEX_MAIN_DECODE_TABLE 5
#define VASM_HOOKS_INDEX_DP_DECODE_TABLE   6
// raw unknown 16 bit instruction, instruction in r_inst; should end with `next_instruction`
#define VASM_HOOKS_INDEX_UNDEFINED16       7
#else
#define VASM_HOOKS_INDEX_UNDEFINED16       4
#endif
// raw unknown 32 bit instruction, instruction in r_inst : r_tmp0; should end with `next_instruction`
#define VASM_HOOKS_INDEX_UNDEFINED32       (VASM_HOOKS_INDEX_UNDEFINED16+1)
#define VASM_HOOKS_INDEX_BKPT_INSTR        (VASM_HOOKS_INDEX_UNDEFINED16+2)
#define VASM_HOOKS_INDEX_HINT_INSTR        (VASM_HOOKS_INDEX_UNDEFINED16+3)
// raw decoded svc_call, instruction in r_inst; SVC number is in low byte of r_inst, should end with `next_instruction`
#define VASM_HOOKS_INDEX_SVC_INSTR         (VASM_HOOKS_INDEX_UNDEFINED16+4)
// raw decoded cps_call, instruction in r_inst; should end with `next_instruction`
#define VASM_HOOKS_INDEX_CPS_INSTR         (VASM_HOOKS_INDEX_UNDEFINED16+5)
#define VASM_HOOKS_INDEX_MRS_INSTR         (VASM_HOOKS_INDEX_UNDEFINED16+6)
#define VASM_HOOKS_INDEX_MSR_INSTR         (VASM_HOOKS_INDEX_UNDEFINED16+7)
#define VASM_HOOKS_INDEX_MISC_CONTROL_INSTR (VASM_HOOKS_INDEX_UNDEFINED16+8)
// primask updated; new value in r_work0
#define VASM_HOOKS_INDEX_UPDATE_PRIMASK_FN (VASM_HOOKS_INDEX_UNDEFINED16+9)
// raw decoded bx to registoer >= 0xf0000000, instruction in r_inst, should end with `next_instruction`
#define VASM_HOOKS_INDEX_EXC_RETURN        (VASM_HOOKS_INDEX_UNDEFINED16+10)
// if an EXC_RETURN is taken to ARMULET_CALL_RETURN_ADDRESS, jmp via here
#define VASM_HOOKS_INDEX_CALL_RETURN       (VASM_HOOKS_INDEX_UNDEFINED16+11)

#ifndef __ASSEMBLER__
#ifdef __cplusplus
extern "C" {
#endif
static_assert(offsetof(armulet_cpu_t, lazy_nz_val) == CPU_OFFSET_LAZY_NZ, "");
static_assert(offsetof(armulet_cpu_t, _V) == CPU_OFFSET_V, "");
static_assert(offsetof(armulet_cpu_t, C) == CPU_OFFSET_C, "");
static_assert(offsetof(armulet_cpu_t, primask) == CPU_OFFSET_PRIMASK, "");
#if ARMULET_FEATURE_SVC_HANDLER
static_assert(offsetof(armulet_cpu_t, svc_handler) == CPU_OFFSET_SVC_HANDLER, "");
#endif
#if ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
static_assert(offsetof(armulet_cpu_t, asm_hook_param) == CPU_OFFSET_ASM_HOOK_PARAM, "");
#endif

#ifdef VARMULET_ASM_HOOK_TYPE
typedef VARMULET_ASM_HOOK_TYPE asm_hook_t;
#else
typedef uintptr_t asm_hook_t;
#endif
typedef struct varmulet_asm_hooks {
    asm_hook_t enter_fn;
    asm_hook_t exit_fn;
    asm_hook_t save_regs_fn;
    asm_hook_t restore_regs_fn;
#if !VARMULET_USE_ENTER_HOOK_TO_OVERRIDE_REGISTER_STORED_HOOKS
    asm_hook_t next_instruction;
    asm_hook_t main_decode_table;
    asm_hook_t dp_decode_table;
#endif
    asm_hook_t undefined16;
    asm_hook_t undefined32;
    asm_hook_t bkpt_instr;
    asm_hook_t hint_instr;
    asm_hook_t svc_instr;
    asm_hook_t cps_instr;
    asm_hook_t mrs_instr;
    asm_hook_t msr_instr;
    asm_hook_t misc_control_instr;
    asm_hook_t update_primask_fn;
    asm_hook_t exc_return;
    asm_hook_t call_return;
} varmulet_asm_hooks_t;

static_assert(offsetof(varmulet_asm_hooks_t, exc_return) == VASM_HOOKS_INDEX_EXC_RETURN * sizeof(asm_hook_t), "");
static_assert(offsetof(varmulet_asm_hooks_t, update_primask_fn) == VASM_HOOKS_INDEX_UPDATE_PRIMASK_FN * sizeof(asm_hook_t), "");
static_assert(offsetof(varmulet_asm_hooks_t, exc_return) == VASM_HOOKS_INDEX_EXC_RETURN * sizeof(asm_hook_t), "");

int varmulet_run(armulet_cpu_t *cpu, const varmulet_asm_hooks_t *hooks);
void varmulet_step(armulet_cpu_t *cpu, const varmulet_asm_hooks_t *hooks);

extern varmulet_asm_hooks_t varmulet_default_asm_hooks;

#ifndef __cplusplus
// declare these addresses all as const void, so there is no temptation to call them - note some compilers might not like this (e.g. CPP)
extern const void varmulet_hook_default_enter_fn;
extern const void varmulet_hook_default_exit_fn;
extern const void varmulet_hook_default_save_regs_fn;
extern const void varmulet_hook_default_restore_regs_fn;
extern const void varmulet_hook_default_execute_instruction;
extern const void varmulet_main_decode_table;
extern const void varmulet_dp_decode_table;
extern const void varmulet_hook_default_bkpt_instr;
extern const void varmulet_hook_default_svc_instr;
extern const void varmulet_hook_default_hint_instr;
extern const void varmulet_hook_default_cps_instr;
extern const void varmulet_hook_default_mrs_instr;
extern const void varmulet_hook_default_msr_instr;
extern const void varmulet_hook_default_misc_control_instr;
extern const void varmulet_hook_default_update_primask_fn;
extern const void varmulet_hook_default_exc_return;
extern const void varmulet_hook_default_call_return;
extern const void varmulet_halt;
#endif

#ifdef __cplusplus
}
#endif
#endif // __ASSEMBLER__
