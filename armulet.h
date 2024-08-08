#pragma once

#ifndef ARMULET_FEATURE_PROFILER
//#define ARMULET_FEATURE_PROFILER 1
#endif
// ability to call into ARMv6M code and have it return
#ifndef ARMULET_FEATURE_CALL
//#define ARMULET_FEATURE_CALL 1
#endif
#ifndef ARMULET_USE_LAZY_NZ
#define ARMULET_USE_LAZY_NZ 1
#endif
#ifndef ARMULET_USE_LAZY_Z
//#define ARMULET_USE_LAZY_Z  1
#endif

#if PICO_ON_DEVICE
#define ARMULET_USE_REAL_MEMORY 1
#endif

#if !defined(ARMULET_FEATURE_ARMV8M_BASELINE_SDIV_UDIV) && defined(ARMULET_FEATURE_ARMV8M_BASELINE)
#define ARMULET_FEATURE_ARMV8M_BASELINE_SDIV_UDIV 1
#endif

#if !defined(ARMULET_FEATURE_ARMV8M_BASELINE_MOVW_MOVT) && defined(ARMULET_FEATURE_ARMV8M_BASELINE)
#define ARMULET_FEATURE_ARMV8M_BASELINE_MOVW_MOVT 1
#endif

#if !defined(ARMULET_FEATURE_ARMV8M_BASELINE_BW) && defined(ARMULET_FEATURE_ARMV8M_BASELINE)
#define ARMULET_FEATURE_ARMV8M_BASELINE_BW 1
#endif

#if !defined(ARMULET_FEATURE_ARMV8M_BASELINE_CBZ_CBNZ) && defined(ARMULET_FEATURE_ARMV8M_BASELINE)
#define ARMULET_FEATURE_ARMV8M_BASELINE_CBZ_CBNZ 1
#endif

#if !defined(ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM) && defined(ARMULET_FEATURE_ARMV8M_BASELINE)
#define ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM 1
#endif

#define ARMULET_CALL_RETURN_ADDRESS 0xffa00000;

// todo move these out
#if ARMULET_FEATURE_STEP_STATUS
#define ARMULET_IST_NORMAL      0
#define ARMULET_IST_BREAKPOINT  1
#define ARMULET_IST_UNDEFINED16 2
#define ARMULET_IST_UNDEFINED32 3
#define ARMULET_IST_SVC         4
#endif

#ifndef __ASSEMBLER__
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#if ARMULET_FEATURE_CALL && !ARMULET_USE_ASM
#include <setjmp.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif

enum M0PLUS_REGS {
    ARM_REG_R0=0,
    ARM_REG_R1,
    ARM_REG_R2,
    ARM_REG_R3,
    ARM_REG_R4,
    ARM_REG_R5,
    ARM_REG_R6,
    ARM_REG_R7,
    ARM_REG_R8,
    ARM_REG_R9,
    ARM_REG_R10,
    ARM_REG_R11,
    ARM_REG_R12,
    ARM_REG_SP,
    ARM_REG_LR,
    ARM_REG_PC,
    NUM_M0PLUS_REGS
};

static_assert(NUM_M0PLUS_REGS == 16, "");

typedef struct armulet_cpu {
    uint32_t regs[NUM_M0PLUS_REGS];
#if ARMULET_USE_LAZY_NZ
    uint32_t lazy_nz_val;
#else
    bool _N; // should not be accessed directly
    bool _Z; // should not be accessed directly
#endif
#if ARMULET_USE_LAZY_Z
    uint32_t lazy_v0;
    uint32_t lazy_v1;
    uint32_t lazy_v2;
#else
    bool _V;
#endif
    bool C;
    bool primask;
    uint8_t ipsr;
#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
    uint32_t splim;
#endif
#if ARMULET_FEATURE_ASM_HOOKS_IS_FUNCTION
    // state to be passed to the asm hook for this instance
    uint32_t asm_hook_param;
#endif
#if ARMULET_FEATURE_SVC_HANDLER
    uint32_t (*svc_handler)(uint32_t a, uint32_t b, uint32_t c, uint32_t d);
#endif
#if ARMULET_FEATURE_STEP_STATUS
    int   step_status;
#endif
#if !ARMULET_USE_ASM
    uint32_t pc_delta; // used post instruction
#if ARMULET_FEATURE_CALL
    bool armulet_call;
    jmp_buf jmpbuf;
#endif
#endif
} armulet_cpu_t;

void armulet_reset_cpu(armulet_cpu_t *cpu);

#if !ARMULET_USE_ASM
void __attribute__((noreturn)) armulet_jump(armulet_cpu_t *cpu, uint32_t addr);
#if ARMULET_FEATURE_CALL
uint32_t armulet_call(armulet_cpu_t *cpu, uint32_t addr);
#endif
#if ARMULET_FEATURE_SPECIAL_READ
uint32_t armulet_cb_special_read(uint32_t addr, int size);
void armulet_cb_special_write(uint32_t addr, int size, uint32_t value);
#endif
#endif

#if ARMULET_DEBUG
#if ARMULET_USE_ASM
extern struct varmulet_asm_hooks single_step_asm_hooks;
#endif
void armulet_single_step(armulet_cpu_t *cpu);
#if !PICO_ON_DEVICE
void armulet_zap(uint8_t byte);
#endif
#endif

#ifdef __cplusplus
}
#endif
#endif