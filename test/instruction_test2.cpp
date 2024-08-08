#include "assembler.h"
#include <string>
#include <vector>
#include <functional>
#include "armulet_accessors.h"
#if ARMULET_USE_ASM
#include "varmulet_hooks_test.h"
#endif

#pragma clang diagnostic ignored "-Wgnu-designator"

#include "armulet.h"

// used to skip tests
#define NEEDS_NZ 0 // with lazy_nz, emulator cannot represent the case where N==1 and Z==1
#define UNPREDICTABLE 0
#define UNDEFINED 1
#define HARD_FAULT 0
#define PRIVILEGED 0
#define CONTROL 0

#include "armulet.h"
#include <cstdarg>
#include <cstring>
#if __riscv
#define RAM_START_ADDRESS 0x20010000u
#else
#define RAM_START_ADDRESS 0x20070000u
#endif

#define INITIAL_SP RAM_START_ADDRESS + 0x4000
#define INITIAL_LR 0x00000000
#define INITIAL_PC RAM_START_ADDRESS

#define IMM_0 0
#define IMM_1 1
#define IMM_3 3
#define IMM_4 4
#define IMM_31 31
#define IMM_32 0

#define IPSR_VAL 0x20

#define READ_ONLY 0
#define READ_WRITE 1

armulet_cpu_t cpu;

#define APSR_N      (1U << 31U) /* Negative flag */
#define APSR_Z      (1 << 30)   /* Zero flag */
#define APSR_C      (1 << 29)   /* Carry flag */
#define APSR_V      (1 << 28)   /* Overflow flag */
#define IPSR_MASK   0x3F        /* Mask for exception number */
#define EPSR_T      (1 << 24)   /* Thumb mode flag */

/* Useful xPSR flag combinations for masking off at runtime. */
#define APSR_NZCV   (APSR_N | APSR_Z | APSR_C | APSR_V)
#define APSR_NZC    (APSR_N | APSR_Z | APSR_C)
#define APSR_NZ     (APSR_N | APSR_Z)
#define APSR_NC     (APSR_N | APSR_C)
#define APSR_ZC     (APSR_Z | APSR_C)

/* Bits in PinkySimContext::PRIMASK */
#define PRIMASK_PM (1 << 0)

/* Condition codes */
#define COND_EQ 0x0
#define COND_NE (COND_EQ | 1)
#define COND_CS 0x2
#define COND_CC (COND_CS | 1)
#define COND_MI 0x4
#define COND_PL (COND_MI | 1)
#define COND_VS 0x6
#define COND_VC (COND_VS | 1)
#define COND_HI 0x8
#define COND_LS (COND_HI | 1)
#define COND_GE 0xA
#define COND_LT (COND_GE | 1)
#define COND_GT 0xC
#define COND_LE (COND_GT | 1)
#define COND_AL 0xE

/* SYSm field values for MSR and MRS instructions. */
#define SYS_APSR    0
#define SYS_IAPSR   1
#define SYS_EAPSR   2
#define SYS_XPSR    3
#define SYS_IPSR    5
#define SYS_EPSR    6
#define SYS_IEPSR   7
#define SYS_MSP     8
#define SYS_PSP     9
#define SYS_PRIMASK 16
#define SYS_CONTROL 20

/* Register names / indices into PinkySimContext::R array for first 13 registers. */
#define R0  0
#define R1  1
#define R2  2
#define R3  3
#define R4  4
#define R5  5
#define R6  6
#define R7  7
#define R8  8
#define R9  9
#define R10 10
#define R11 11
#define R12 12
#define SP  13
#define LR  14
#define PC  15

/* Values that can be returned from the pinkySimStep() or pinkySimRun() function. */
#define PINKYSIM_STEP_OK            0   /* Executed instruction successfully. */
#define PINKYSIM_STEP_UNDEFINED16    1   /* Encountered undefined instruction. */
#define PINKYSIM_STEP_UNPREDICTABLE 2   /* Encountered instruction with unpredictable behaviour. */
#define PINKYSIM_STEP_HARDFAULT     3   /* Encountered instruction which generates hard fault. */
#define PINKYSIM_STEP_BKPT          4   /* Encountered BKPT instruction or other debug event. */
#define PINKYSIM_STEP_UNSUPPORTED   5   /* Encountered instruction not supported by simulator. */
#define PINKYSIM_STEP_SVC           6   /* Encountered SVC instruction. */
#define PINKYSIM_RUN_INTERRUPT      7   /* pinkySimRun() callback signalled interrupt. */
#define PINKYSIM_RUN_WATCHPOINT     8   /* pinkySimRun() callback signalled watchpoint event. */
#define PINKYSIM_RUN_SINGLESTEP     9   /* pinkySimRun() callback signalled single step. */
#define PINKYSIM_STEP_WFE           10
#define PINKYSIM_STEP_WFI           11
#define PINKYSIM_STEP_SEV           12
#define PINKYSIM_STEP_UNDEFINED32   13   /* Encountered undefined instruction. */

void CHECK_EQUAL(uint32_t t, uint32_t other) {
    if (t != other) {
        printf("Mismatch: %08x != %08x\n", (int)t, (int)other);
//            std::cout << "Mismatch: " << other << " != " << t << std::endl;
#if PICO_RISCV
        asm ("ebreak");
#endif
        assert(false);
    }
}

#define CHECK_TRUE(x) CHECK_EQUAL((x), true)
#define CHECK_FALSE(x) CHECK_EQUAL((x), false)

struct IMemory {};

struct PinkySimContext {
    IMemory *pMemory;
    int PRIMASK;
};
struct cpu_test
{
    cpu_test(const std::string &name, std::function<void()> func) : name(name), func(func) {}

    void operator()() {
        func();
    }

    std::string name;
    std::function<void()> func;
};

int             m_expectedStepReturn;
uint32_t        m_expectedXPSRflags;
uint32_t        m_expectedRegisterValues[13];
uint32_t        m_expectedSPmain;
uint32_t        m_expectedLR;
uint32_t        m_expectedPC;
uint32_t        m_expectedIPSR;
#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
uint32_t        m_expectedMSPLIM;
#endif
uint32_t        m_emitAddress;
PinkySimContext m_context;

void initContext();
#if __APPLE__ || __riscv
#define OTHER_VALIST_BEHAVIOR 1
#endif
#if OTHER_VALIST_BEHAVIOR
va_list emitInstruction16Varg(const char* pEncoding, va_list valist);
#else
void emitInstruction16Varg(const char* pEncoding, va_list valist);
#endif
void validateXPSR();
void validateRegisters();

void setup()
{
    m_expectedStepReturn = PINKYSIM_STEP_OK;
#if !PICO_ON_DEVICE
    static uint8_t xx;
    armulet_zap(++xx);
#endif
    armulet_reset_cpu(&cpu);
    initContext();
}

void teardown()
{
}

uint32_t IMemory_Read32(IMemory* pThis, uint32_t address) {
    return armulet_read_u32(address);
}

void SimpleMemory_SetMemory(IMemory* pMem, uint32_t address, uint32_t value, int readOnly)
{
    armulet_write_u32(address, value);
}

void setExpectedStepReturn(int expectedStepReturn)
{
    m_expectedStepReturn = expectedStepReturn;
}

void setExpectedXPSRflags(const char* pExpectedFlags)
{
    // Remember what expected APSR flags should be after instruction execution and flip initial flag state to make
    // sure that simular correctly flips the state and doesn't just get lucky to match a pre-existing condition.
    bool n = armulet_get_N(&cpu);
    bool z = armulet_get_Z(&cpu);
    while (*pExpectedFlags)
    {
        switch (*pExpectedFlags)
        {
            case 'n':
                m_expectedXPSRflags &= ~APSR_N;
                n = true;
                break;
            case 'N':
                m_expectedXPSRflags |= APSR_N;
                n = false;
                break;
            case 'z':
                m_expectedXPSRflags &= ~APSR_Z;
                z = true;
                break;
            case 'Z':
                m_expectedXPSRflags |= APSR_Z;
                z = false;
                break;
            case 'c':
                m_expectedXPSRflags &= ~APSR_C;
                cpu.C = true;
                break;
            case 'C':
                m_expectedXPSRflags |= APSR_C;
                cpu.C = false;
                break;
            case 'v':
                m_expectedXPSRflags &= ~APSR_V;
                cpu._V = true;
                break;
            case 'V':
                m_expectedXPSRflags |= APSR_V;
                cpu._V = false;
                break;
            case 't':
                m_expectedXPSRflags &= ~EPSR_T;
                break;
            case 'T':
                m_expectedXPSRflags |= EPSR_T;
                break;
        }
        if (n && z) {
            // we cant set both neg and zero
            static int foo;
            if (foo & 1) n = false; else z = false;
            foo++;
        }
        armulet_set_NZ(&cpu, n, z);
        pExpectedFlags++;
    }
}

void setExpectedIPSR(uint32_t expectedValue)
{
    m_expectedIPSR = expectedValue;
}

#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
void setExpectedMSPLIM(uint32_t expectedValue)
{
    m_expectedMSPLIM = expectedValue;
}
#endif

void setExpectedRegisterValue(int index, uint32_t expectedValue)
{
    assert (index >= 0 && index <= PC);

    if (index == PC)
        m_expectedPC = expectedValue;
    else if (index == LR)
        m_expectedLR = expectedValue;
    else if (index == SP)
        m_expectedSPmain = expectedValue;
    else
        m_expectedRegisterValues[index] = expectedValue;
}

void setRegisterValue(int index, uint32_t value)
{
    assert (index >= 0 && index <= PC);

    setExpectedRegisterValue(index, value);
    if (index == PC) {
        setExpectedRegisterValue(index, value + 2);
        cpu.regs[PC] = value;
//        armulet_set_pc(&cpu, value);
    } else if (index == LR) {
        cpu.regs[LR] = value;
    } else if (index == SP) {
//            m_context.spMain = value;
        cpu.regs[SP] = value;
    } else {
        cpu.regs[index] = value;
    }
}

void initContext()
{
    /* By default we will place the processor in Thumb mode. */
//        m_context.xPSR = EPSR_T;
    m_expectedXPSRflags = 0;
    // initial expected flags should be unchanged
    if (cpu.C) m_expectedXPSRflags |= APSR_C;
    if (armulet_get_V(&cpu)) m_expectedXPSRflags |= APSR_V;
    if (armulet_get_N(&cpu)) m_expectedXPSRflags |= APSR_N;
    if (armulet_get_Z(&cpu)) m_expectedXPSRflags |= APSR_Z;
    m_expectedXPSRflags |= EPSR_T;

    /* Expect the interrupt number to be 0 by default. */
    setExpectedIPSR(0);

    setExpectedMSPLIM(0);

    /* Randomly initialize each APSR flag to help verify that the simulator doesn't clear/set a bit that the
       specification indicates shouldn't be modified by an instruction. */
    // todo
//        for (uint32_t bit = APSR_N ; bit >= APSR_V ; bit >>= 1)
//        {
//            int setOrClear = rand() & 1;
//            if (setOrClear)
//            {
//                m_context.xPSR |= bit;
//                m_expectedXPSRflags |= bit;
//            }
//            else
//            {
//                m_context.xPSR &= ~bit;
//                m_expectedXPSRflags &= ~bit;
//            }
//        }

    /* Place 0x11111111 in R1, 0x22222222 in R2, etc. */
    uint32_t value = 0;
    for (int i = 0 ; i < 13 ; i++)
    {
        cpu.regs[i] = value;
        m_expectedRegisterValues[i] = value;
        value += 0x11111111;
    }

    /* These defaults are values that would work on the LPC1768. */
    setRegisterValue(SP, INITIAL_SP);
    setRegisterValue(LR, INITIAL_LR);
    setRegisterValue(PC, INITIAL_PC);

    m_emitAddress = INITIAL_PC;
}

void emitInstruction16(const char* pEncoding, ...)
{
    va_list     valist;

    va_start(valist, pEncoding);
    emitInstruction16Varg(pEncoding, valist);
    va_end(valist);
}

void emitInstruction32(const char* pEncoding1, const char* pEncoding2, ...)
{
    va_list     valist;

    va_start(valist, pEncoding2);
#if OTHER_VALIST_BEHAVIOR
    valist = emitInstruction16Varg(pEncoding1, valist);
    valist = emitInstruction16Varg(pEncoding2, valist);
#else
    emitInstruction16Varg(pEncoding1, valist);
    emitInstruction16Varg(pEncoding2, valist);
#endif
    va_end(valist);

    setExpectedRegisterValue(PC, INITIAL_PC + 4);
}

#if OTHER_VALIST_BEHAVIOR
va_list emitInstruction16Varg(const char* pEncoding, va_list valist)
#else
void emitInstruction16Varg(const char* pEncoding, va_list valist)
#endif
{
    uint16_t    instr = 0;
    size_t      i = 0;
    char        last = '\0';
    const char* p;
    struct Field
    {
        uint32_t value;
        char     c;
    } fields[6];

    assert (16 == strlen(pEncoding));
    memset(fields, 0, sizeof(fields));

    // Go through pEncoding from left to right and find all fields to be inserted.
    p = pEncoding;
    while (*p)
    {
        char c = *p++;

        if (c != '1' && c != '0' && c != last)
        {
            // Determine if we already saw this field earlier.
            bool found = false;
            for (size_t j = 0 ; j < i ; j++)
            {
                if (fields[j].c == c)
                    found = true;
            }

            // If this is the first time we have seen the field, then save its value in fields array.
            if (!found)
            {
                assert (i < sizeof(fields)/sizeof(fields[0]));

                fields[i].value = va_arg(valist, uint32_t);
                fields[i].c = c;
                last = c;
                i++;
            }
        }
    }

    // Go through pEncoding again from right to left and insert field bits.
    p = pEncoding + 15;
    while (p >= pEncoding)
    {
        char c = *p--;

        instr >>= 1;

        if (c == '1')
        {
            instr |= (1 << 15);
        }
        else if (c == '0')
        {
            instr |= (0 << 15);
        }
        else
        {
            size_t j;
            for (j = 0 ; j < i ; j++)
            {
                if (fields[j].c == c)
                    break;
            }
            assert (j != i);

            instr |= (fields[j].value & 1) << 15;
            fields[j].value >>= 1;
        }
    }

    armulet_write_u16(m_emitAddress, instr);
    m_emitAddress += 2;
#if OTHER_VALIST_BEHAVIOR
    return valist;
#endif
}

void pinkySimStep(PinkySimContext* pContext)
{
    cpu.primask = (bool)(pContext->PRIMASK & PRIMASK_PM);
    armulet_single_step(&cpu);
    pContext->PRIMASK = cpu.primask ? PRIMASK_PM : 0;
    // todo graham
    if (m_expectedStepReturn == PINKYSIM_STEP_SEV ||
            m_expectedStepReturn == PINKYSIM_STEP_WFE ||
            m_expectedStepReturn == PINKYSIM_STEP_WFI) {
        m_expectedStepReturn = PINKYSIM_STEP_OK;
    }
    if (m_expectedStepReturn == PINKYSIM_STEP_OK) {
        CHECK_EQUAL(cpu.step_status, ARMULET_IST_NORMAL);
    } else if (m_expectedStepReturn == PINKYSIM_STEP_BKPT) {
        CHECK_EQUAL(cpu.step_status, ARMULET_IST_BREAKPOINT);
    } else if (m_expectedStepReturn == PINKYSIM_STEP_UNDEFINED16) {
        CHECK_EQUAL(cpu.step_status, ARMULET_IST_UNDEFINED16);
    } else if (m_expectedStepReturn == PINKYSIM_STEP_UNDEFINED32) {
        CHECK_EQUAL(cpu.step_status, ARMULET_IST_UNDEFINED32);
    } else if (m_expectedStepReturn == PINKYSIM_STEP_SVC) {
        CHECK_EQUAL(cpu.step_status, ARMULET_IST_SVC);
    } else {
        printf("WARNING CAN'T CHECK ABNORMAL RETURN\n");
    }
//        int result = ::pinkySimStep(pContext);
//        CHECK_EQUAL(m_expectedStepReturn, result);
    validateXPSR();
    validateRegisters();
}

void validateXPSR()
{
    // todo graham
    CHECK_EQUAL((bool)(m_expectedXPSRflags & APSR_C), cpu.C);
    CHECK_EQUAL((bool)(m_expectedXPSRflags & APSR_V), armulet_get_V(&cpu));
    CHECK_EQUAL((bool)(m_expectedXPSRflags & APSR_N), armulet_get_N(&cpu));
    CHECK_EQUAL((bool)(m_expectedXPSRflags & APSR_Z), armulet_get_Z(&cpu));
//        CHECK_EQUAL(m_expectedXPSRflags, m_context.xPSR & (APSR_NZCV | EPSR_T));
    CHECK_EQUAL(m_expectedIPSR, cpu.ipsr & IPSR_MASK);
}

void validateRegisters()
{
    for (int i = 0 ; i < 13 ; i++)
        CHECK_EQUAL(m_expectedRegisterValues[i], cpu.regs[i]);
    CHECK_EQUAL(m_expectedSPmain, cpu.regs[SP]);
    CHECK_EQUAL(m_expectedLR, cpu.regs[LR]);
    CHECK_EQUAL(m_expectedPC, cpu.regs[PC]);
}

static void setCarry()
{
    cpu.C = true;
}

static void clearCarry()
{
    cpu.C = false;
}

void setZero()
{
    armulet_set_NZ(&cpu, false, true);
}

void clearZero()
{
    armulet_set_NZ(&cpu, armulet_get_N(&cpu), false);
}

void setNegative()
{
    armulet_set_NZ(&cpu, true, false);
}

void clearNegative()
{
    armulet_set_NZ(&cpu, false, armulet_get_Z(&cpu));
}

void setOverflow()
{
    armulet_set_V(&cpu, true);
}

void clearOverflow()
{
    armulet_set_V(&cpu, false);
}

void setIPSR(uint32_t ipsr)
{
    cpu.ipsr = ipsr & IPSR_MASK;
}

#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
void setMSPLIM(uint32_t msplim)
{
    cpu.splim = msplim;
}
#endif

template<typename T>
struct expecter
{
    explicit expecter(T t) : t(t) {}

    T t;

    void toEqual(T other) {
        if (t != other) {
            __breakpoint();
            while(true) {
                busy_wait_at_least_cycles(10000000);
            }
            printf("Mismatch: %08x != %08x\n", (int)other, (int)t);
//            std::cout << "Mismatch: " << other << " != " << t << std::endl;
#if PICO_RISCV
            asm ("ebreak");
#endif
            assert(false);
        }
    }

    void toBe(T other) {
        toEqual(other);
    }
};

std::vector<cpu_test> tests = {
/* ADC - Register (ADd with Carry)
   Encoding: 010000 0101 Rm:3 Rdn:3 */
        cpu_test("adcRegister : UseR1ForAllArgs", [] {
            emitInstruction16("0100000101mmmddd", R1, R1);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R1, 0x11111111U + 0x11111111U);
            // Carry In state is important for ADC tests.
            clearCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("adcRegister : UseLowestRegisterForAllArgs", [] {
            emitInstruction16("0100000101mmmddd", R0, R0);
            setExpectedXPSRflags("nZcv");
            setExpectedRegisterValue(R0, 0U);
            clearCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("adcRegister : UseHigestRegisterForAllArgsPositiveOverflow", [] {
            emitInstruction16("0100000101mmmddd", R7, R7);
            setExpectedXPSRflags("NzcV");
            setExpectedRegisterValue(R7, 0x77777777U + 0x77777777U);
            clearCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("adcRegister : UseDifferentRegistersForEachArg", [] {
            emitInstruction16("0100000101mmmddd", R1, R2);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R2, 0x11111111U + 0x22222222U);
            clearCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("adcRegister : Add0to0WithCarryInSetToGiveAResultOf1", [] {
            emitInstruction16("0100000101mmmddd", R0, R0);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R0, 0U + 0U + 1U);
            setCarry();
            pinkySimStep(&m_context);
        }),

// Force APSR flags to be set which haven't already been covered above.
        cpu_test("adcRegister : ForceCarryOut", [] {
            emitInstruction16("0100000101mmmddd", R1, R2);
            setExpectedXPSRflags("nZCv");
            setRegisterValue(R1, -1);
            setRegisterValue(R2, 1);
            setExpectedRegisterValue(R2, -1 + 1);
            clearCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("adcRegister : ForceCarryOutAndOverflow", [] {
            emitInstruction16("0100000101mmmddd", R1, R2);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R1, -1);
            setRegisterValue(R2, 0x80000000U);
            setExpectedRegisterValue(R2,  -1 + (uint32_t)0x80000000U);
            clearCarry();
            pinkySimStep(&m_context);
        }),

        /* ADD - Immediate - Encoding T1
   Encoding: 000 11 1 0 Imm:3 Rn:3 Rd:3 */
        cpu_test("addImmediate : T1UseLowestRegisterOnlyAddLargestImmediate", []
        {
            emitInstruction16("0001110iiinnnddd", 7, R0, R0);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R0, 0U + 7U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addImmediate : T1UseHigestRegisterOnlyAddSmallestImmediate", []
        {
            emitInstruction16("0001110iiinnnddd", 0, R7, R7);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R7, 0x77777777U + 0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addImmediate : T1UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("0001110iiinnnddd", 3, R7, R0);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R0, 0x77777777U + 3U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addImmediate : T1ForceCarryByAdding1ToLargestInteger", []
        {
            emitInstruction16("0001110iiinnnddd", 1, R6, R1);
            setExpectedXPSRflags("nZCv");
            setRegisterValue(R6, 0xFFFFFFFFU);
            setExpectedRegisterValue(R1, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("addImmediate : T1ForceOverflowPastLargestPositiveInteger", []
        {
            emitInstruction16("0001110iiinnnddd", 1, R2, R5);
            setExpectedXPSRflags("NzcV");
            setRegisterValue(R2, 0x7FFFFFFFU);
            setExpectedRegisterValue(R5, 0x7FFFFFFFU + 1);
            pinkySimStep(&m_context);
        }),



/* ADD - Immediate - Encoding T2
   Encoding: 001 10 Rdn:3 Imm:8 */
        cpu_test("addImmediate : T2UseLowestRegisterAndAddLargestImmediate", []
        {
            emitInstruction16("00110dddiiiiiiii", R0, 255);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R0, 0U + 255U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addImmediate : T2UseHigestRegisterAndAddSmallestImmediate", []
        {
            emitInstruction16("00110dddiiiiiiii", R7, 0);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R7, 0x77777777U + 0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addImmediate : T2ForceCarryByAdding1ToLargestInteger", []
        {
            emitInstruction16("00110dddiiiiiiii", R3, 1);
            setExpectedXPSRflags("nZCv");
            setRegisterValue(R3, 0xFFFFFFFFU);
            setExpectedRegisterValue(R3, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("addImmediate : T2ForceOverflowPastLargestPositiveInteger", []
        {
            emitInstruction16("00110dddiiiiiiii", R3, 1);
            setExpectedXPSRflags("NzcV");
            setRegisterValue(R3, 0x7FFFFFFFU);
            setExpectedRegisterValue(R3, 0x7FFFFFFFU + 1);
            pinkySimStep(&m_context);
        }),


/* ADD - Register - Encoding T1
   Encoding: 000 11 0 0 Rm:3 Rn:3 Rd:3 */
        cpu_test("addRegister : T1UseLowestRegisterForAllArgs", []
        {
            emitInstruction16("0001100mmmnnnddd", R0, R0, R0);
            setExpectedXPSRflags("nZcv");
            setExpectedRegisterValue(R0, 0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T1UseHigestRegisterForAllArgs", []
        {
            emitInstruction16("0001100mmmnnnddd", R7, R7, R7);
            setExpectedXPSRflags("NzcV");
            setExpectedRegisterValue(R7, 0x77777777U + 0x77777777U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T1UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("0001100mmmnnnddd", R1, R2, R3);
            setExpectedXPSRflags("nzcv");
            setExpectedRegisterValue(R3, 0x11111111U + 0x22222222U);
            pinkySimStep(&m_context);
        }),

// Force APSR flags to be set which haven't already been covered above.
        cpu_test("addRegister : T1ForceCarryWithNoOverflow", []
        {
            emitInstruction16("0001100mmmnnnddd", R1, R2, R0);
            setExpectedXPSRflags("nZCv");
            setRegisterValue(R1, -1);
            setRegisterValue(R2, 1);
            setExpectedRegisterValue(R0, -1 + 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T1ForceCarryAndOverflow", []
        {
            emitInstruction16("0001100mmmnnnddd", R1, R2, R0);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R1, -1);
            setRegisterValue(R2, 0x80000000U);
            setExpectedRegisterValue(R0, -1 + (uint32_t)0x80000000U);
            pinkySimStep(&m_context);
        }),



/* ADD - Register - Encoding T2
   Encoding: 010001 00 DN:1 Rm:4 Rdn:3
   NOTE: Shouldn't modify any of the APSR flags.*/
        cpu_test("addRegister : T2UseR1ForAllArgs", []
        {
            emitInstruction16("01000100dmmmmddd", R1, R1);
            setExpectedRegisterValue(R1, 0x11111111U + 0x11111111U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2UseLowestRegisterForAllArgs", []
        {
            emitInstruction16("01000100dmmmmddd", R0, R0);
            setExpectedRegisterValue(R0, 0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2UseR12ForAllArgs", []
        {
            emitInstruction16("01000100dmmmmddd", R12, R12);
            setExpectedRegisterValue(R12, 0xCCCCCCCCU + 0xCCCCCCCCU);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("01000100dmmmmddd", R2, R1);
            setExpectedRegisterValue(R2, 0x11111111U + 0x22222222U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2WrapAroundTo0", []
        {
            emitInstruction16("01000100dmmmmddd", R2, R1);
            setRegisterValue(R1, -1);
            setRegisterValue(R2, 1);
            setExpectedRegisterValue(R2, -1 + 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2OverflowFromLowestNegativeValue", []
        {
            emitInstruction16("01000100dmmmmddd", R11, R10);
            setRegisterValue(R10, -1);
            setRegisterValue(R11, 0x80000000U);
            setExpectedRegisterValue(R11, -1 + (uint32_t)0x80000000U);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2Add4ToSP", []
        {
            emitInstruction16("01000100dmmmmddd", SP, R1);
            setRegisterValue(SP, INITIAL_SP - 4);
            setRegisterValue(R1, 4);
            setExpectedRegisterValue(SP, INITIAL_SP - 4 + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2Subtract4FromSP", []
        {
            emitInstruction16("01000100dmmmmddd", SP, R1);
            setRegisterValue(R1, -4);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            pinkySimStep(&m_context);
        }),

#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
        cpu_test("addRegister_MSPLIM_nooverflow : T2Add-4ToSP", []
        {
            emitInstruction16("01000100dmmmmddd", SP, R1);
            setMSPLIM(INITIAL_SP - 4);
            setRegisterValue(SP, INITIAL_SP);
            setRegisterValue(R1, -4);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister_MSPLIM_overflow : T2Add-8ToSP", []
        {
            emitInstruction16("01000100dmmmmddd", SP, R1);
            setMSPLIM(INITIAL_SP - 4);
            setRegisterValue(SP, INITIAL_SP);
            setRegisterValue(R1, -8);
            setExpectedRegisterValue(SP, INITIAL_SP - 8);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT); // we do breakpoint on stack overflow
            pinkySimStep(&m_context);
        }),
#endif

        cpu_test("addRegister : T2Add1ToLR", []
        {
            emitInstruction16("01000100dmmmmddd", LR, R1);
            setRegisterValue(R1, 1);
            setExpectedRegisterValue(LR, INITIAL_LR + 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2Add1ToPCWhichWillBeOddAndRoundedDown", []
        {
            emitInstruction16("01000100dmmmmddd", PC, R1);
            setRegisterValue(R1, 1);
            setExpectedRegisterValue(PC, (INITIAL_PC + 4 + 1) & 0xFFFFFFFE);
            pinkySimStep(&m_context);
        }),

        cpu_test("addRegister : T2Add2ToPC", []
        {
            emitInstruction16("01000100dmmmmddd", PC, R1);
            setRegisterValue(R1, 2);
            setExpectedRegisterValue(PC, (INITIAL_PC + 4 + 2) & 0xFFFFFFFE);
            pinkySimStep(&m_context);
        }),

#if UNPREDICTABLE
        TEST_SIM_ONLY(addRegister, T2ItIsUnpredictableToHaveBothArgsBePC)
        {
            emitInstruction16("01000100dmmmmddd", PC, PC);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

        /* ADD SP Plus Immediate - Encoding T1
   Encoding: 1010 1 Rd:3 Imm:8 */
        cpu_test("addSP : T1UseHighestRegisterAddSmallestImmediate", []
        {
            emitInstruction16("10101dddiiiiiiii", R7, 0);
            setExpectedRegisterValue(R7, INITIAL_SP + 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("addSP : T1UseLowestRegisterAddLargestImmediate", []
        {
            emitInstruction16("10101dddiiiiiiii", R0, 255);
            setExpectedRegisterValue(R0, INITIAL_SP + 255 * 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("addSP : T1UseIntermediateValues", []
        {
            emitInstruction16("10101dddiiiiiiii", R3, 128);
            setExpectedRegisterValue(R3, INITIAL_SP + 128 * 4);
            pinkySimStep(&m_context);
        }),



/* ADD SP Plus Immediate - Encoding T2
   Encoding: 1011 0000 0 Imm:7 */
        cpu_test("addSP : T2SmallestImmediate", []
        {
            emitInstruction16("101100000iiiiiii", 0);
            setRegisterValue(SP, INITIAL_PC + 1024);
            setExpectedRegisterValue(SP, INITIAL_PC + 1024 + 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("addSP : T2LargestImmediate", []
        {
            emitInstruction16("101100000iiiiiii", 127);
            setRegisterValue(SP, INITIAL_PC + 1024);
            setExpectedRegisterValue(SP, INITIAL_PC + 1024 + 127 * 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("addSP : T2IntermediateValues", []
        {
            emitInstruction16("101100000iiiiiii", 64);
            setRegisterValue(SP, INITIAL_PC + 1024);
            setExpectedRegisterValue(SP, INITIAL_PC + 1024 + 64 * 4);
            pinkySimStep(&m_context);
        }),
/* ADR (ADDress of label)
   Encoding: 1010 0 Rd:3 Imm:8 */
        cpu_test("adr : LowestRegisterWithLargestOffset", []
        {
            emitInstruction16("10100dddiiiiiiii", R0, 255);
            setExpectedRegisterValue(R0, INITIAL_PC + 4 + 255 * 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("adr : HighesttRegisterWithSmallestOffset", []
        {
            emitInstruction16("10100dddiiiiiiii", R7, 0);
            setExpectedRegisterValue(R7, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("adr : pcWillNeedToBeWordAlignedBeforeAdd", []
        {
            // Emit UNDEFINED 16-bit instruction.
            emitInstruction16("1101111000000000");
            // Emit actual test instruction at a 2-byte aligned address which isn't 4-byte aligned.
            emitInstruction16("10100dddiiiiiiii", R3, 0);
            setRegisterValue(PC, INITIAL_PC + 2);
            setExpectedRegisterValue(R3, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        /* AND - Register
   Encoding: 010000 0000 Rm:3 Rdn:3 */
/* NOTE: APSR_C state is maintained by this instruction. */
        cpu_test("andRegister : UseLowestRegisterForBothArgs", []
        {
            emitInstruction16("0100000000mmmddd", R0, R0);
            setExpectedXPSRflags("nZc");
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            clearCarry();
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("andRegister : UseHighestRegisterForBothArgs", []
        {
            emitInstruction16("0100000000mmmddd", R7, R7);
            setExpectedXPSRflags("nzC");
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("andRegister : AndR3andR7", []
        {
            emitInstruction16("0100000000mmmddd", R3, R7);
            setExpectedXPSRflags("nz");
            setExpectedRegisterValue(R7, 0x33333333);
            pinkySimStep(&m_context);
        }),

        cpu_test("andRegister : UseAndToJustKeepNegativeSignBit", []
        {
            emitInstruction16("0100000000mmmddd", R6, R1);
            setRegisterValue(R1, -1);
            setRegisterValue(R6, 0x80000000);
            setExpectedXPSRflags("Nz");
            setExpectedRegisterValue(R1, 0x80000000);
            pinkySimStep(&m_context);
        }),

        cpu_test("andRegister : HaveAndResultNotBeSameAsEitherSource", []
        {
            emitInstruction16("0100000000mmmddd", R5, R2);
            setRegisterValue(R2, 0x12345678);
            setRegisterValue(R5, 0xF0F0F0F0);
            setExpectedXPSRflags("nz");
            setExpectedRegisterValue(R2, 0x10305070);
            pinkySimStep(&m_context);
        }),


/* ASR - Immediate (Arithmetic Shift Right)
   Encoding: 000 10 imm:5 Rm:3 Rd:3 */
        cpu_test("asrImmediate : ShiftNegativeNumberBy1_Shift0OutFromLowestBit", []
        {
            emitInstruction16("00010iiiiimmmddd", IMM_1, R0, R7);
            setRegisterValue(R0, 0x80000000U);
            setExpectedXPSRflags("Nzc");
            setExpectedRegisterValue(R7, (int32_t)0x80000000U >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrImmediate : ShiftPositiveNumberBy1_Shift1OutFromLowestBit", []
        {
            emitInstruction16("00010iiiiimmmddd", IMM_1, R7, R0);
            setRegisterValue(R7, 0x7FFFFFFFU);
            setExpectedXPSRflags("nzC");
            setExpectedRegisterValue(R0, (int32_t)0x7FFFFFFFU >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrImmediate : NegativeNumberBy32_Shift1OutFromHighestBit", []
        {
            emitInstruction16("00010iiiiimmmddd", IMM_32, R0, R0);
            setRegisterValue(R0, 0x80000000U);
            setExpectedXPSRflags("NzC");
            setExpectedRegisterValue(R0, 0xFFFFFFFFU);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrImmediate : PositiveNumberBy32_Shift0FromHighestBit", []
        {
            emitInstruction16("00010iiiiimmmddd", IMM_32, R1, R6);
            setRegisterValue(R1, 0x7FFFFFFFU);
            setExpectedXPSRflags("nZc");
            setExpectedRegisterValue(R6, 0x0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrImmediate : R1by1ToR7", []
        {
            emitInstruction16("00010iiiiimmmddd", IMM_1, R1, R7);
            setExpectedXPSRflags("nzC");
            setExpectedRegisterValue(R7, (int32_t)0x11111111U >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrImmediate : R7by1ToR2", []
        {
            emitInstruction16("00010iiiiimmmddd", IMM_1, R7, R2);
            setExpectedXPSRflags("nzC");
            setExpectedRegisterValue(R2, (int32_t)0x77777777U >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrImmediate : R0by1", []
        {
            emitInstruction16("00010iiiiimmmddd", IMM_1, R0, R0);
            setExpectedXPSRflags("nZc");
            setExpectedRegisterValue(R0, (int32_t)0x00000000U >> 1);
            pinkySimStep(&m_context);
        }),

/* ASR - Register (Arithmetic Shift Right)
   Encoding: 010000 0100 Rm:3 Rdn:3 */
        cpu_test("asrRegister : Shift1by1_CarryOutFromLowestBit", []
        {
            emitInstruction16("0100000100mmmddd", R0, R7);
            setRegisterValue(R7, 1);
            setRegisterValue(R0, 1);
            setExpectedXPSRflags("nZC");
            setExpectedRegisterValue(R7, (int32_t)1U >> 1U);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : Shift1by0_MinimumShift_CarryUnmodified", []
        {
            emitInstruction16("0100000100mmmddd", R0, R7);
            setRegisterValue(R7, 1);
            setRegisterValue(R0, 0);
            setExpectedXPSRflags("nz");
            setExpectedRegisterValue(R7, (int32_t)1 >> 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : Shift2by1_NoCarryFromLowestBit", []
        {
            emitInstruction16("0100000100mmmddd", R3, R2);
            setRegisterValue(R2, 2);
            setRegisterValue(R3, 1);
            setExpectedXPSRflags("nzc");
            setExpectedRegisterValue(R2, (int32_t)2 >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : ShiftNegativeNumberby31", []
        {
            emitInstruction16("0100000100mmmddd", R3, R2);
            setRegisterValue(R2, -1);
            setRegisterValue(R3, 31);
            setExpectedXPSRflags("NzC");
            setExpectedRegisterValue(R2, (int32_t)-1 >> 31);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : ShiftMaximumNegativeValueBy32_CarryOutFromHighestBit", []
        {
            emitInstruction16("0100000100mmmddd", R7, R0);
            setRegisterValue(R0, 0x80000000);
            setRegisterValue(R7, 32);
            setExpectedXPSRflags("NzC");
            setExpectedRegisterValue(R0, -1);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : ShiftNegativeValueby33", []
        {
            emitInstruction16("0100000100mmmddd", R3, R2);
            setRegisterValue(R2, -1);
            setRegisterValue(R3, 33);
            setExpectedXPSRflags("NzC");
            setExpectedRegisterValue(R2, -1);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : ShiftPositiveValueby33", []
        {
            emitInstruction16("0100000100mmmddd", R3, R2);
            setRegisterValue(R2, 0x7FFFFFFF);
            setRegisterValue(R3, 33);
            setExpectedXPSRflags("nZc");
            setExpectedRegisterValue(R2, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : ShiftNegativeValueByMaximumShiftOf255", []
        {
            emitInstruction16("0100000100mmmddd", R3, R2);
            setRegisterValue(R2, -1);
            setRegisterValue(R3, 255);
            setExpectedXPSRflags("NzC");
            setExpectedRegisterValue(R2, -1);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : ShiftOf256ShouldBeTreatedAsShiftOf0_CarryUnmodified", []
        {
            emitInstruction16("0100000100mmmddd", R7, R0);
            setRegisterValue(R0, -1);
            setRegisterValue(R7, 256);
            setExpectedXPSRflags("Nz");
            setExpectedRegisterValue(R0, (int32_t)-1 >> 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : ShiftLargestPositiveNumberBy31", []
        {
            emitInstruction16("0100000100mmmddd", R2, R3);
            setRegisterValue(R3, 0x7FFFFFFF);
            setRegisterValue(R2, 31);
            setExpectedXPSRflags("nZC");
            setExpectedRegisterValue(R3, (int32_t)0x7FFFFFFF >> 31);
            pinkySimStep(&m_context);
        }),

        cpu_test("asrRegister : ShiftLargestNegativeNumberBy1", []
        {
            emitInstruction16("0100000100mmmddd", R2, R3);
            setRegisterValue(R3, 0x80000000);
            setRegisterValue(R2, 1);
            setExpectedXPSRflags("Nzc");
            setExpectedRegisterValue(R3, (int32_t)0x80000000 >> 1);
            pinkySimStep(&m_context);
        }),


/* BIC - Register
   Encoding: 010000 1110 Rm:3 Rdn:3 */
/* NOTE: APSR_C state is maintained by this instruction. */
        cpu_test("bicRegister : UseLowestRegisterForBothArgs", []
        {
            emitInstruction16("0100001110mmmddd", R0, R0);
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            setExpectedXPSRflags("nZc");
            clearCarry();
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("bicRegister : UseHighestRegisterForBothArgs", []
        {
            emitInstruction16("0100001110mmmddd", R7, R7);
            setExpectedXPSRflags("nZC");
            setCarry();
            setExpectedRegisterValue(R7, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("bicRegister : UseR3andR7", []
        {
            emitInstruction16("0100001110mmmddd", R3, R7);
            setExpectedXPSRflags("nz");
            setExpectedRegisterValue(R7, 0x77777777 & ~0x33333333);
            pinkySimStep(&m_context);
        }),

        cpu_test("bicRegister : UseBicToClearLSbit", []
        {
            emitInstruction16("0100001110mmmddd", R6, R1);
            setRegisterValue(R1, -1);
            setRegisterValue(R6, 1);
            setExpectedXPSRflags("Nz");
            setExpectedRegisterValue(R1, -1U & ~1);
            pinkySimStep(&m_context);
        }),

/* BKPT
   Encoding: 1011 1110 Imm:8 */
/* NOTE: Simulator behaviour is good.  On real hardware, leads to multiple debug events: breakpoint & single step. */
        cpu_test("bkpt : SmallestImmediate", []
        {
            emitInstruction16("10111110iiiiiiii", 0);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT);
            pinkySimStep(&m_context);
        }),

        cpu_test("bkpt : LargestImmediate", []
        {
            emitInstruction16("10111110iiiiiiii", 255);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT);
            pinkySimStep(&m_context);
        }),

        /* BL (Branch with Link)
   Encoding: 11110 S Imm:10
             11 J1:1 1 J2:1 Imm:11
    Note: J1 and J2 are translated to immediate bits via I? = NOT(J? XOR S) */

        cpu_test("bl : OffsetOf0", []
        {
            emitInstruction32("11110Siiiiiiiiii", "11j1kiiiiiiiiiii", 0, 0, 1, 1, 0);
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            setExpectedRegisterValue(LR, (INITIAL_PC + 4) | 1);
            pinkySimStep(&m_context);
        }),

#if !VARMULET_USE_EARLY_INSTR_READ
        cpu_test("bl : MaximumPositiveOffset", []
        {
            emitInstruction32("11110Siiiiiiiiii", "11j1kiiiiiiiiiii", 0, 0x3FF, 0, 0, 0x7FF);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 + 16777214);
            setExpectedRegisterValue(LR, (INITIAL_PC + 4) | 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("bl : MaximumNegativeOffset", []
        {
            emitInstruction32("11110Siiiiiiiiii", "11j1kiiiiiiiiiii", 1, 0, 0, 0, 0);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 - 16777216);
            setExpectedRegisterValue(LR, (INITIAL_PC + 4) | 1);
            pinkySimStep(&m_context);
        }),
#endif

/* BLX (Branch with Link and Exchange)
   Encoding: 010001 11 1 Rm:4 (0)(0)(0) */
#if HARD_FAULT
        cpu_test("blx : UseLowestRegisterToBranchToEvenAddressWhichClearsThumbModeToCauseHardFaultOnNextInstruction", []
        {
            emitInstruction16("010001111mmmm000", R0);
            setExpectedXPSRflags("t");
            setRegisterValue(R0, INITIAL_PC + 16);
            setExpectedRegisterValue(PC, INITIAL_PC + 16);
            setExpectedRegisterValue(LR, (INITIAL_PC + 2) | 1);
            pinkySimStep(&m_context);

            const uint16_t NOP = 0xBF00;
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16, NOP, READ_ONLY);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            pinkySimStep(&m_context);
        }),
#endif

        cpu_test("blx : UseHighestRegisterToBranchToOddAddressAsRequiredForThumb", []
        {
            emitInstruction16("010001111mmmm000", LR);
            setRegisterValue(LR, (INITIAL_PC + 16) | 1);
            setExpectedRegisterValue(PC, INITIAL_PC + 16);
            setExpectedRegisterValue(LR, (INITIAL_PC + 2) | 1);
            pinkySimStep(&m_context);
        }),

#if UNPREDICTABLE
        TEST_SIM_ONLY(blx, UnpredictableToUseR15)
        {
            emitInstruction16("010001111mmmm000", PC);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(blx, UnpredictableForBit0ToBeHigh)
        {
            emitInstruction16("010001111mmmm001", R0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(blx, UnpredictableForBit1ToBeHigh)
        {
            emitInstruction16("010001111mmmm010", R0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(blx, UnpredictableForBit2ToBeHigh)
        {
            emitInstruction16("010001111mmmm100", R0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif


/* B - Encoding T1 (Conditional)
   Encoding: 1101 Cond:4 Imm:8 */
        cpu_test("b : BEQ_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_EQ, 0);
            // These tests set the APSR flags to specific value and expect them to be unmodified upon return.
            setExpectedXPSRflags("z");
            clearZero();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BEQ_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_EQ, 0);
            setExpectedXPSRflags("Z");
            setZero();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BNE_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_NE, 0);
            setExpectedXPSRflags("Z");
            setZero();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BNE_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_NE, 0);
            setExpectedXPSRflags("z");
            clearZero();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BCS_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_CS, 0);
            setExpectedXPSRflags("c");
            clearCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BCS_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_CS, 0);
            setExpectedXPSRflags("C");
            setCarry();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BCC_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_CC, 0);
            setExpectedXPSRflags("C");
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BCC_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_CC, 0);
            setExpectedXPSRflags("c");
            clearCarry();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BMI_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_MI, 0);
            setExpectedXPSRflags("n");
            clearNegative();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BMI_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_MI, 0);
            setExpectedXPSRflags("N");
            setNegative();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BPL_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_PL, 0);
            setExpectedXPSRflags("N");
            setNegative();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BPL_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_PL, 0);
            setExpectedXPSRflags("n");
            clearNegative();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BVS_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_VS, 0);
            setExpectedXPSRflags("v");
            clearOverflow();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BVS_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_VS, 0);
            setExpectedXPSRflags("V");
            setOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BVC_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_VC, 0);
            setExpectedXPSRflags("V");
            setOverflow();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BVC_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_VC, 0);
            setExpectedXPSRflags("v");
            clearOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BHI_NotTaken1", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_HI, 0);
            setExpectedXPSRflags("cz");
            clearCarry(); clearZero();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BHI_NotTaken2", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_HI, 0);
            setExpectedXPSRflags("cZ");
            clearCarry(); setZero();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BHI_NotTaken3", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_HI, 0);
            setExpectedXPSRflags("CZ");
            setCarry(); setZero();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BHI_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_HI, 0);
            setExpectedXPSRflags("Cz");
            setCarry(); clearZero();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLS_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LS, 0);
            setExpectedXPSRflags("Cz");
            setCarry(); clearZero();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLS_Taken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LS, 0);
            setExpectedXPSRflags("cZ");
            clearCarry(); setZero();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BGE_NotTaken1", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GE, 0);
            setExpectedXPSRflags("Nv");
            setNegative(); clearOverflow();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BGE_NotTaken2", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GE, 0);
            setExpectedXPSRflags("nV");
            clearNegative(); setOverflow();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BGE_Taken1", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GE, 0);
            setExpectedXPSRflags("NV");
            setNegative(); setOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BGE_Taken2", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GE, 0);
            setExpectedXPSRflags("nv");
            clearNegative(); clearOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLT_NotTaken", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LT, 0);
            setExpectedXPSRflags("NV");
            setNegative(); setOverflow();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLT_Taken1", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LT, 0);
            setExpectedXPSRflags("Nv");
            setNegative(); clearOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLT_Taken2", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LT, 0);
            setExpectedXPSRflags("nV");
            clearNegative(); setOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

#if NEEDS_NZ
        cpu_test("b : BGT_NotTaken1", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GT, 0);
            setExpectedXPSRflags("ZNV");
            setZero(); setNegative(); setOverflow();
            pinkySimStep(&m_context);
        }),
#endif

        cpu_test("b : BGT_NotTaken2", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GT, 0);
            setExpectedXPSRflags("znV");
            clearZero(); clearNegative(); setOverflow();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BGT_NotTaken3", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GT, 0);
            setExpectedXPSRflags("Znv");
            setZero(); clearNegative(); clearOverflow();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BGT_Taken1", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GT, 0);
            setExpectedXPSRflags("znv");
            clearZero(); clearNegative(); clearOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BGT_Taken2", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_GT, 0);
            setExpectedXPSRflags("zNV");
            clearZero(); setNegative(); setOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLE_NotTaken1", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LE, 0);
            setExpectedXPSRflags("znv");
            clearZero(); clearNegative(); clearOverflow();
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLE_NotTaken2", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LE, 0);
            setExpectedXPSRflags("zNV");
            clearZero(); setNegative(); setOverflow();
            pinkySimStep(&m_context);
        }),

#if NEEDS_NZ
        cpu_test("b : BLE_Taken1", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LE, 0);
            setExpectedXPSRflags("ZNv");
            setZero(); setNegative(); clearOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),
#endif

        cpu_test("b : BLE_Taken2", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LE, 0);
            setExpectedXPSRflags("zNv");
            clearZero(); setNegative(); clearOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLE_Taken3", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LE, 0);
            setExpectedXPSRflags("ZnV");
            setZero(); clearNegative(); setOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLE_Taken4", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LE, 0);
            setExpectedXPSRflags("znV");
            clearZero(); clearNegative(); setOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BLE_Taken5", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LE, 0);
            setExpectedXPSRflags("Znv");
            setZero(); clearNegative(); clearOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

#if NEEDS_NZ
        cpu_test("b : BLE_Taken6", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_LE, 0);
            setExpectedXPSRflags("ZNV");
            setZero(); setNegative(); setOverflow();
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),
#endif

        cpu_test("b : BEQ_TakenWithLargestPositiveOffset", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_EQ, 127);
            setExpectedXPSRflags("Z");
            setZero();
            setExpectedRegisterValue(PC, INITIAL_PC + 4 + 127 * 2);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BEQ_TakenWithLargesNegativeOffset", []
        {
            emitInstruction16("1101cccciiiiiiii", COND_EQ, -128);
            setExpectedXPSRflags("Z");
            setZero();
            setExpectedRegisterValue(PC, INITIAL_PC + 4 - 128 * 2);
            pinkySimStep(&m_context);
        }),



/* B - Encoding T2 (Unconditional)
   Encoding: 11100 Imm:11 */
        cpu_test("b : BAL_ZeroOffset", []
        {
            emitInstruction16("11100iiiiiiiiiii", 0);
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BAL_LargestPositiveOffset", []
        {
            emitInstruction16("11100iiiiiiiiiii", 1023);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 + 1023 * 2);
            pinkySimStep(&m_context);
        }),

        cpu_test("b : BAL_LargestNegativeOffset", []
        {
            emitInstruction16("11100iiiiiiiiiii", -1024);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 - 1024 * 2);
            pinkySimStep(&m_context);
        }),

#if ARMULET_FEATURE_ARMV8M_BASELINE_BW
        cpu_test("b.w : B.W_OffsetOf0", []
        {
            emitInstruction32("11110Siiiiiiiiii", "10j1kiiiiiiiiiii", 0, 0, 1, 1, 0);
            setExpectedRegisterValue(PC, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),
        cpu_test("b.w : B.W_LargestPositiveOffset", []
        {
            emitInstruction32("11110Siiiiiiiiii", "10j1kiiiiiiiiiii", 0, 0x3FF, 0, 0, 0x7FF);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 + 16777214);
            pinkySimStep(&m_context);
        }),

        cpu_test("b.w : B.W_LargestNegativeOffset", []
        {
            emitInstruction32("11110Siiiiiiiiii", "10j1kiiiiiiiiiii", 1, 0, 0, 0, 0);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 - 16777216);
            pinkySimStep(&m_context);
        }),
#endif
        // SKIPPED breakpointTest.cpp

#if HARD_FAULT
        /* BX (Branch and Exchange)
   Encoding: 010001 11 0 Rm:4 (0)(0)(0) */
        cpu_test("bx : UseLowestRegisterToBranchToEvenAddressWhichClearsThumbModeToCauseHardFaultOnNextInstruction", []
        {
            emitInstruction16("010001110mmmm000", R0);
            setExpectedXPSRflags("t");
            setRegisterValue(R0, INITIAL_PC + 16);
            setExpectedRegisterValue(PC, INITIAL_PC + 16);
            pinkySimStep(&m_context);

            const uint16_t NOP = 0xBF00;
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16, NOP, READ_ONLY);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            pinkySimStep(&m_context);
        }),
#endif

        cpu_test("bx : UseHighestRegisterToBranchToOddAddressWhichIsRequiredForThumb", []
        {
            emitInstruction16("010001110mmmm000", LR);
            setRegisterValue(LR, (INITIAL_PC + 16) | 1);
            setExpectedRegisterValue(PC, INITIAL_PC + 16);
            pinkySimStep(&m_context);
        }),

#if UNPREDICTABLE
        TEST_SIM_ONLY(bx, UnpredictableToUseR15)
        {
            emitInstruction16("010001110mmmm000", PC);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(bx, UnpredictableForBit0ToBeHigh)
        {
            emitInstruction16("010001110mmmm001", R0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(bx, UnpredictableForBit1ToBeHigh)
        {
            emitInstruction16("010001110mmmm010", R0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(bx, UnpredictableForBit2ToBeHigh)
        {
            emitInstruction16("010001110mmmm100", R0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

#if ARMULET_FEATURE_ARMV8M_BASELINE_CBZ_CBNZ
        cpu_test("cbnz : NotTaken", []
        {
            setRegisterValue(R2, 0);
            emitInstruction16("101110i1jjjjjnnn", 1, 31, R2);
            pinkySimStep(&m_context);
        }),

        cpu_test("cbnz : Taken Small", []
        {
            setRegisterValue(R2, 123);
            emitInstruction16("101110i1jjjjjnnn", 0, 6, R2);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 + 2 * 6);
            pinkySimStep(&m_context);
        }),

        cpu_test("cbnz : Taken Big", []
        {
            setRegisterValue(R2, 123);
            emitInstruction16("101110i1jjjjjnnn", 1, 31, R2);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 + 2 * 63);
            pinkySimStep(&m_context);
        }),

        cpu_test("cbz : NotTaken", []
        {
            setRegisterValue(R2, 123);
            emitInstruction16("101100i1jjjjjnnn", 1, 31, R2);
            pinkySimStep(&m_context);
        }),

        cpu_test("cbz : Taken Small", []
        {
            setRegisterValue(R2, 0);
            emitInstruction16("101100i1jjjjjnnn", 0, 6, R2);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 + 2 * 6);
            pinkySimStep(&m_context);
        }),

        cpu_test("cbz : Taken Big", []
        {
            setRegisterValue(R2, 0);
            emitInstruction16("101100i1jjjjjnnn", 1, 31, R2);
            setExpectedRegisterValue(PC, INITIAL_PC + 4 + 2 * 63);
            pinkySimStep(&m_context);
        }),

#endif

/* CMN - Register (Compare Negative)
   Encoding: 010000 1011 Rm:3 Rn:3 */
        cpu_test("cmnRegister : UseLowestRegisterForAllArgs", []
        {
            emitInstruction16("0100001011mmmnnn", R0, R0);
            setExpectedXPSRflags("nZcv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmnRegister : UseHighestRegisterForAllArgs", []
        {
            emitInstruction16("0100001011mmmnnn", R7, R7);
            setExpectedXPSRflags("NzcV");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmnRegister : UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("0100001011mmmnnn", R1, R2);
            setExpectedXPSRflags("nzcv");
            pinkySimStep(&m_context);
        }),

// Force APSR flags to be set which haven't already been covered above.
        cpu_test("cmnRegister : ForceCarryWithNoOverflow", []
        {
            emitInstruction16("0100001011mmmnnn", R1, R2);
            setExpectedXPSRflags("nZCv");
            setRegisterValue(R1, -1);
            setRegisterValue(R2, 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("cmnRegister : ForceCarryAndOverflow", []
        {
            emitInstruction16("0100001011mmmnnn", R1, R2);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R1, -1);
            setRegisterValue(R2, 0x80000000U);
            pinkySimStep(&m_context);
        }),


/* CMP - Immediate
   Encoding: 001 01 Rn:3 Imm:8 */
        cpu_test("cmpImmediate : CompareLowestRegisterToEqualValue", []
        {
            emitInstruction16("00101nnniiiiiiii", R0, 0);
            setExpectedXPSRflags("nZCv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpImmediate : CompareHighestRegisterToImmediateWhichIsSmaller", []
        {
            emitInstruction16("00101nnniiiiiiii", R7, 127);
            setExpectedXPSRflags("nzCv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpImmediate : CompareRegisterToLargestImmediateWhichIsLarger", []
        {
            emitInstruction16("00101nnniiiiiiii", R0, 255);
            setExpectedXPSRflags("Nzcv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpImmediate : CompareRegisterToImmediateWhichWillGenerateNegativeOverflow", []
        {
            emitInstruction16("00101nnniiiiiiii", R3, 1);
            setRegisterValue(R3, 0x80000000);
            setExpectedXPSRflags("nzCV");
            pinkySimStep(&m_context);
        }),


/* CMP - Register - Encoding T1
   Encoding: 010000 1010 Rm:3 Rn:3 */
        cpu_test("cmpRegister : T1UseLowestRegisterForAllArgs", []
        {
            emitInstruction16("0100001010mmmnnn", R0, R0);
            setExpectedXPSRflags("nZCv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T1UseHigestRegisterForAllArgs", []
        {
            emitInstruction16("0100001010mmmnnn", R7, R7);
            setExpectedXPSRflags("nZCv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T1RnLargerThanRm", []
        {
            emitInstruction16("0100001010mmmnnn", R1, R2);
            setExpectedXPSRflags("nzCv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T1RnSmallerThanRm", []
        {
            emitInstruction16("0100001010mmmnnn", R1, R0);
            setExpectedXPSRflags("Nzcv");
            setRegisterValue(R1, 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T1ForceNegativeOverflow", []
        {
            emitInstruction16("0100001010mmmnnn", R1, R2);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R2, 0x80000000U);
            setRegisterValue(R1, 1U);
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T1ForcePositiveOverflow", []
        {
            emitInstruction16("0100001010mmmnnn", R1, R2);
            setExpectedXPSRflags("NzcV");
            setRegisterValue(R2, 0x7FFFFFFFU);
            setRegisterValue(R1, -1U);
            pinkySimStep(&m_context);
        }),



/* CMP - Register - Encoding T2
   Encoding: 010001 01 N:1 Rm:4 Rn:3
   NOTE: At least one register must be high register, R8 - R14. */
        cpu_test("cmpRegister : T2CompareLowestRegisterToHighestRegister", []
        {
            emitInstruction16("01000101nmmmmnnn", R0, LR);
            setRegisterValue(LR, 0xEEEEEEEE);
            setExpectedXPSRflags("nzcv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T2CompareHighestRegisterToLowestRegister", []
        {
            emitInstruction16("01000101nmmmmnnn", LR, R0);
            setRegisterValue(LR, 0xEEEEEEEE);
            setExpectedXPSRflags("NzCv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T2CompareR8ToItself", []
        {
            emitInstruction16("01000101nmmmmnnn", R8, R8);
            setExpectedXPSRflags("nZCv");
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T2ForceNegativeOverflow", []
        {
            emitInstruction16("01000101nmmmmnnn", R11, R12);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R11, 0x80000000U);
            setRegisterValue(R12, 1U);
            pinkySimStep(&m_context);
        }),

        cpu_test("cmpRegister : T2ForcePositiveOverflow", []
        {
            emitInstruction16("01000101nmmmmnnn", R11, R12);
            setExpectedXPSRflags("NzcV");
            setRegisterValue(R11, 0x7FFFFFFFU);
            setRegisterValue(R12, -1U);
            pinkySimStep(&m_context);
        }),

#if UNPREDICTABLE
        TEST_SIM_ONLY(cmpRegister, T2UnpredictableForBothArgsToBeLowRegisters)
        {
            emitInstruction16("01000101nmmmmnnn", R6, R7);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(cmpRegister, T2UnpredictableForRnToBeR15)
        {
            emitInstruction16("01000101nmmmmnnn", PC, R8);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(cmpRegister, T2UnpredictableForRmToBeR15)
        {
            emitInstruction16("01000101nmmmmnnn", R8, PC);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

        /* CPS
   Encoding: 1011 0110 011 im:1 (0)(0)(1)(0) */
        cpu_test("cps : InterruptEnable", []
        {
            emitInstruction16("10110110011i0010", 0);
            m_context.PRIMASK |= PRIMASK_PM;
            pinkySimStep(&m_context);
            CHECK_FALSE(m_context.PRIMASK & PRIMASK_PM);
        }),

        cpu_test("cps : InterruptDisable", []
        {
            emitInstruction16("10110110011i0010", 1);
            m_context.PRIMASK &= ~PRIMASK_PM;
            pinkySimStep(&m_context);
            CHECK_TRUE(m_context.PRIMASK & PRIMASK_PM);
        }),

#if UNPREDICTABLE
        TEST_SIM_ONLY(cps, UnpredictableBecauseOfBit0)
        {
            emitInstruction16("10110110011i0011", 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(cps, UnpredictableBecauseOfBit1)
        {
            emitInstruction16("10110110011i0000", 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
        TEST_SIM_ONLY(cps, UnpredictableBecauseOfBit2)
        {
            emitInstruction16("10110110011i0110", 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(cps, UnpredictableBecauseOfBit3)
        {
            emitInstruction16("10110110011i1010", 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif
        // SKIPPED breakpointTest.cpp
        cpu_test("dmb : OptionSetTo15", []
        {
            emitInstruction32("1111001110111111", "100011110101oooo", 15);
            pinkySimStep(&m_context);
        }),

        cpu_test("dmb : OptionSetTo0", []
        {
            emitInstruction32("1111001110111111", "100011110101oooo", 0);
            pinkySimStep(&m_context);
        }),
#if UNPREDICTABLE
        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit1_0)
        {
            emitInstruction32("1111001110111110", "100011110101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit1_1)
        {
            emitInstruction32("1111001110111101", "100011110101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit1_2)
        {
            emitInstruction32("1111001110111011", "100011110101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit1_3)
        {
            emitInstruction32("1111001110110111", "100011110101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit2_8)
        {
            emitInstruction32("1111001110111111", "100011100101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit2_9)
        {
            emitInstruction32("1111001110111111", "100011010101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit2_10)
        {
            emitInstruction32("1111001110111111", "100010110101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit2_11)
        {
            emitInstruction32("1111001110111111", "100001110101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dmb, UnpredictableBecauseOfBit2_13)
        {
            emitInstruction32("1111001110111111", "101011110101oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* DSB
   Encoding: 11110 0 111 01 1 (1)(1)(1)(1)
             10 (0) 0 (1)(1)(1)(1) 0100 option:4 */
        cpu_test("dsb : OptionSetTo15", []
        {
            emitInstruction32("1111001110111111", "100011110100oooo", 15);
            pinkySimStep(&m_context);
        }),

        cpu_test("dsb : OptionSetTo0", []
        {
            emitInstruction32("1111001110111111", "100011110100oooo", 0);
            pinkySimStep(&m_context);
        }),

#if UNPREDICTABLE
        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit1_0)
        {
            emitInstruction32("1111001110111110", "100011110100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit1_1)
        {
            emitInstruction32("1111001110111101", "100011110100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit1_2)
        {
            emitInstruction32("1111001110111011", "100011110100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit1_3)
        {
            emitInstruction32("1111001110110111", "100011110100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit2_8)
        {
            emitInstruction32("1111001110111111", "100011100100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit2_9)
        {
            emitInstruction32("1111001110111111", "100011010100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit2_10)
        {
            emitInstruction32("1111001110111111", "100010110100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit2_11)
        {
            emitInstruction32("1111001110111111", "100001110100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(dsb, UnpredictableBecauseOfBit2_13)
        {
            emitInstruction32("1111001110111111", "101011110100oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* EOR - Register
   Encoding: 010000 0001 Rm:3 Rdn:3 */
/* NOTE: APSR_C state is maintained by this instruction. */
        cpu_test("eorRegister : UseLowestRegisterForBothArgs", []
        {
            emitInstruction16("0100000001mmmddd", R0, R0);
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            setExpectedXPSRflags("nZc");
            clearCarry();
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("eorRegister : UseHighestRegisterForBothArgs", []
        {
            emitInstruction16("0100000001mmmddd", R7, R7);
            setExpectedXPSRflags("nZC");
            setCarry();
            setExpectedRegisterValue(R7, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("eorRegister : XorR3andR7", []
        {
            emitInstruction16("0100000001mmmddd", R3, R7);
            setExpectedXPSRflags("nzc");
            setExpectedRegisterValue(R7, 0x33333333 ^ 0x77777777);
            clearCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("eorRegister : UseXorToJustFlipNegativeSignBitOn", []
        {
            emitInstruction16("0100000001mmmddd", R6, R3);
            setRegisterValue(R6, 0x80000000);
            setExpectedXPSRflags("NzC");
            setExpectedRegisterValue(R3, 0x33333333 ^ 0x80000000);
            setCarry();
            pinkySimStep(&m_context);
        }),

        /* ISB
   Encoding: 11110 0 111 01 1 (1)(1)(1)(1)
             10 (0) 0 (1)(1)(1)(1) 0110 option:4 */
        cpu_test("isb : OptionSetTo15", []
        {
            emitInstruction32("1111001110111111", "100011110110oooo", 15);
            pinkySimStep(&m_context);
        }),

        cpu_test("isb : OptionSetTo0", []
        {
            emitInstruction32("1111001110111111", "100011110110oooo", 0);
            pinkySimStep(&m_context);
        }),
#if UNPREDICTABLE
        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit1_0)
        {
            emitInstruction32("1111001110111110", "100011110110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit1_1)
        {
            emitInstruction32("1111001110111101", "100011110110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit1_2)
        {
            emitInstruction32("1111001110111011", "100011110110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit1_3)
        {
            emitInstruction32("1111001110110111", "100011110110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit2_8)
        {
            emitInstruction32("1111001110111111", "100011100110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit2_9)
        {
            emitInstruction32("1111001110111111", "100011010110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit2_10)
        {
            emitInstruction32("1111001110111111", "100010110110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit2_11)
        {
            emitInstruction32("1111001110111111", "100001110110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(isb, UnpredictableBecauseOfBit2_13)
        {
            emitInstruction32("1111001110111111", "101011110110oooo", 15);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* LDM
   Encoding: 1100 1 Rn:3 RegisterList:8 */
        cpu_test("ldm : JustPopR0WithR7AsAddress_WritebackNewAddressToR7", []
        {
            emitInstruction16("11001nnnrrrrrrrr", R7, (1 << 0));
            setRegisterValue(R7, INITIAL_PC + 16);
            setExpectedRegisterValue(R7, INITIAL_PC + 16 + 1 * 4);
            setExpectedRegisterValue(R0, 0xFFFFFFFF);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16, 0xFFFFFFFF, READ_ONLY);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldm : JustPopR7WithR0AsAddress_WritebackNewAddressToR0", []
        {
            emitInstruction16("11001nnnrrrrrrrr", R0, (1 << 7));
            setRegisterValue(R0, INITIAL_PC + 16);
            setExpectedRegisterValue(R0, INITIAL_PC + 16 + 1 * 4);
            setExpectedRegisterValue(R7, 0xFFFFFFFF);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16, 0xFFFFFFFF, READ_ONLY);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldm : PopAllNoWriteback", []
        {
            emitInstruction16("11001nnnrrrrrrrr", R0, 0xFF);
            setRegisterValue(R0, INITIAL_PC + 16);
            setExpectedRegisterValue(R0, 0);
            setExpectedRegisterValue(R1, 1);
            setExpectedRegisterValue(R2, 2);
            setExpectedRegisterValue(R3, 3);
            setExpectedRegisterValue(R4, 4);
            setExpectedRegisterValue(R5, 5);
            setExpectedRegisterValue(R6, 6);
            setExpectedRegisterValue(R7, 7);
            for (int i = 0 ; i < 8 ; i++)
                SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16 + 4 * i, i, READ_ONLY);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldm : PopAllButAddressRegister_WritebackNewAddress", []
        {
            emitInstruction16("11001nnnrrrrrrrr", R7, 0x7F);
            setRegisterValue(R7, INITIAL_PC + 16);
            setExpectedRegisterValue(R0, 0);
            setExpectedRegisterValue(R1, 1);
            setExpectedRegisterValue(R2, 2);
            setExpectedRegisterValue(R3, 3);
            setExpectedRegisterValue(R4, 4);
            setExpectedRegisterValue(R5, 5);
            setExpectedRegisterValue(R6, 6);
            setExpectedRegisterValue(R7, INITIAL_PC + 16 + 7 * 4);
            for (int i = 0 ; i < 7 ; i++)
                SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16 + 4 * i, i, READ_ONLY);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT
        cpu_test("ldm : HardFaultFromInvalidMemoryRead", []
        {
            emitInstruction16("11001nnnrrrrrrrr", 0, (1 << 0));
            setRegisterValue(R0, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif
#if UNPREDICTABLE
        TEST_SIM_ONLY(ldm, UnpredictableToPopNoRegisters)
        {
            emitInstruction16("11001nnnrrrrrrrr", 0, 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* LDRB - Immediate
   Encoding: 011 1 1 Imm:5 Rn:3 Rt:3 */
        cpu_test("ldrbImmediate : UseAMixOfRegistersWordAligned", []
        {
            emitInstruction16("01111iiiiinnnttt", 0, R7, R0);
            setRegisterValue(R7, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbImmediate : UseAnotherMixOfRegistersSecondByteInWord", []
        {
            emitInstruction16("01111iiiiinnnttt", 1, R0, R7);
            setRegisterValue(R0, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xFE);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbImmediate : YetAnotherMixOfRegistersThirdByteInWord", []
        {
            emitInstruction16("01111iiiiinnnttt", 2, R1, R4);
            setRegisterValue(R1, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R4, 0xAD);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbImmediate : YetAnotherMixOfRegistersFourthByteInWord", []
        {
            emitInstruction16("01111iiiiinnnttt", 3, R2, R5);
            setRegisterValue(R2, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R5, 0xBA);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbImmediate : UseLargestOffset", []
        {
            emitInstruction16("01111iiiiinnnttt", 31, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 28, 0x12345678, READ_ONLY);
            setExpectedRegisterValue(R0, 0x12);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbImmediate : LoadAPositiveValue", []
        {
            emitInstruction16("01111iiiiinnnttt", 0, R3, R0);
            setRegisterValue(R3, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xFFFFFF7F, READ_ONLY);
            setExpectedRegisterValue(R0, 0x7F);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT
        cpu_test("ldrbImmediate : AttemptLoadInvalidAddress", []
        {
            emitInstruction16("01111iiiiinnnttt", 0, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

        /* LDRB - Register
   Encoding: 0101 110 Rm:3 Rn:3 Rt:3 */
        cpu_test("ldrbRegister : UseAMixOfRegistersWordAligned", []
        {
            emitInstruction16("0101110mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbRegister : UseAnotherMixOfRegistersSecondByteInWord", []
        {
            emitInstruction16("0101110mmmnnnttt", R1, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            setRegisterValue(R1, 5);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xFE);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbRegister : YetAnotherMixOfRegistersThirdByteInWord", []
        {
            emitInstruction16("0101110mmmnnnttt", R0, R7, R4);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 6);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R4, 0xAD);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbRegister : YetAnotherMixOfRegistersFourthByteInWord", []
        {
            emitInstruction16("0101110mmmnnnttt", R0, R7, R5);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 7);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R5, 0xBA);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrbRegister : LoadAPositiveValue", []
        {
            emitInstruction16("0101110mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xFFFFFF7F, READ_ONLY);
            setExpectedRegisterValue(R0, 0x7F);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT
        cpu_test("ldrbRegister : AttemptLoadInvalidAddress", []
        {
            emitInstruction16("0101110mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setRegisterValue(R7, 0);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

        /* LDRH - Immediate
   Encoding: 1000 1 Imm:5 Rn:3 Rt:3 */
        cpu_test("ldrhImmediate : UseAMixOfRegistersWordAligned", []
        {
            emitInstruction16("10001iiiiinnnttt", 0, R7, R0);
            setRegisterValue(R7, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrhImmediate : UseAnotherMixOfRegistersNotWordAligned", []
        {
            emitInstruction16("10001iiiiinnnttt", 1, R0, R7);
            setRegisterValue(R0, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xBAAD);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrhImmediate : LargestOffset", []
        {
            emitInstruction16("10001iiiiinnnttt", 31, R1, R6);
            setRegisterValue(R1, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 60, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R6, 0xBAAD);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT
        cpu_test("ldrhImmediate : AttemptLoadFromInvalidAddress", []
        {
            emitInstruction16("10001iiiiinnnttt", 0, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif


/* LDR - Immediate Encoding T1
   Encoding: 011 0 1 Imm:5 Rn:3 Rt:3 */
        cpu_test("ldrImmediate : T1UseAMixOfRegistersWithSmallestOffset", []
        {
            emitInstruction16("01101iiiiinnnttt", 0, R7, R0);
            setRegisterValue(R7, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrImmediate : T1UseAnotherMixOfRegistersWithLargestOffset", []
        {
            emitInstruction16("01101iiiiinnnttt", 31, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 31 * 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT
        cpu_test("ldrImmediate : T1AttemptUnalignedLoad", []
        {
            emitInstruction16("01101iiiiinnnttt", 0, R3, R2);
            setRegisterValue(R3, INITIAL_PC + 2);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrImmediate : T1AttemptLoadFromInvalidAddress", []
        {
            emitInstruction16("01101iiiiinnnttt", 16, R3, R2);
            setRegisterValue(R3, 0xFFFFFFFC - 16 * 4);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* LDR - Immediate Encoding T2 (SP is base register)
   Encoding: 1001 1 Rt:3 Imm:8 */
        cpu_test("ldrImmediate : T2UseHighestRegisterWithSmallestOffset", []
        {
            emitInstruction16("10011tttiiiiiiii", R7, 0);
            setRegisterValue(SP, INITIAL_PC + 1024);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 1024, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrImmediate : T2UseLowestRegisterWithLargestOffset", []
        {
            emitInstruction16("10011tttiiiiiiii", R0, 255);
            setRegisterValue(SP, INITIAL_PC + 1024);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 1024 + 255 * 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT
        TEST_SIM_ONLY(ldrImmediate, T2AttemptUnalignedLoad)
        {
            emitInstruction16("10011tttiiiiiiii", R2, 0);
            setRegisterValue(SP, INITIAL_PC + 1026);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 1024, 0xBAADFEED, READ_ONLY);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(ldrImmediate, T2AttemptLoadFromInvalidAddress)
        {
            emitInstruction16("10011tttiiiiiiii", R2, 0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* LDR - Literal
   Encoding: 01001 Rt:3 Imm:8 */
        cpu_test("ldrLiteral : LoadOffset0IntoHighestRegister", []
        {
            emitInstruction16("01001tttiiiiiiii", R7, 0);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrLiteral : LoadOffset0IntoHighestRegisterNot4ByteAligned", []
        {
            // Emit UNDEFINED 16-bit instruction.
            emitInstruction16("1101111000000000");
            // Emit actual test instruction at a 2-byte aligned address which isn't 4-byte aligned.
            emitInstruction16("01001tttiiiiiiii", R7, 0);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            // Move PC to point to second instruction.
            setRegisterValue(PC, cpu.regs[PC] + 2);
            setExpectedRegisterValue(R7, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrLiteral : LoadMaximumOffsetIntoLowestRegister", []
        {
            emitInstruction16("01001tttiiiiiiii", R0, 255);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4 + 255*4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT
        cpu_test("ldrLiteral : AttemptToLoadFromInvalidAddress", []
        {
            m_emitAddress = INITIAL_SP - 128;
            setRegisterValue(PC, INITIAL_SP - 128);
            setExpectedRegisterValue(PC, INITIAL_SP - 128);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 128, 0, READ_WRITE);
            emitInstruction16("01001tttiiiiiiii", R0, 255);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            pinkySimStep(&m_context);
        }),
#endif
        /* LDR - Register
   Encoding: 0101 100 Rm:3 Rn:3 Rt:3 */
        cpu_test("ldrRegister : UseAMixOfRegisters", []
        {
            emitInstruction16("0101100mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrRegister : UseAnotherMixOfRegisters", []
        {
            emitInstruction16("0101100mmmnnnttt", R1, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            setRegisterValue(R1, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrRegister : YetAnotherMixOfRegisters", []
        {
            emitInstruction16("0101100mmmnnnttt", R0, R7, R4);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R4, 0xBAADFEED);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT

        cpu_test("ldrRegister : AttemptUnalignedLoad", []
        {
            emitInstruction16("0101100mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 2);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrRegister : AttemptLoadFromInvalidAddress", []
        {
            emitInstruction16("0101100mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setRegisterValue(R7, 0);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

        /* LDRSB - Register
   Encoding: 0101 011 Rm:3 Rn:3 Rt:3 */
        cpu_test("ldrsbRegister : UseAMixOfRegistersWordAligned_NegativeValue", []
        {
            emitInstruction16("0101011mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xFFFFFFED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrsbRegister : UseAnotherMixOfRegistersSecondByteInWord_NegativeValue", []
        {
            emitInstruction16("0101011mmmnnnttt", R1, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            setRegisterValue(R1, 5);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xFFFFFFFE);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrsbRegister : YetAnotherMixOfRegistersThirdByteInWord_NegativeValue", []
        {
            emitInstruction16("0101011mmmnnnttt", R0, R7, R4);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 6);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R4, 0xFFFFFFAD);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrsbRegister : YetAnotherMixOfRegistersFourthByteInWord_NegativeValue", []
        {
            emitInstruction16("0101011mmmnnnttt", R0, R7, R5);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 7);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R5, 0xFFFFFFBA);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrsbRegister : LoadAPositiveValue", []
        {
            emitInstruction16("0101011mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xFFFFFF7F, READ_ONLY);
            setExpectedRegisterValue(R0, 0x7F);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT

        cpu_test("ldrsbRegister : AttemptLoadInvalidAddress", []
        {
            emitInstruction16("0101011mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setRegisterValue(R7, 0);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

        /* LDRSH - Register
   Encoding: 0101 111 Rm:3 Rn:3 Rt:3 */
        cpu_test("ldrshRegister : UseAMixOfRegistersWordAligned_NegativeValue", []
        {
            emitInstruction16("0101111mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R0, 0xFFFFFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrshRegister : UseAnotherMixOfRegistersWordAligned_NegativeValue", []
        {
            emitInstruction16("0101111mmmnnnttt", R1, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            setRegisterValue(R1, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R7, 0xFFFFFEED);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrshRegister : YetAnotherMixOfRegistersNotWordAligned_NegativeValue", []
        {
            emitInstruction16("0101111mmmnnnttt", R0, R7, R4);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 6);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_ONLY);
            setExpectedRegisterValue(R4, 0xFFFFBAAD);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrshRegister : LoadPositiveHalfWord", []
        {
            emitInstruction16("0101111mmmnnnttt", R0, R7, R4);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xFFFF7FFF, READ_ONLY);
            setExpectedRegisterValue(R4, 0x7FFF);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT

        cpu_test("ldrshRegister : AttemptUnalignedLoad", []
        {
            emitInstruction16("0101111mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 1);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        cpu_test("ldrshRegister : AttemptLoadFromInvalidAddress", []
        {
            emitInstruction16("0101111mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setRegisterValue(R7, 0);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

        /* LSL - Immediate (Logical Shift Left)
   Encoding: 000 00 imm:5 Rm:3 Rd:3 */
        cpu_test("lslImmediate : MovR7toR0_CarryUnmodified", []
        {
            emitInstruction16("00000iiiiimmmddd", IMM_0, R7, R0);
            setExpectedXPSRflags("nzc");
            clearCarry();
            setExpectedRegisterValue(R0, 0x77777777U);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslImmediate : MovR0toR7_ZeroResultAndCarryUnmodified", []
        {
            emitInstruction16("00000iiiiimmmddd", IMM_0, R0, R7);
            setExpectedXPSRflags("nZC");
            setCarry();
            setExpectedRegisterValue(R7, 0x0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslImmediate : ShiftR1by3_ResultInNegativeValue", []
        {
            emitInstruction16("00000iiiiimmmddd", IMM_3, R1, R0);
            setExpectedXPSRflags("Nzc");
            setExpectedRegisterValue(R0, 0x11111111U << 3);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslImmediate : ShiftR1by4_HasCarryOut", []
        {
            emitInstruction16("00000iiiiimmmddd", IMM_4, R1, R0);
            setExpectedXPSRflags("nzC");
            setExpectedRegisterValue(R0, 0x11111111U << 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslImmediate : ShiftR0by31_PushesLowestbitIntoSignBit", []
        {
            emitInstruction16("00000iiiiimmmddd", IMM_31, R0, R0);
            setExpectedXPSRflags("Nzc");
            setRegisterValue(R0, 1U);
            setExpectedRegisterValue(R0, 1U << 31);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslImmediate : CarryOutFromHighestBit", []
        {
            emitInstruction16("00000iiiiimmmddd", IMM_1, R0, R0);
            setExpectedXPSRflags("nzC");
            setRegisterValue(R0, 0xA0000000U);
            setExpectedRegisterValue(R0, 0xA0000000U << 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslImmediate : CarryOutFromLowestBit", []
        {
            emitInstruction16("00000iiiiimmmddd", IMM_31, R0, R0);
            setExpectedXPSRflags("nZC");
            setRegisterValue(R0, 0x2U);
            setExpectedRegisterValue(R0, 0x2U << 31);
            pinkySimStep(&m_context);
        }),

        /* LSL - Register (Logical Shift Left)
   Encoding: 010000 0010 Rm:3 Rdn:3 */
        cpu_test("lslRegister : ShiftR7by0_MinimumShift_CarryShouldBeUnmodified", []
        {
            emitInstruction16("0100000010mmmddd", R0, R7);
            setExpectedXPSRflags("nzC");
            setCarry();
            setExpectedRegisterValue(R7, 0x77777777U);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslRegister : ShiftValue1by31_NegativeResult", []
        {
            emitInstruction16("0100000010mmmddd", R4, R3);
            setExpectedXPSRflags("Nzc");
            setRegisterValue(R3, 1);
            setRegisterValue(R4, 31);
            setExpectedRegisterValue(R3, 1 << 31);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslRegister : ShiftValue1by32_CarryOutFromLowestBit", []
        {
            emitInstruction16("0100000010mmmddd", R7, R0);
            setExpectedXPSRflags("nZC");
            setRegisterValue(R0, 1);
            setRegisterValue(R7, 32);
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslRegister : ShiftNegativeValueBy1_CarryOutFromHighestBit", []
        {
            emitInstruction16("0100000010mmmddd", R3, R4);
            setExpectedXPSRflags("NzC");
            setRegisterValue(R4, -1);
            setRegisterValue(R3, 1);
            setExpectedRegisterValue(R4, -1 << 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslRegister : ShiftValue1by33_NoCarry", []
        {
            emitInstruction16("0100000010mmmddd", R7, R0);
            setExpectedXPSRflags("nZc");
            setRegisterValue(R0, 1);
            setRegisterValue(R7, 33);
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslRegister : ShiftValuee1by255_MaximumShift", []
        {
            emitInstruction16("0100000010mmmddd", R7, R0);
            setExpectedXPSRflags("nZc");
            setRegisterValue(R0, 1);
            setRegisterValue(R7, 255);
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lslRegister : ShiftValue1by256_ShouldBeTreatedAs0Shift_CarryUnmodified", []
        {
            emitInstruction16("0100000010mmmddd", R7, R0);
            setExpectedXPSRflags("nzc");
            clearCarry();
            setRegisterValue(R0, 1);
            setRegisterValue(R7, 256);
            setExpectedRegisterValue(R0, 1);
            pinkySimStep(&m_context);
        }),

        /* LSR - Immediate (Logical Shift Right)
   Encoding: 000 01 imm:5 Rm:3 Rd:3 */
        cpu_test("lsrImmediate : R2by1toR0", []
        {
            emitInstruction16("00001iiiiimmmddd", IMM_1, R2, R0);
            setExpectedXPSRflags("nzc");
            setExpectedRegisterValue(R0, 0x22222222U >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrImmediate : R7by32toR0_ZeroResult", []
        {
            emitInstruction16("00001iiiiimmmddd", IMM_32, R7, R0);
            setExpectedXPSRflags("nZc");
            setExpectedRegisterValue(R0, 0x0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrImmediate : R1by1toR7_CarryOut", []
        {
            emitInstruction16("00001iiiiimmmddd", IMM_1, R1, R7);
            setExpectedXPSRflags("nzC");
            setExpectedRegisterValue(R7, 0x11111111U >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrImmediate : R0by32_CarryOutAndIsZero", []
        {
            emitInstruction16("00001iiiiimmmddd", IMM_32, R0, R0);
            setExpectedXPSRflags("nZC");
            setRegisterValue(R0, 0x80000000U);
            setExpectedRegisterValue(R0, 0U);
            pinkySimStep(&m_context);
        }),

/* LSR - Register (Logical Shift Right)
   Encoding: 010000 0011 Rm:3 Rdn:3 */
        cpu_test("lsrRegister : ShiftValue1by1_CarryOutFromLowestBit", []
        {
            emitInstruction16("0100000011mmmddd", R0, R7);
            setExpectedXPSRflags("nZC");
            setRegisterValue(R0, 1);
            setRegisterValue(R7, 1);
            setExpectedRegisterValue(R7, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrRegister : ShiftValue1by0_MinimumShift_CarryUnmodified", []
        {
            emitInstruction16("0100000011mmmddd", R0, R7);
            setExpectedXPSRflags("nzc");
            clearCarry();
            setRegisterValue(R7, 1);
            setRegisterValue(R0, 0);
            setExpectedRegisterValue(R7, 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrRegister : ShiftValue2by1_NoCarry", []
        {
            emitInstruction16("0100000011mmmddd", R3, R2);
            setExpectedXPSRflags("nzc");
            setRegisterValue(R2, 2);
            setRegisterValue(R3, 1);
            setExpectedRegisterValue(R2, 2 >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrRegister : ShiftNegativeValueBy31", []
        {
            emitInstruction16("0100000011mmmddd", R3, R2);
            setExpectedXPSRflags("nzC");
            setRegisterValue(R2, -1);
            setRegisterValue(R3, 31);
            setExpectedRegisterValue(R2, -1U >> 31);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrRegister : ShiftNegativeValueBy32_CarryOutFromHighestBit", []
        {
            emitInstruction16("0100000011mmmddd", R7, R0);
            setExpectedXPSRflags("nZC");
            setRegisterValue(R0, 0x80000000);
            setRegisterValue(R7, 32);
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrRegister : ShiftNegativeValueBy33_ResultIsZero_CarryClear", []
        {
            emitInstruction16("0100000011mmmddd", R3, R2);
            setExpectedXPSRflags("nZc");
            setRegisterValue(R2, -1);
            setRegisterValue(R3, 33);
            setExpectedRegisterValue(R2, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrRegister : MaximumShiftOf255_ResultIsZero_CarryClear", []
        {
            emitInstruction16("0100000011mmmddd", R3, R2);
            setExpectedXPSRflags("nZc");
            setRegisterValue(R2, -1);
            setRegisterValue(R3, 255);
            setExpectedRegisterValue(R2, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("lsrRegister : ShiftOf256_ShouldBeTreatedAs0Shift_CarryUnmodified", []
        {
            emitInstruction16("0100000011mmmddd", R7, R0);
            setExpectedXPSRflags("NzC");
            setCarry();
            setRegisterValue(R0, -1);
            setRegisterValue(R7, 256);
            setExpectedRegisterValue(R0, -1);
            pinkySimStep(&m_context);
        }),

        /* MOV - Immediate
   Encoding: 001 00 Rd:3 Imm:8 */
/* NOTE: APSR_C state is maintained by this instruction. */
        cpu_test("movImmediate : MovToR0", []
        {
            emitInstruction16("00100dddiiiiiiii", R0, 127);
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            setExpectedXPSRflags("nzc");
            clearCarry();
            setExpectedRegisterValue(R0, 127);
            pinkySimStep(&m_context);
        }),

        cpu_test("movImmediate : MovToR7", []
        {
            emitInstruction16("00100dddiiiiiiii", R7, 127);
            setExpectedXPSRflags("nzC");
            setCarry();
            setExpectedRegisterValue(R7, 127);
            pinkySimStep(&m_context);
        }),

        cpu_test("movImmediate : MovSmallestImmediateValueToR3", []
        {
            emitInstruction16("00100dddiiiiiiii", R3, 0);
            setExpectedXPSRflags("nZ");
            setExpectedRegisterValue(R3, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("movImmediate : MovLargestImmediateValueToR3", []
        {
            emitInstruction16("00100dddiiiiiiii", R3, 255);
            setExpectedXPSRflags("nz");
            setExpectedRegisterValue(R3, 255);
            pinkySimStep(&m_context);
        }),

#if ARMULET_FEATURE_ARMV8M_BASELINE_MOWV_MOVT
        cpu_test("movw : MovToR0", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 1, 5, 2, R0, 0x34);
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            setExpectedXPSRflags("nzc");
            clearCarry();
            clearNegative();
            clearZero();
            setExpectedRegisterValue(R0, 0x5a34);
            pinkySimStep(&m_context);
        }),

        cpu_test("movw : MovToR7", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 1, 6, 2, R7, 0x34);

            setExpectedXPSRflags("nzC");
            clearZero();
            clearNegative();
            setCarry();
            setExpectedRegisterValue(R7, 0x6a34);
            pinkySimStep(&m_context);
        }),

        cpu_test("movw : MovToR10", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 0, 5, 2, R10, 0x34);
            setExpectedXPSRflags("NzC");
            clearZero();
            setNegative();
            setCarry();
            setExpectedRegisterValue(R10, 0x5234);
            pinkySimStep(&m_context);
        }),

        cpu_test("movw : MovSmallestImmediateValueToR3", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 0, 0, 0, R3, 0);
            setExpectedXPSRflags("nz");
            clearZero();
            clearNegative();
            setExpectedRegisterValue(R3, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("movw : MovLargestImmediateValueToR3", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 1, 0xf, 0x7, R3, 0xff);
            // should not set flags
            setExpectedXPSRflags("nZ");
            clearNegative();
            setZero();
            setExpectedRegisterValue(R3, 65535);
            pinkySimStep(&m_context);
        }),

        // Test each immediate sub-field
        cpu_test("movw : MovImm8ToR4", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 0, 0x0, 0x0, R4, 0xff);
            // should not set flags
            setExpectedXPSRflags("nZ");
            clearNegative();
            setZero();
            setExpectedRegisterValue(R4, 0x00ff);
            pinkySimStep(&m_context);
        }),

        cpu_test("movw : MovImm3ToR4", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 0, 0x0, 0x7, R4, 0x0);
            // should not set flags
            setExpectedXPSRflags("nZ");
            clearNegative();
            setZero();
            setExpectedRegisterValue(R4, 0x0700);
            pinkySimStep(&m_context);
        }),

        cpu_test("movw : MovImm1ToR4", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 1, 0x0, 0x0, R4, 0x0);
            // should not set flags
            setExpectedXPSRflags("nZ");
            clearNegative();
            setZero();
            setExpectedRegisterValue(R4, 0x0800);
            pinkySimStep(&m_context);
        }),

        cpu_test("movw : MovImm4ToR4", []
        {
            emitInstruction32("11110i100100jjjj", "0kkkddddllllllll", 0, 0xf, 0x0, R4, 0x0);
            // should not set flags
            setExpectedXPSRflags("nZ");
            clearNegative();
            setZero();
            setExpectedRegisterValue(R4, 0xf000);
            pinkySimStep(&m_context);
        }),

        cpu_test("movt : MovToR0", []
        {
            emitInstruction32("11110i101100jjjj", "0kkkddddllllllll", 1, 5, 2, R0, 0x34);
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            setExpectedXPSRflags("nzc");
            clearCarry();
            clearNegative();
            clearZero();
            setExpectedRegisterValue(R0, 0x5a340000);
            pinkySimStep(&m_context);
        }),

        cpu_test("movt : MovToR7", []
        {
            emitInstruction32("11110i101100jjjj", "0kkkddddllllllll", 1, 6, 2, R7, 0x34);

            setExpectedXPSRflags("nzC");
            clearZero();
            clearNegative();
            setCarry();
            setExpectedRegisterValue(R7, 0x6a347777);
            pinkySimStep(&m_context);
        }),

        cpu_test("movt : MovToR10", []
        {
            emitInstruction32("11110i101100jjjj", "0kkkddddllllllll", 0, 5, 2, R10, 0x34);
            setExpectedXPSRflags("NzC");
            clearZero();
            setNegative();
            setCarry();
            setExpectedRegisterValue(R10, 0x5234aaaa);
            pinkySimStep(&m_context);
        }),

        cpu_test("movt : MovSmallestImmediateValueToR3", []
        {
            emitInstruction32("11110i101100jjjj", "0kkkddddllllllll", 0, 0, 0, R3, 0);
            setExpectedXPSRflags("nz");
            clearZero();
            clearNegative();
            setExpectedRegisterValue(R3, 0x3333);
            pinkySimStep(&m_context);
        }),

        cpu_test("movt : MovLargestImmediateValueToR3", []
        {
            emitInstruction32("11110i101100jjjj", "0kkkddddllllllll", 1, 0xf, 0x7, R3, 0xff);
            // should not set flags
            setExpectedXPSRflags("nZ");
            clearNegative();
            setZero();
            setExpectedRegisterValue(R3, 0xffff3333);
            pinkySimStep(&m_context);
        }),
#endif
        /* MOV - Register Encoding 1
   Encoding: 010001 10 D:1 Rm:4 Rd:3
   NOTE: This encoding doesn't update the APSR flags. */
        cpu_test("movRegister : UseLowestRegisterForAllArgs", []
        {
            emitInstruction16("01000110dmmmmddd", R0, R0);
            pinkySimStep(&m_context);
        }),

        cpu_test("movRegister : UseHighRegisterForAllArgs", []
        {
            emitInstruction16("01000110dmmmmddd", LR, LR);
            pinkySimStep(&m_context);
        }),

        cpu_test("movRegister : MoveHighRegisterToLowRegister", []
        {
            emitInstruction16("01000110dmmmmddd", R7, R12);
            setExpectedRegisterValue(R7, 0xCCCCCCCC);
            pinkySimStep(&m_context);
        }),

        cpu_test("movRegister : MoveLowRegisterToLHighRegister", []
        {
            emitInstruction16("01000110dmmmmddd", R12, R7);
            setExpectedRegisterValue(R12, 0x77777777);
            pinkySimStep(&m_context);
        }),

        cpu_test("movRegister : MoveOddAddressIntoPCAndMakeSureLSbitIsCleared", []
        {
            emitInstruction16("01000110dmmmmddd", PC, R1);
            setRegisterValue(R1, INITIAL_PC + 1025);
            setExpectedRegisterValue(PC, INITIAL_PC + 1024);
            pinkySimStep(&m_context);
        }),

        cpu_test("movRegister : MoveEvenAddressIntoPC", []
        {
            emitInstruction16("01000110dmmmmddd", PC, R2);
            setRegisterValue(R2, INITIAL_PC + 1024);
            setExpectedRegisterValue(PC, INITIAL_PC + 1024);
            pinkySimStep(&m_context);
        }),

        cpu_test("movRegister : MovePCintoOtherRegister", []
        {
            emitInstruction16("01000110dmmmmddd", R3, PC);
            setExpectedRegisterValue(R3, INITIAL_PC + 4);
            pinkySimStep(&m_context);
        }),

#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
        cpu_test("movRegister_MSPLIM_nooverflow", []
        {
            emitInstruction16("01000110dmmmmddd", SP, R3);
            setRegisterValue(R3, 0x10000004);
            setMSPLIM(0x10000004);
            setExpectedRegisterValue(SP, 0x10000004);
            pinkySimStep(&m_context);
        }),

        cpu_test("movRegister_MSPLIM_overflow", []
        {
            emitInstruction16("01000110dmmmmddd", SP, R3);
            setRegisterValue(R3, 0x10000000);
            setMSPLIM(0x10000004);
            setExpectedRegisterValue(SP, 0x10000000);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT); // we do breakpoint on stack overflow
            pinkySimStep(&m_context);
        }),
#endif
        /* MRS
   Encoding: 11110 0 1111 1 (0) (1)(1)(1)(1)
             10 (0) 0 Rd:4 SYSm:8 */
        cpu_test("mrs : FromAPSR", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_APSR);
            setExpectedXPSRflags("NzCv");
            setNegative(); clearZero(); setCarry(); clearOverflow();
            setIPSR(IPSR_VAL);
            setExpectedIPSR(IPSR_VAL);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, APSR_N | APSR_C);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : FromIAPSR", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R0, SYS_IAPSR);
            setExpectedXPSRflags("NzCv");
            setNegative(); clearZero(); setCarry(); clearOverflow();
            setIPSR(IPSR_VAL);
            setExpectedIPSR(IPSR_VAL);
            setRegisterValue(R0, 0xFFFFFFFF);
            setExpectedRegisterValue(R0, APSR_N | APSR_C | IPSR_VAL);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : FromEAPSR", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_EAPSR);
            setExpectedXPSRflags("NzCv");
            setNegative(); clearZero(); setCarry(); clearOverflow();
            setIPSR(IPSR_VAL);
            setExpectedIPSR(IPSR_VAL);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, APSR_N | APSR_C);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : FromXPSR", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_XPSR);
            setExpectedXPSRflags("NzCv");
            setNegative(); clearZero(); setCarry(); clearOverflow();
            setIPSR(IPSR_VAL);
            setExpectedIPSR(IPSR_VAL);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, APSR_N | APSR_C | IPSR_VAL);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : FromIPSR", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_IPSR);
            setExpectedXPSRflags("NzCv");
            setNegative(); clearZero(); setCarry(); clearOverflow();
            setIPSR(IPSR_VAL);
            setExpectedIPSR(IPSR_VAL);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, IPSR_VAL);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : FromEPSR", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_EPSR);
            setExpectedXPSRflags("NzCv");
            setNegative(); clearZero(); setCarry(); clearOverflow();
            setIPSR(IPSR_VAL);
            setExpectedIPSR(IPSR_VAL);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : FromIEPSR", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_IEPSR);
            setExpectedXPSRflags("NzCv");
            setNegative(); clearZero(); setCarry(); clearOverflow();
            setIPSR(IPSR_VAL);
            setExpectedIPSR(IPSR_VAL);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, IPSR_VAL);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : FromMSP", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_MSP);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, INITIAL_SP);
            pinkySimStep(&m_context);
        }),

#if PRIVILEGED
        TEST_SIM_ONLY(mrs, FromPSP)
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_PSP);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, 0x0);
            pinkySimStep(&m_context);
        }),
#endif

        cpu_test("mrs : FromPRIMASKsetTo1", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_PRIMASK);
            setRegisterValue(R12, 0xFFFFFFFF);
            m_context.PRIMASK = 1;
            setExpectedRegisterValue(R12, 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : FromPRIMASKsetTo0", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_PRIMASK);
            setRegisterValue(R12, 0xFFFFFFFF);
            m_context.PRIMASK = 0;
            setExpectedRegisterValue(R12, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("mrs : CONTROLIgnored", []
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", R12, SYS_CONTROL);
            setRegisterValue(R12, 0xFFFFFFFF);
            setExpectedRegisterValue(R12, 0);
            pinkySimStep(&m_context);
        }),

#if UNPREDICTABLE
        TEST_SIM_ONLY(mrs, R13IsUnpredictable)
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", SP, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(mrs, R15IsUnpredictable)
        {
            emitInstruction32("1111001111101111", "1000ddddssssssss", PC, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(mrs, UnpredictableSYSm)
        {
            uint32_t predictables[] = {0, 1, 2, 3, 5, 6, 7, 8, 9, 16, 20}),;
            size_t   nextSkip = 0;

            for (uint32_t i = 0 ; i < 256 ; i++)
            {
                if (nextSkip < sizeof(predictables)/sizeof(predictables[0]) && i == predictables[nextSkip])
                {
                    nextSkip++;
                }),
                else
                {
                    initContext();
                    emitInstruction32("1111001111101111", "1000ddddssssssss", R0, i);
                    setRegisterValue(R0, 0xFFFFFFFF);
                    setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
                    setExpectedRegisterValue(PC, INITIAL_PC);
                    pinkySimStep(&m_context);
                }),
            }),
        }),

        TEST_SIM_ONLY(mrs, UnpredictableBecauseOfBit2_13)
        {
            emitInstruction32("1111001111101111", "1010ddddssssssss", R12, SYS_XPSR);
            setRegisterValue(R0, 0xFFFFFFFF);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(mrs, UnpredictableBecauseOfBit1_0)
        {
            emitInstruction32("1111001111101110", "1000ddddssssssss", R12, SYS_XPSR);
            setRegisterValue(R0, 0xFFFFFFFF);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(mrs, UnpredictableBecauseOfBit1_1)
        {
            emitInstruction32("1111001111101101", "1000ddddssssssss", R12, SYS_XPSR);
            setRegisterValue(R0, 0xFFFFFFFF);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(mrs, UnpredictableBecauseOfBit1_2)
        {
            emitInstruction32("1111001111101011", "1000ddddssssssss", R12, SYS_XPSR);
            setRegisterValue(R0, 0xFFFFFFFF);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(mrs, UnpredictableBecauseOfBit1_3)
        {
            emitInstruction32("1111001111100111", "1000ddddssssssss", R12, SYS_XPSR);
            setRegisterValue(R0, 0xFFFFFFFF);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(mrs, UnpredictableBecauseOfBit1_4)
        {
            emitInstruction32("1111001111111111", "1000ddddssssssss", R12, SYS_XPSR);
            setRegisterValue(R0, 0xFFFFFFFF);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* MSR
   Encoding: 11110 0 1110 0 (0) Rn:4
             10 (0) 0 (1) (0) (0) (0) SYSm:8 */
#if NEEDS_NZ
        cpu_test("msr : ToAPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_APSR);
            setExpectedXPSRflags("NZCV");
            clearNegative(); clearZero(); clearCarry(); clearOverflow();
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToIAPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_IAPSR);
            setExpectedXPSRflags("NZCV");
            clearNegative(); clearZero(); clearCarry(); clearOverflow();
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToEAPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_EAPSR);
            setExpectedXPSRflags("NZCV");
            clearNegative(); clearZero(); clearCarry(); clearOverflow();
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToXPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R0, SYS_XPSR);
            setExpectedXPSRflags("NZCV");
            clearNegative(); clearZero(); clearCarry(); clearOverflow();
            setRegisterValue(R0, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToIPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_IPSR);
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToEPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_EPSR);
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToIEPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_IEPSR);
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),
#endif
#if 0
        cpu_test("msr : ToAPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_APSR);
            setExpectedXPSRflags("nZCV");
            setNegative(); clearZero(); clearCarry(); clearOverflow();
            setRegisterValue(R12, 0x7FFFFFFF);
            pinkySimStep(&m_context);
        }),
#endif
        cpu_test("msr : ToIAPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_IAPSR);
            setExpectedXPSRflags("NzcV");
            clearNegative(); setZero(); setCarry(); clearOverflow();
            setRegisterValue(R12, 0x9FFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToEAPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_EAPSR);
            setExpectedXPSRflags("nzCV");
            setNegative(); setZero(); clearCarry(); clearOverflow();
            setRegisterValue(R12, 0x3FFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToXPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R0, SYS_XPSR);
            setExpectedXPSRflags("nZCV");
            setNegative(); clearZero(); clearCarry(); clearOverflow();
            setRegisterValue(R0, 0x7FFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToIPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_IPSR);
            setExpectedXPSRflags("Nzcv");
            setNegative(); clearZero(); clearCarry(); clearOverflow();
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToEPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_EPSR);
            setExpectedXPSRflags("Nzcv");
            setNegative(); clearZero(); clearCarry(); clearOverflow();
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToIEPSR", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_IEPSR);
            setExpectedXPSRflags("nZCV");
            clearNegative(); setZero(); setCarry(); setOverflow();
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("msr : ToMSP", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_MSP);
            setRegisterValue(R12, INITIAL_PC + 1024 + 2);
            setExpectedRegisterValue(SP, INITIAL_PC + 1024);
            pinkySimStep(&m_context);
        }),

#if PRIVILEGED
        cpu_test("msr : ToPSP", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_PSP);
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
        }),
#endif

        cpu_test("msr : PRIMASKto1", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_PRIMASK);
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
            CHECK_EQUAL(1, m_context.PRIMASK);
        }),

        cpu_test("msr : PRIMASKto0", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_PRIMASK);
            setRegisterValue(R12, 0xFFFFFFFE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0, m_context.PRIMASK);
        }),

        cpu_test("msr : CONTROLignored", []
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", R12, SYS_CONTROL);
            setRegisterValue(R12, 0xFFFFFFFF);
            pinkySimStep(&m_context);
//            CHECK_EQUAL(0, m_context.CONTROL);
        }),

#if UNPREDICTABLE        
        TEST_SIM_ONLY(msr, R13IsUnpredictable)
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", SP, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(msr, R15IsUnpredictable)
        {
            emitInstruction32("111100111000nnnn", "10001000ssssssss", PC, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(msr, UnpredictableSYSm)
        {
            uint32_t predictables[] = {0, 1, 2, 3, 5, 6, 7, 8, 9, 16, 20}),;
            size_t   nextSkip = 0;

            for (uint32_t i = 0 ; i < 256 ; i++)
            {
                if (nextSkip < sizeof(predictables)/sizeof(predictables[0]) && i == predictables[nextSkip])
                {
                    nextSkip++;
                }),
                else
                {
                    initContext();
                    emitInstruction32("111100111000nnnn", "10001000ssssssss", R0, i);
                    setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
                    setExpectedRegisterValue(PC, INITIAL_PC);
                    pinkySimStep(&m_context);
                }),
            }),
        }),

        TEST_SIM_ONLY(msr, UnpredictableBecauseOfBit2_8)
        {
            emitInstruction32("111100111000nnnn", "10001001ssssssss", R0, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(msr, UnpredictableBecauseOfBit2_9)
        {
            emitInstruction32("111100111000nnnn", "10001010ssssssss", R0, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(msr, UnpredictableBecauseOfBit2_10)
        {
            emitInstruction32("111100111000nnnn", "10001100ssssssss", R0, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(msr, UnpredictableBecauseOfBit2_11)
        {
            emitInstruction32("111100111000nnnn", "10000000ssssssss", R0, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(msr, UnpredictableBecauseOfBit2_13)
        {
            emitInstruction32("111100111000nnnn", "10101000ssssssss", R0, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(msr, UnpredictableBecauseOfBit1_4)
        {
            emitInstruction32("111100111001nnnn", "10001000ssssssss", R0, SYS_XPSR);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* MUL
   Encoding: 010000 1101 Rn:3 Rdm:3 */
        cpu_test("mul : UseLowestRegisterForAllArgs", []
        {
            emitInstruction16("0100001101nnnddd", R0, R0);
            setExpectedXPSRflags("nZ");
            setExpectedRegisterValue(R0, 0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("mul : UseHigestRegisterForAllArgs_OnlyGetLower32bitsOfResult", []
        {
            emitInstruction16("0100001101nnnddd", R7, R7);
            setExpectedXPSRflags("Nz");
            setExpectedRegisterValue(R7, 0x77777777U * 0x77777777U);
            pinkySimStep(&m_context);
        }),

        cpu_test("mul : UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("0100001101nnnddd", R1, R2);
            setRegisterValue(R1, 0xA5A5);
            setRegisterValue(R2, 2);
            setExpectedXPSRflags("nz");
            setExpectedRegisterValue(R2, 0xA5A5U << 1U);
            pinkySimStep(&m_context);
        }),

        cpu_test("mul : MultiplyBy16BitMaximumValues", []
        {
            emitInstruction16("0100001101nnnddd", R1, R2);
            setRegisterValue(R1, 0xFFFF);
            setRegisterValue(R2, 0xFFFF);
            setExpectedXPSRflags("Nz");
            setExpectedRegisterValue(R2, 0xFFFFu * 0xFFFFu);
            pinkySimStep(&m_context);
        }),

        /* MVN - Register (MOve Negative)
   Encoding: 010000 1111 Rm:3 Rd:3 */
/* NOTE: APSR_C state is maintained by this instruction. */
        cpu_test("mvnRegister : UseLowestRegisterForAllArgs", []
        {
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            emitInstruction16("0100001111mmmddd", R0, R0);
            setExpectedXPSRflags("NzC");
            setCarry();
            setExpectedRegisterValue(R0, ~0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("mvnRegister : UseHigestRegisterForAllArgs", []
        {
            emitInstruction16("0100001111mmmddd", R7, R7);
            setExpectedXPSRflags("Nzc");
            clearCarry();
            setExpectedRegisterValue(R7, ~0x77777777U);
            pinkySimStep(&m_context);
        }),

        cpu_test("mvnRegister : UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("0100001111mmmddd", R2, R1);
            setExpectedXPSRflags("Nz");
            setExpectedRegisterValue(R1, ~0x22222222U);
            pinkySimStep(&m_context);
        }),

        cpu_test("mvnRegister : MoveANegationOfNegativeOne_ClearsNegativeFlagAndSetsZeroFlag", []
        {
            emitInstruction16("0100001111mmmddd", R2, R1);
            setRegisterValue(R2, -1);
            setExpectedXPSRflags("nZ");
            setExpectedRegisterValue(R1, 0U);
            pinkySimStep(&m_context);
        }),

        /* NOP
   Encoding: 1011 1111 0000 0000 */
        cpu_test("nop : BasicTest", []
        {
            emitInstruction16("1011111100000000");
            pinkySimStep(&m_context);
        }),


/* ORR - Register
   Encoding: 010000 1100 Rm:3 Rdn:3 */
/* NOTE: APSR_C state is maintained by this instruction. */
        cpu_test("orrRegister : UseLowestRegisterForBothArgs", []
        {
            emitInstruction16("0100001100mmmddd", R0, R0);
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            setExpectedXPSRflags("nZc");
            clearCarry();
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("orrRegister : UseHighestRegisterForBothArgs", []
        {
            emitInstruction16("0100001100mmmddd", R7, R7);
            setExpectedXPSRflags("nzC");
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("orrRegister : OrR3andR7", []
        {
            emitInstruction16("0100001100mmmddd", R3, R7);
            setExpectedXPSRflags("nz");
            setExpectedRegisterValue(R7, 0x33333333 | 0x77777777);
            pinkySimStep(&m_context);
        }),

        cpu_test("orrRegister : UseOrToTurnOnNegativeSignBit", []
        {
            emitInstruction16("0100001100mmmddd", R7, R0);
            setRegisterValue(R0, 0x7FFFFFFF);
            setRegisterValue(R7, 0x80000000);
            setExpectedXPSRflags("Nz");
            setExpectedRegisterValue(R0, 0x7FFFFFFF | 0x80000000);
            pinkySimStep(&m_context);
        }),

        cpu_test("orrRegister : HaveAndResultNotBeSameAsEitherSource", []
        {
            emitInstruction16("0100001100mmmddd", R0, R7);
            setRegisterValue(R0, 0x12345678);
            setRegisterValue(R7, 0xF0F0F0F0);
            setExpectedXPSRflags("Nz");
            setExpectedRegisterValue(R7, 0xF2F4F6F8);
            pinkySimStep(&m_context);
        }),

/* POP
   Encoding: 1011 1 10 P:1 RegisterList:8 */
        cpu_test("pop : JustPopPC", []
        {
            emitInstruction16("1011110Prrrrrrrr", 1, 0);
            setRegisterValue(SP, INITIAL_SP - 4);
            setExpectedRegisterValue(SP, INITIAL_SP);
            setExpectedRegisterValue(PC, INITIAL_PC + 16);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, (INITIAL_PC + 16) | 1, READ_ONLY);
            pinkySimStep(&m_context);
        }),

        cpu_test("pop : JustPopR0", []
        {
            emitInstruction16("1011110Prrrrrrrr", 0, 1);
            setRegisterValue(SP, INITIAL_SP - 4);
            setExpectedRegisterValue(SP, INITIAL_SP);
            setExpectedRegisterValue(R0, 0xFFFFFFFF);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0xFFFFFFFF, READ_ONLY);
            pinkySimStep(&m_context);
        }),

        cpu_test("pop : JustPopR7", []
        {
            emitInstruction16("1011110Prrrrrrrr", 0, (1 << 7));
            setRegisterValue(SP, INITIAL_SP - 4);
            setExpectedRegisterValue(SP, INITIAL_SP);
            setExpectedRegisterValue(R7, 0xFFFFFFFF);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0xFFFFFFFF, READ_ONLY);
            pinkySimStep(&m_context);
        }),

        cpu_test("pop : PopAll", []
        {
            emitInstruction16("1011110Prrrrrrrr", 1, 0xFF);
            setRegisterValue(SP, INITIAL_SP - 4 * 9);
            setExpectedRegisterValue(SP, INITIAL_SP);
            setExpectedRegisterValue(R0, 9);
            setExpectedRegisterValue(R1, 8);
            setExpectedRegisterValue(R2, 7);
            setExpectedRegisterValue(R3, 6);
            setExpectedRegisterValue(R4, 5);
            setExpectedRegisterValue(R5, 4);
            setExpectedRegisterValue(R6, 3);
            setExpectedRegisterValue(R7, 2);
            setExpectedRegisterValue(PC, 1 & ~1);
            for (int i = 1 ; i <= 9 ; i++)
                SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4 * i, i, READ_ONLY);
            pinkySimStep(&m_context);
        }),

#if HARD_FAULT
        cpu_test("pop : PopToSetPCToEvenAddressWhichGeneratesHardFault", []
        {
            emitInstruction16("1011110Prrrrrrrr", 1, 0);
            setExpectedXPSRflags("t");
            setRegisterValue(SP, INITIAL_SP - 4);
            setExpectedRegisterValue(SP, INITIAL_SP);
            setExpectedRegisterValue(PC, INITIAL_PC + 16);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, INITIAL_PC + 16, READ_ONLY);
            pinkySimStep(&m_context);

            const uint16_t NOP = 0xBF00;
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16, NOP, READ_ONLY);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            pinkySimStep(&m_context);
        }),
#endif

#if UNPREDICTABLE
        TEST_SIM_ONLY(pop, HardFaultFromInvalidMemoryRead)
        {
            emitInstruction16("1011110Prrrrrrrr", 0, 1);
            setRegisterValue(SP, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(pop, UnpredictableToPopNoRegisters)
        {
            emitInstruction16("1011110Prrrrrrrr", 0, 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* PUSH
   Encoding: 1011 0 10 M:1 RegisterList:8 */
        cpu_test("push : JustPushLR", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 1, 0);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0x0, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(INITIAL_LR, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push : JustPushR0", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 0, 1);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x0, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push : JustPushR7", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 0, 1 << 7);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x77777777, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push : PushAll", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 1, 0xFF);
            setExpectedRegisterValue(SP, INITIAL_SP - 4 * 9);
            for (int i = 1 ; i <= 9 ; i++)
                SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4 * i, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            for (int i = 0 ; i < 8 ; i++)
                CHECK_EQUAL(0x11111111U * i, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4 * (9 - i)));
            CHECK_EQUAL(INITIAL_LR, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
        cpu_test("push_MSPLIM_nooverflow : JustPushLR", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 1, 0);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            setMSPLIM(INITIAL_SP - 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0x0, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(INITIAL_LR, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push_MSPLIM_nooverflow : JustPushR0", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 0, 1);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            setMSPLIM(INITIAL_SP - 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x0, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push_MSPLIM_nooverflow : JustPushR7", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 0, 1 << 7);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            setMSPLIM(INITIAL_SP - 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x77777777, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push_MSPLIM_nooverflow : PushAll", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 1, 0xFF);
            setExpectedRegisterValue(SP, INITIAL_SP - 4 * 9);
            setMSPLIM(INITIAL_SP - 4 * 9);
            for (int i = 1 ; i <= 9 ; i++)
                SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4 * i, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            for (int i = 0 ; i < 8 ; i++)
                CHECK_EQUAL(0x11111111U * i, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4 * (9 - i)));
            CHECK_EQUAL(INITIAL_LR, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push_MSPLIM_overflow : JustPushLR", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 1, 0);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            setMSPLIM(INITIAL_SP);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0x12345678, READ_WRITE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT); // we do breakpoint on stack overflow
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x12345678, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push_MSPLIM_overflow : JustPushR0", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 0, 1);
            setMSPLIM(INITIAL_SP);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0x12345678, READ_WRITE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT); // we do breakpoint on stack overflow
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x12345678, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push_MSPLIM_overflow : JustPushR7", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 0, 1 << 7);
            setMSPLIM(INITIAL_SP);
            setExpectedRegisterValue(SP, INITIAL_SP - 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4, 0x12345678, READ_WRITE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT); // we do breakpoint on stack overflow
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x12345678, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4));
        }),

        cpu_test("push_MSPLIM_overflow : PushAll", []
        {
            emitInstruction16("1011010Mrrrrrrrr", 1, 0xFF);
            setExpectedRegisterValue(SP, INITIAL_SP - 4 * 9);
            setMSPLIM(INITIAL_SP - 4 * 8);
            for (int i = 1 ; i <= 9 ; i++)
                SimpleMemory_SetMemory(m_context.pMemory, INITIAL_SP - 4 * i, 0x12345678 + i, READ_WRITE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT); // we do breakpoint on stack overflow
            pinkySimStep(&m_context);
            for (int i = 1 ; i <= 9 ; i++)
                CHECK_EQUAL(0x12345678 + i, IMemory_Read32(m_context.pMemory, INITIAL_SP - 4 * i));
        }),
#endif
#if HARD_FAULT
        TEST_SIM_ONLY(push, HardFaultFromInvalidMemoryWrite)
        {
            emitInstruction16("1011010Mrrrrrrrr", 0, 1);
            setRegisterValue(SP, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif
        
#if UNPREDICTABLE
        TEST_SIM_ONLY(push, UnpredictableToPushNoRegisters)
        {
            emitInstruction16("1011010Mrrrrrrrr", 0, 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* REV16
   Encoding: 1011 1010 01 Rm:3 Rd:3 */
        cpu_test("rev16 : RevR0toR7", []
        {
            emitInstruction16("1011101001mmmddd", R0, R7);
            setRegisterValue(R0, 0x12345678);
            setExpectedRegisterValue(R7, 0x34127856);
            pinkySimStep(&m_context);
        }),

        cpu_test("rev16 : RevR7toR0", []
        {
            emitInstruction16("1011101001mmmddd", R7, R0);
            setRegisterValue(R7, 0x12345678);
            setExpectedRegisterValue(R0, 0x34127856);
            pinkySimStep(&m_context);
        }),

        /* REV
   Encoding: 1011 1010 00 Rm:3 Rd:3 */
        cpu_test("rev : RevR0toR7", []
        {
            emitInstruction16("1011101000mmmddd", R0, R7);
            setRegisterValue(R0, 0x12345678);
            setExpectedRegisterValue(R7, 0x78563412);
            pinkySimStep(&m_context);
        }),

        cpu_test("rev : RevR7toR0", []
        {
            emitInstruction16("1011101000mmmddd", R7, R0);
            setRegisterValue(R7, 0x12345678);
            setExpectedRegisterValue(R0, 0x78563412);
            pinkySimStep(&m_context);
        }),
/* REVSH
   Encoding: 1011 1010 11 Rm:3 Rd:3 */
        cpu_test("revsh : RevR0toR7", []
        {
            emitInstruction16("1011101011mmmddd", R0, R7);
            setRegisterValue(R0, 0x12345678);
            setExpectedRegisterValue(R7, 0x7856);
            pinkySimStep(&m_context);
        }),

        cpu_test("revsh : RevR7toR0", []
        {
            emitInstruction16("1011101011mmmddd", R7, R0);
            setRegisterValue(R7, 0x12345678);
            setExpectedRegisterValue(R0, 0x7856);
            pinkySimStep(&m_context);
        }),

        cpu_test("revsh : PositiveValue", []
        {
            emitInstruction16("1011101011mmmddd", R7, R0);
            setRegisterValue(R7, 0xFF7F);
            setExpectedRegisterValue(R0, 0x7FFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("revsh : NegativeValue", []
        {
            emitInstruction16("1011101011mmmddd", R7, R0);
            setRegisterValue(R7, 0x0080);
            setExpectedRegisterValue(R0, 0xFFFF8000);
            pinkySimStep(&m_context);
        }),

        /* ROR - Register (ROtate Right)
   Encoding: 010000 0111 Rm:3 Rdn:3 */
        cpu_test("rorRegister : Rotate1by1_CarryOutFromLowestBit", []
        {
            emitInstruction16("0100000111mmmddd", R0, R7);
            setExpectedXPSRflags("NzC");
            setRegisterValue(R0, 1);
            setRegisterValue(R7, 1);
            setExpectedRegisterValue(R7, 0x80000000);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : Rotate1by0_MinimumShift_CarryUnmodified", []
        {
            emitInstruction16("0100000111mmmddd", R7, R0);
            setExpectedXPSRflags("nz");
            setRegisterValue(R0, 1);
            setRegisterValue(R7, 0);
            setExpectedRegisterValue(R0, 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : Rotate2by1_NoCarry", []
        {
            emitInstruction16("0100000111mmmddd", R3, R2);
            setExpectedXPSRflags("nzc");
            setRegisterValue(R2, 2);
            setRegisterValue(R3, 1);
            setExpectedRegisterValue(R2, 2 >> 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : Rotate16Bits", []
        {
            emitInstruction16("0100000111mmmddd", R3, R2);
            setExpectedXPSRflags("nzc");
            setRegisterValue(R2, 0x12345678);
            setRegisterValue(R3, 16);
            setExpectedRegisterValue(R2, 0x56781234);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : RotateWithShiftOf31", []
        {
            emitInstruction16("0100000111mmmddd", R3, R2);
            setExpectedXPSRflags("nzc");
            setRegisterValue(R2, 0x80000000);
            setRegisterValue(R3, 31);
            setExpectedRegisterValue(R2, 0x00000001);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : RotateBy32_CarryOutHighestBit", []
        {
            emitInstruction16("0100000111mmmddd", R7, R0);
            setExpectedXPSRflags("NzC");
            setRegisterValue(R0, 0x80000000);
            setRegisterValue(R7, 32);
            setExpectedRegisterValue(R0, 0x80000000);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : RotateBy33", []
        {
            emitInstruction16("0100000111mmmddd", R3, R2);
            setExpectedXPSRflags("NzC");
            setRegisterValue(R2, 0x80000001);
            setRegisterValue(R3, 33);
            setExpectedRegisterValue(R2, 0xC0000000);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : RotateWithMaximumShiftOf255", []
        {
            emitInstruction16("0100000111mmmddd", R3, R2);
            setExpectedXPSRflags("nzc");
            setRegisterValue(R2, 0x80000000);
            setRegisterValue(R3, 255);
            setExpectedRegisterValue(R2, 0x00000001);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : RotateWithShiftOf256_ShouldBeTreatedAs0Shift_CarryUnmodified", []
        {
            emitInstruction16("0100000111mmmddd", R7, R0);
            setExpectedXPSRflags("Nz");
            setRegisterValue(R0, 0x80000000);
            setRegisterValue(R7, 256);
            setExpectedRegisterValue(R0, 0x80000000);
            pinkySimStep(&m_context);
        }),

        cpu_test("rorRegister : Rotate0by16", []
        {
            emitInstruction16("0100000111mmmddd", R7, R0);
            setExpectedXPSRflags("nZc");
            setRegisterValue(R0, 0);
            setRegisterValue(R7, 16);
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        /* RSB - Immediate
   Encoding: 010000 1001 Rn:3 Rd:3 */
        cpu_test("rsbImmediate : UseLowestRegisterOnly", []
        {
            emitInstruction16("0100001001nnnddd", R0, R0);
            setExpectedXPSRflags("nZCv");
            setExpectedRegisterValue(R0, 0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("rsbImmediate : UseHigestRegisterOnly", []
        {
            emitInstruction16("0100001001nnnddd", R7, R7);
            setExpectedXPSRflags("Nzcv");
            setExpectedRegisterValue(R7, -0x77777777U);
            pinkySimStep(&m_context);
        }),

        cpu_test("rsbImmediate : UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("0100001001nnnddd", R2, R0);
            setExpectedXPSRflags("Nzcv");
            setExpectedRegisterValue(R0, -0x22222222);
            pinkySimStep(&m_context);
        }),

        cpu_test("rsbImmediate : ForceOverflowByNegatingLargestNegativeValue", []
        {
            emitInstruction16("0100001001nnnddd", R0, R7);
            setExpectedXPSRflags("NzcV");
            setRegisterValue(R0, 0x80000000);
            setExpectedRegisterValue(R7, 0x80000000U);
            pinkySimStep(&m_context);
        }),

        /* SBC - Register (SUBtract with Carry)
   Encoding: 010000 0110 Rm:3 Rdn:3 */
        cpu_test("sbcRegister : UseLowestRegisterForAllArgsAndShouldBeZeroWithCarrySetForNoBorrow", []
        {
            emitInstruction16("0100000110mmmddd", R0, R0);
            setExpectedXPSRflags("nZCv");
            setExpectedRegisterValue(R0, 0);
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("sbcRegister : UseHigestRegisterForAllArgsAndShouldBeZeroWithCarrySetForNoBorrow", []
        {
            emitInstruction16("0100000110mmmddd", R7, R7);
            setExpectedXPSRflags("nZCv");
            setExpectedRegisterValue(R7, 0);
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("sbcRegister : UseDifferentRegistersForEachArgAndOnlyCarryShouldBeSetToIndicateNoBorrow", []
        {
            emitInstruction16("0100000110mmmddd", R1, R2);
            setExpectedXPSRflags("nzCv");
            setExpectedRegisterValue(R2, 0x22222222U - 0x11111111U);
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("sbcRegister : ForceCarryClearToIndicateBorrowAndResultWillBeNegative", []
        {
            emitInstruction16("0100000110mmmddd", R1, R0);
            setExpectedXPSRflags("Nzcv");
            setRegisterValue(R1, 1);
            setExpectedRegisterValue(R0, 0U - 1U);
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("sbcRegister : ForceNegativeOverflow", []
        {
            emitInstruction16("0100000110mmmddd", R1, R2);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R2, 0x80000000U);
            setRegisterValue(R1, 1U);
            setExpectedRegisterValue(R2, 0x80000000U - 1);
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("sbcRegister : ForcePositiveOverflow", []
        {
            emitInstruction16("0100000110mmmddd", R1, R2);
            setExpectedXPSRflags("NzcV");
            setRegisterValue(R2, 0x7FFFFFFFU);
            setRegisterValue(R1, -1U);
            setExpectedRegisterValue(R2, 0x7FFFFFFFu + 1u);
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("sbcRegister : ClearCarryToCauseABorrowToOccur", []
        {
            emitInstruction16("0100000110mmmddd", R1, R2);
            setExpectedXPSRflags("nzCv");
            setExpectedRegisterValue(R2, 0x22222222U - 1U - 0x11111111U);
            clearCarry(); // Causes borrow.
            pinkySimStep(&m_context);
        }),

#if ARMULET_FEATURE_ARMV8M_BASELINE_SDIV_UDIV
        cpu_test("sdiv : low regs", []
        {
            emitInstruction32("111110111001nnnn", "1111dddd1111mmmm", R3, R2, R7);
            setRegisterValue(R3, (uint32_t)-300);
            setRegisterValue(R7, (uint32_t)21);
            // don't expect to update flags
            setExpectedXPSRflags("nZcv");
            clearNegative();
            setZero();
            clearCarry();
            clearOverflow();
            setExpectedRegisterValue(R2, (uint)-14);
            pinkySimStep(&m_context);
        }),

        cpu_test("sdiv : hi regs", []
        {
            emitInstruction32("111110111001nnnn", "1111dddd1111mmmm", R9, R11, R10);
            setRegisterValue(R9, (uint32_t)300);
            setRegisterValue(R10, (uint32_t)-21);
            // don't expect to update flags
            setExpectedXPSRflags("nZcv");
            clearNegative();
            setZero();
            clearCarry();
            clearOverflow();
            setExpectedRegisterValue(R11, (uint)-14);
            pinkySimStep(&m_context);
        }),


        cpu_test("sdiv : divByZero", []
        {
            emitInstruction32("111110111001nnnn", "1111dddd1111mmmm", R9, R11, R10);
            setRegisterValue(R9, (uint32_t)300);
            setRegisterValue(R10, 0);
            // don't expect to update flags
            setExpectedXPSRflags("nZcv");
            clearNegative();
            setZero();
            clearCarry();
            clearOverflow();
            setExpectedRegisterValue(R11, 0);
            pinkySimStep(&m_context);
        }),


#endif

/* SEV
   Encoding: 1011 1111 0100 0000 */
        cpu_test("sev : BasicTest", []
        {
            emitInstruction16("1011111101000000");
            setExpectedStepReturn(PINKYSIM_STEP_WFE);
            pinkySimStep(&m_context);
        }),

        /* PUSH
   Encoding: 1100 0 Rn:3 RegisterList:8 */
        cpu_test("stm : JustPushR0WithR7AsAddress", []
        {
            emitInstruction16("11000nnnrrrrrrrr", R7, (1 << 0));
            setRegisterValue(R7, INITIAL_PC + 16);
            setExpectedRegisterValue(R7, INITIAL_PC + 16 + 1 * 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x0, IMemory_Read32(m_context.pMemory, INITIAL_PC + 16));
        }),

        cpu_test("stm : JustPushR7WithR0AsAddress", []
        {
            emitInstruction16("11000nnnrrrrrrrr", R0, (1 << 7));
            setRegisterValue(R0, INITIAL_PC + 16);
            setExpectedRegisterValue(R0, INITIAL_PC + 16 + 1 * 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x77777777, IMemory_Read32(m_context.pMemory, INITIAL_PC + 16));
        }),

        cpu_test("stm : PushAllWithR0AsAddress", []
        {
            emitInstruction16("11000nnnrrrrrrrr", R0, 0xFF);
            setRegisterValue(R0, INITIAL_PC + 16);
            setExpectedRegisterValue(R0, INITIAL_PC + 16 + 8 * 4);
            for (int i = 0 ; i < 8 ; i++)
                SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16 + 4 * i, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(INITIAL_PC + 16, IMemory_Read32(m_context.pMemory, INITIAL_PC + 16 + 4 * 0));
            for (int i = 1 ; i < 8 ; i++)
                CHECK_EQUAL(0x11111111U * i, IMemory_Read32(m_context.pMemory, INITIAL_PC + 16 + 4 * i));
        }),

        cpu_test("stm : PushAllButR7WithR7AsAddress", []
        {
            emitInstruction16("11000nnnrrrrrrrr", R7, 0x7F);
            setRegisterValue(R7, INITIAL_PC + 16);
            setExpectedRegisterValue(R7, INITIAL_PC + 16 + 7 * 4);
            for (int i = 0 ; i < 7 ; i++)
                SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 16 + 4 * i, 0xFFFFFFFF, READ_WRITE);
            pinkySimStep(&m_context);
            for (int i = 0 ; i < 7 ; i++)
                CHECK_EQUAL(0x11111111U * i, IMemory_Read32(m_context.pMemory, INITIAL_PC + 16 + 4 * i));
        }),

#if HARD_FAULT
        cpu_test("stm : HardFaultFromInvalidMemoryWrite", []
        {
            emitInstruction16("11000nnnrrrrrrrr", R0, 1 << 0);
            setRegisterValue(R0, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

#if UNPREDICTABLE
        TEST_SIM_ONLY(stm, UnpredictableToPushNoRegisters)
        {
            emitInstruction16("11000nnnrrrrrrrr", R0, 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(stm, UnpredictableToPushWritebackRegisterWhichIsntFirstSaved)
        {
            emitInstruction16("11000nnnrrrrrrrr", R7, 0xFF);
            setRegisterValue(R7, INITIAL_PC + 16);
            setExpectedStepReturn(PINKYSIM_STEP_UNPREDICTABLE);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* STRB - Immediate
   Encoding: 011 1 0 Imm:5 Rn:3 Rt:3 */
        cpu_test("strbImmediate : UseAMixOfRegistersWordAlignedWithSmallestOffset", []
        {
            emitInstruction16("01110iiiiinnnttt", 0, R7, R0);
            setRegisterValue(R7, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBAADFE00, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strbImmediate : UseAnotherMixOfRegistersSecondByteInWord", []
        {
            emitInstruction16("01110iiiiinnnttt", 1, R0, R7);
            setRegisterValue(R0, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBAAD77ED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strbImmediate : YetAnotherMixOfRegistersThirdByteInWord", []
        {
            emitInstruction16("01110iiiiinnnttt", 2, R3, R4);
            setRegisterValue(R3, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBA44FEED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strbImmediate : YetAnotherMixOfRegistersFourthByteInWord", []
        {
            emitInstruction16("01110iiiiinnnttt", 3, R1, R5);
            setRegisterValue(R1, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x55ADFEED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strbImmediate : LargestOffset", []
        {
            emitInstruction16("01110iiiiinnnttt", 31, R2, R4);
            setRegisterValue(R2, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 28, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x44ADFEED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 28));
        }),
#if HARD_FAULT
        cpu_test("strbImmediate : AttemptStoreToInvalidAddress", []
        {
            emitInstruction16("01110iiiiinnnttt", 0, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* STRB - Register
   Encoding: 0101 010 Rm:3 Rn:3 Rt:3 */
        cpu_test("strbRegister : UseAMixOfRegistersWordAligned", []
        {
            emitInstruction16("0101010mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBAADFE00, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strbRegister : UseAnotherMixOfRegistersSecondByteInWord", []
        {
            emitInstruction16("0101010mmmnnnttt", R1, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            setRegisterValue(R1, 5);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBAAD77ED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strbRegister : YetAnotherMixOfRegistersThirdByteInWord", []
        {
            emitInstruction16("0101010mmmnnnttt", R0, R7, R4);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 6);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBA44FEED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strbRegister : YetAnotherMixOfRegistersFourthByteInWord", []
        {
            emitInstruction16("0101010mmmnnnttt", R0, R7, R5);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 7);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x55ADFEED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

#if HARD_FAULT
        cpu_test("strbRegister : AttemptStoreToInvalidAddress", []
        {
            emitInstruction16("0101010mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setRegisterValue(R7, 0);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* STRH - Immediate
   Encoding: 1000 0 Imm:5 Rn:3 Rt:3 */
        cpu_test("strhImmediate : UseAMixOfRegistersWordAlignedWithSmallestOffset", []
        {
            emitInstruction16("10000iiiiinnnttt", 0, R7, R0);
            setRegisterValue(R7, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBAAD0000, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strhImmediate : AnotherMixOfRegistersNotWordAligned", []
        {
            emitInstruction16("10000iiiiinnnttt", 1, R0, R7);
            setRegisterValue(R0, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x7777FEED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strhImmediate : LargestOffset", []
        {
            emitInstruction16("10000iiiiinnnttt", 31, R1, R6);
            setRegisterValue(R1, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 60, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x6666FEED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 60));
        }),
#if HARD_FAULT
        cpu_test("strhImmediate : AttemptStoreToInvalidAddress", []
        {
            emitInstruction16("10000iiiiinnnttt", 0, R3, R1);
            setRegisterValue(R3, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* STRH - Register
   Encoding: 0101 001 Rm:3 Rn:3 Rt:3 */
        cpu_test("strhRegister : UseAMixOfRegistersWordAligned", []
        {
            emitInstruction16("0101001mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBAAD0000, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strhRegister : UseAnotherMixOfRegistersWordAligned", []
        {
            emitInstruction16("0101001mmmnnnttt", R1, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            setRegisterValue(R1, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0xBAAD7777, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strhRegister : YetAnotherMixOfRegistersNotWordAligned", []
        {
            emitInstruction16("0101001mmmnnnttt", R0, R7, R4);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 6);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x4444FEED, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

#if HARD_FAULT
        cpu_test("strhRegister : AttemptUnalignedStore", []
        {
            emitInstruction16("0101001mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC + 1024);
            setRegisterValue(R7, 1);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 1024, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
        }),

        cpu_test("strhRegister : AttemptStoreToInvalidAddress", []
        {
            emitInstruction16("0101001mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setRegisterValue(R7, 0);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* STR - Immediate Encoding T1
   Encoding: 011 0 0 Imm:5 Rn:3 Rt:3 */
        cpu_test("strImmediate : T1UseAMixOfRegistersWithSmallestImmediateOffset", []
        {
            emitInstruction16("01100iiiiinnnttt", 0, R7, R0);
            setRegisterValue(R7, INITIAL_PC + 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x00000000, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strImmediate : T1UseAnotherMixOfRegistersWithLargestImmediateOffset", []
        {
            emitInstruction16("01100iiiiinnnttt", 31, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 31 * 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x77777777, IMemory_Read32(m_context.pMemory, INITIAL_PC + 31 * 4));
        }),

#if HARD_FAULT
        cpu_test("strImmediate : T1AttemptUnalignedStore", []
        {
            emitInstruction16("01100iiiiinnnttt", 0, R3, R2);
            setRegisterValue(R3, INITIAL_PC + 2);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        cpu_test("strImmediate : T1AttemptStoreToInvalidAddress", []
        {
            emitInstruction16("01100iiiiinnnttt", 16, R3, R2);
            setRegisterValue(R3, 0xFFFFFFFC - 16 * 4);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* STR - Immediate Encoding T2 (SP is base register)
   Encoding: 1001 0 Rt:3 Imm:8 */
        cpu_test("strImmediate : T2HighestRegisterWithSmallestImmediateOffset", []
        {
            emitInstruction16("10010tttiiiiiiii", R7, 0);
            setRegisterValue(SP, INITIAL_PC + 1024);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 1024, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x77777777, IMemory_Read32(m_context.pMemory, INITIAL_PC + 1024));
        }),

        cpu_test("strImmediate : T2LowestRegisterWithLargestImmediateOffset", []
        {
            emitInstruction16("10010tttiiiiiiii", R0, 255);
            setRegisterValue(SP, INITIAL_PC + 1024);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 1024 + 255 * 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x00000000, IMemory_Read32(m_context.pMemory, INITIAL_PC + 1024 + 255 * 4));
        }),

#if HARD_FAULT
        TEST_SIM_ONLY(strImmediate, T2AttemptUnalignedStore)
        {
            emitInstruction16("10010tttiiiiiiii", R2, 0);
            setRegisterValue(SP, INITIAL_PC + 1026);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 1024, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
        }),

        TEST_SIM_ONLY(strImmediate, T2AttemptStoreToInvalidAddress)
        {
            emitInstruction16("10010tttiiiiiiii", R2, 0);
            setRegisterValue(SP, 0xFFFFFFFC);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif
/* STR - Register
   Encoding: 0101 000 Rm:3 Rn:3 Rt:3 */
        cpu_test("strRegister : UseAMixOfRegisters", []
        {
            emitInstruction16("0101000mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x00000000, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strRegister : UseAnotherMixOfRegisters", []
        {
            emitInstruction16("0101000mmmnnnttt", R1, R0, R7);
            setRegisterValue(R0, INITIAL_PC);
            setRegisterValue(R1, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x77777777, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),

        cpu_test("strRegister : YetAnotherMixOfRegisters", []
        {
            emitInstruction16("0101000mmmnnnttt", R0, R7, R4);
            setRegisterValue(R7, INITIAL_PC);
            setRegisterValue(R0, 4);
            SimpleMemory_SetMemory(m_context.pMemory, INITIAL_PC + 4, 0xBAADFEED, READ_WRITE);
            pinkySimStep(&m_context);
            CHECK_EQUAL(0x44444444, IMemory_Read32(m_context.pMemory, INITIAL_PC + 4));
        }),
#if HARD_FAULT
        cpu_test("strRegister : AttemptUnalignedStore", []
        {
            emitInstruction16("0101000mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, INITIAL_PC);
            setRegisterValue(R7, 2);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),

        cpu_test("strRegister : AttemptStoreToInvalidAddress", []
        {
            emitInstruction16("0101000mmmnnnttt", R7, R3, R0);
            setRegisterValue(R3, 0xFFFFFFFC);
            setRegisterValue(R7, 0);
            setExpectedStepReturn(PINKYSIM_STEP_HARDFAULT);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif
/* SUB - Immediate - Encoding T1
   Encoding: 000 11 1 1 Imm:3 Rn:3 Rd:3 */
        cpu_test("subImmediate : T1UseLowestRegisterOnly_SmallestImmediate", []
        {
            emitInstruction16("0001111iiinnnddd", 0, R0, R0);
            setExpectedXPSRflags("nZCv");
            setExpectedRegisterValue(R0, 0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("subImmediate : T1UseHigestRegisterOnly_LargestImmediate", []
        {
            emitInstruction16("0001111iiinnnddd", 7, R7, R7);
            setExpectedXPSRflags("nzCv");
            setExpectedRegisterValue(R7, 0x77777777U - 7U);
            pinkySimStep(&m_context);
        }),

        cpu_test("subImmediate : T1UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("0001111iiinnnddd", 3, R0, R2);
            setExpectedXPSRflags("Nzcv");
            setExpectedRegisterValue(R2, 0U - 3U);
            pinkySimStep(&m_context);
        }),

        cpu_test("subImmediate : T1ForceOverflowPastLargestNegativeInteger", []
        {
            emitInstruction16("0001111iiinnnddd", 1, R1, R6);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R1, 0x80000000);
            setExpectedRegisterValue(R6, 0x80000000U - 1U);
            pinkySimStep(&m_context);
        }),



/* SUB - Immediate - Encoding T2
   Encoding: 001 11 Rdn:3 Imm:8 */
        cpu_test("subImmediate : T2LowestRegister_SmallestImmediate", []
        {
            emitInstruction16("00111dddiiiiiiii", R0, 0);
            setExpectedXPSRflags("nZCv");
            setExpectedRegisterValue(R0, 0U);
            pinkySimStep(&m_context);
        }),

        cpu_test("subImmediate : T2HigestRegister_LargestImmediate", []
        {
            emitInstruction16("00111dddiiiiiiii", R7, 255);
            setExpectedXPSRflags("nzCv");
            setExpectedRegisterValue(R7, 0x77777777U - 255U);
            pinkySimStep(&m_context);
        }),

        cpu_test("subImmediate : T2Subtract127FromR0CausesNoCarryToIndicateBorrowAndNegativeResult", []
        {
            emitInstruction16("00111dddiiiiiiii", R0, 127);
            setExpectedXPSRflags("Nzcv");
            setExpectedRegisterValue(R0, 0U - 127U);
            pinkySimStep(&m_context);
        }),

        cpu_test("subImmediate : T2ForceOverflowPastLargestNegativeInteger", []
        {
            emitInstruction16("00111dddiiiiiiii", R3, 1);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R3, 0x80000000);
            setExpectedRegisterValue(R3, 0x80000000U - 1U);
            pinkySimStep(&m_context);
        }),

        /* SUB - Register
   Encoding: 000 11 0 1 Rm:3 Rn:3 Rd:3 */
        cpu_test("subRegister : UseLowestRegisterForAllArgs", []
        {
            emitInstruction16("0001101mmmnnnddd", R0, R0, R0);
            setExpectedXPSRflags("nZCv");
            setExpectedRegisterValue(R0, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("subRegister : UseHigestRegisterForAllArgs", []
        {
            emitInstruction16("0001101mmmnnnddd", R7, R7, R7);
            setExpectedXPSRflags("nZCv");
            setExpectedRegisterValue(R7, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("subRegister : UseDifferentRegistersForEachArg", []
        {
            emitInstruction16("0001101mmmnnnddd", R1, R2, R0);
            setExpectedXPSRflags("nzCv");
            setExpectedRegisterValue(R0, 0x22222222U - 0x11111111U);
            pinkySimStep(&m_context);
        }),

// Force APSR flags to be set which haven't already been covered above.
        cpu_test("subRegister : ForceCarryClearToIndicateBorrowAndResultWillBeNegative", []
        {
            emitInstruction16("0001101mmmnnnddd", R1, R0, R2);
            setExpectedXPSRflags("Nzcv");
            setRegisterValue(R1, 1);
            setExpectedRegisterValue(R2, 0U - 1U);
            pinkySimStep(&m_context);
        }),

        cpu_test("subRegister : ForceNegativeOverflow", []
        {
            emitInstruction16("0001101mmmnnnddd", R1, R2, R0);
            setExpectedXPSRflags("nzCV");
            setRegisterValue(R2, 0x80000000U);
            setRegisterValue(R1, 1U);
            setExpectedRegisterValue(R0,0x80000000U - 1);
            pinkySimStep(&m_context);
        }),

        cpu_test("subRegister : ForcePositiveOverflow", []
        {
            emitInstruction16("0001101mmmnnnddd", R1, R2, R0);
            setExpectedXPSRflags("NzcV");
            setRegisterValue(R2, 0x7FFFFFFFU);
            setRegisterValue(R1, -1U);
            setExpectedRegisterValue(R0, 0x7FFFFFFFU + 1U);
            pinkySimStep(&m_context);
        }),

        /* SUB SP Minus Immediate
   Encoding: 1011 0000 1 Imm:7 */
        cpu_test("subSP : SmallestImmediate", []
        {
            emitInstruction16("101100001iiiiiii", 0);
            setExpectedRegisterValue(SP, INITIAL_SP - 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("subSP : LargestImmediate", []
        {
            emitInstruction16("101100001iiiiiii", 127);
            setExpectedRegisterValue(SP, INITIAL_SP - 127 * 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("subSP : UseIntermediateValues", []
        {
            emitInstruction16("101100001iiiiiii", 64);
            setExpectedRegisterValue(SP, INITIAL_SP - 64 * 4);
            pinkySimStep(&m_context);
        }),

#if ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM
        cpu_test("subSP_MSPLIM_nooverflow : UseIntermediateValues", []
        {
            emitInstruction16("101100001iiiiiii", 64);
            setMSPLIM(INITIAL_SP - 64 * 4);
            setExpectedRegisterValue(SP, INITIAL_SP - 64 * 4);
            pinkySimStep(&m_context);
        }),

        cpu_test("subSP_MSPLIM_overflow : UseIntermediateValues", []
        {
            emitInstruction16("101100001iiiiiii", 64);
            setMSPLIM(INITIAL_SP - 63 * 4);
            setExpectedRegisterValue(SP, INITIAL_SP - 64 * 4);
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_BKPT); // we do breakpoint on stack overflow
            pinkySimStep(&m_context);
        }),
#endif

        /* SVC
   Encoding: 1101 1111 Imm:8 */
        cpu_test("svc : SmallestImmediate", []
        {
            emitInstruction16("11011111iiiiiiii", 0);
            setExpectedStepReturn(PINKYSIM_STEP_SVC);
            pinkySimStep(&m_context);
        }),

        cpu_test("svc : LargestImmediate", []
        {
            emitInstruction16("11011111iiiiiiii", 255);
            setExpectedStepReturn(PINKYSIM_STEP_SVC);
            pinkySimStep(&m_context);
        }),

        /* SXTB (Sign ExTend Byte)
   Encoding: 1011 0010 01 Rm:3 Rd:3 */
        cpu_test("sxtb : SignExtendLowestRegisterIntoHighestRegister_PositiveValue", []
        {
            emitInstruction16("1011001001mmmddd", R7, R0);
            setRegisterValue(R7, 0x7F);
            setExpectedRegisterValue(R0, 0x7F);
            pinkySimStep(&m_context);
        }),

        cpu_test("sxtb : SignExtendHighestRegisterIntoLowestRegister_NegativeValue", []
        {
            emitInstruction16("1011001001mmmddd", R0, R7);
            setRegisterValue(R0, 0x80);
            setExpectedRegisterValue(R7, 0xFFFFFF80);
            pinkySimStep(&m_context);
        }),

        cpu_test("sxtb : OverwriteUpperBits_PositiveValue", []
        {
            emitInstruction16("1011001001mmmddd", R6, R1);
            setRegisterValue(R6, 0xBADBAD7F);
            setExpectedRegisterValue(R1, 0x7F);
            pinkySimStep(&m_context);
        }),

        cpu_test("sxtb : OverwriteUpperBits_NegativeValue", []
        {
            emitInstruction16("1011001001mmmddd", R2, R5);
            setRegisterValue(R2, 0xBADBAD80);
            setExpectedRegisterValue(R5, 0xFFFFFF80);
            pinkySimStep(&m_context);
        }),

        /* SXTH (Sign ExTend Halfword)
   Encoding: 1011 0010 00 Rm:3 Rd:3 */
        cpu_test("sxth : SignExtendLowestRegisterIntoHighestRegister_PositiveValue", []
        {
            emitInstruction16("1011001000mmmddd", R7, R0);
            setRegisterValue(R7, 0x7FFF);
            setExpectedRegisterValue(R0, 0x7FFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("sxth : SignExtendHighestRegisterIntoLowestRegister_NegativeValue", []
        {
            emitInstruction16("1011001000mmmddd", R0, R7);
            setRegisterValue(R0, 0x8000);
            setExpectedRegisterValue(R7, 0xFFFF8000);
            pinkySimStep(&m_context);
        }),

        cpu_test("sxth : OverwriteUpperBits_PositiveValue", []
        {
            emitInstruction16("1011001000mmmddd", R6, R1);
            setRegisterValue(R6, 0xF00D7FFF);
            setExpectedRegisterValue(R1, 0x00007FFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("sxth : OverwriteUpperBits_NegativeValue", []
        {
            emitInstruction16("1011001000mmmddd", R2, R5);
            setRegisterValue(R2, 0xF00D8000);
            setExpectedRegisterValue(R5, 0xFFFF8000);
            pinkySimStep(&m_context);
        }),


/* TST - Register
   Encoding: 010000 1000 Rm:3 Rn:3 */
/* NOTE: APSR_C state is maintained by this instruction. */
        cpu_test("tstRegister : UseLowestRegisterForBothArgsAndResultShouldBeZero", []
        {
            emitInstruction16("0100001000mmmnnn", R0, R0);
            // Use a couple of tests to explicitly set/clear carry to verify both states are maintained.
            setExpectedXPSRflags("nZc");
            clearCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("tstRegister : UseHighestRegisterForBothArgsAndRegisterWillBeUnchanged", []
        {
            emitInstruction16("0100001000mmmnnn", R7, R7);
            setExpectedXPSRflags("nzC");
            setCarry();
            pinkySimStep(&m_context);
        }),

        cpu_test("tstRegister : AndR3andR7", []
        {
            emitInstruction16("0100001000mmmnnn", R3, R7);
            setExpectedXPSRflags("nz");
            pinkySimStep(&m_context);
        }),

        cpu_test("tstRegister : UseAndToJustKeepNegativeSignBit", []
        {
            emitInstruction16("0100001000mmmnnn", R7, R0);
            setRegisterValue(R0, -1);
            setRegisterValue(R7, 0x80000000);
            setExpectedXPSRflags("Nz");
            pinkySimStep(&m_context);
        }),

#if ARMULET_FEATURE_ARMV8M_BASELINE_SDIV_UDIV

        cpu_test("udiv : low regs", []
        {
            emitInstruction32("111110111011nnnn", "1111dddd1111mmmm", R3, R2, R7);
            setRegisterValue(R3, (uint32_t)-300);
            setRegisterValue(R7, (uint32_t)21);
            // don't expect to update flags
            setExpectedXPSRflags("nZcv");
            clearNegative();
            setZero();
            clearCarry();
            clearOverflow();
            setExpectedRegisterValue(R2, 204522237);
            pinkySimStep(&m_context);
        }),

        cpu_test("udiv : hi regs", []
        {
            emitInstruction32("111110111011nnnn", "1111dddd1111mmmm", R9, R11, R10);
            setRegisterValue(R9, (uint32_t)300);
            setRegisterValue(R10, (uint32_t)-21);
            // don't expect to update flags
            setExpectedXPSRflags("nZcv");
            clearNegative();
            setZero();
            clearCarry();
            clearOverflow();
            setExpectedRegisterValue(R11, 0);
            pinkySimStep(&m_context);
        }),

        cpu_test("udiv : hi regs", []
        {
            emitInstruction32("111110111011nnnn", "1111dddd1111mmmm", R9, R11, R10);
            setRegisterValue(R9, (uint32_t)300);
            setRegisterValue(R10, (uint32_t)21);
            // don't expect to update flags
            setExpectedXPSRflags("nZcv");
            clearNegative();
            setZero();
            clearCarry();
            clearOverflow();
            setExpectedRegisterValue(R11, (uint)14);
            pinkySimStep(&m_context);
        }),


        cpu_test("udiv : divByZero", []
        {
            emitInstruction32("111110111011nnnn", "1111dddd1111mmmm", R9, R11, R10);
            setRegisterValue(R9, (uint32_t)300);
            setRegisterValue(R10, 0);
            // don't expect to update flags
            setExpectedXPSRflags("nZcv");
            clearNegative();
            setZero();
            clearCarry();
            clearOverflow();
            setExpectedRegisterValue(R11, 0);
            pinkySimStep(&m_context);
        }),
#endif

#if UNDEFINED
        // todo more bit patterns
        cpu_test("undefined : Undedfined16BitWithAllZeroesForImmedaite", []
        {
            emitInstruction16("11011110iiiiiiii", 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNDEFINED16);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
        
        cpu_test("undefined : Undedfined16BitWithAllOnesForImmedaite", []
        {
            emitInstruction16("11011110iiiiiiii", -1);
            setExpectedStepReturn(PINKYSIM_STEP_UNDEFINED16);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
        
        cpu_test("undefined : Undefined32BitWithAllZeroesForImmediate", []
        {
            emitInstruction32("111101111111iiii", "1010iiiiiiiiiiii", 0, 0);
            setExpectedStepReturn(PINKYSIM_STEP_UNDEFINED32);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
        
        cpu_test("undefined : Undefined32BitWithAllOnesForImmediate", []
        {
            emitInstruction32("111101111111iiii", "1010iiiiiiiiiiii", -1, -1);
            setExpectedStepReturn(PINKYSIM_STEP_UNDEFINED32);
            setExpectedRegisterValue(PC, INITIAL_PC);
            pinkySimStep(&m_context);
        }),
#endif

/* UXTB (Unsigned ExTend Byte)
   Encoding: 1011 0010 11 Rm:3 Rd:3 */
        cpu_test("uxtb : ExtendLowestRegisterIntoHighestRegister_PositiveValue", []
        {
            emitInstruction16("1011001011mmmddd", R7, R0);
            setRegisterValue(R7, 0x7F);
            setExpectedRegisterValue(R0, 0x7F);
            pinkySimStep(&m_context);
        }),

        cpu_test("uxtb : ExtendHighestRegisterIntoLowestRegister_NegativeValue", []
        {
            emitInstruction16("1011001011mmmddd", R0, R7);
            setRegisterValue(R0, 0x80);
            setExpectedRegisterValue(R7, 0x80);
            pinkySimStep(&m_context);
        }),

        cpu_test("uxtb : OverwriteUpperBits_PositiveValue", []
        {
            emitInstruction16("1011001011mmmddd", R6, R1);
            setRegisterValue(R6, 0xBADBAD7F);
            setExpectedRegisterValue(R1, 0x7F);
            pinkySimStep(&m_context);
        }),

        cpu_test("uxtb : OverwriteUpperBits_NegativeValue", []
        {
            emitInstruction16("1011001011mmmddd", R2, R5);
            setRegisterValue(R2, 0xBADBAD80);
            setExpectedRegisterValue(R5, 0x80);
            pinkySimStep(&m_context);
        }),

        /* UXTH (Unsigned ExTend Halfword)
   Encoding: 1011 0010 10 Rm:3 Rd:3 */
        cpu_test("uxth : ExtendLowestRegisterIntoHighestRegister_PositiveValue", []
        {
            emitInstruction16("1011001010mmmddd", R7, R0);
            setRegisterValue(R7, 0x7FFF);
            setExpectedRegisterValue(R0, 0x7FFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("uxth : ExtendHighestRegisterIntoLowestRegister_NegativeValue", []
        {
            emitInstruction16("1011001010mmmddd", R0, R7);
            setRegisterValue(R0, 0x8000);
            setExpectedRegisterValue(R7, 0x8000);
            pinkySimStep(&m_context);
        }),

        cpu_test("uxth : OverwriteUpperBits_PositiveValue", []
        {
            emitInstruction16("1011001010mmmddd", R6, R1);
            setRegisterValue(R6, 0xF00D7FFF);
            setExpectedRegisterValue(R1, 0x7FFF);
            pinkySimStep(&m_context);
        }),

        cpu_test("uxth : OverwriteUpperBits_NegativeValue", []
        {
            emitInstruction16("1011001010mmmddd", R2, R5);
            setRegisterValue(R2, 0xF00D8000);
            setExpectedRegisterValue(R5, 0x8000);
            pinkySimStep(&m_context);
        }),

        /* WFE
   Encoding: 1011 1111 0010 0000 */
        cpu_test("wfe : BasicTest", []
        {
            emitInstruction16("1011111100100000");
            setExpectedStepReturn(PINKYSIM_STEP_WFE);
            pinkySimStep(&m_context);
        }),

        /* WFI
   Encoding: 1011 1111 0011 0000 */
        cpu_test("wfi : BasicTest", []
        {
            emitInstruction16("1011111100110000");
            setExpectedStepReturn(PINKYSIM_STEP_WFI);
            pinkySimStep(&m_context);
        }),
};

uint32_t armulet_cb_special_read(uint32_t addr, int size) {
    return 0;
}

void armulet_cb_special_write(uint32_t addr, int size, uint32_t value) {
}

// 4547 cmp r7, r8
#if PICO_ON_DEVICE
#include "pico/stdlib.h"
#else
#ifndef count_of
#define count_of(x) (sizeof(x) / sizeof((x)[0]))
#endif
#endif
int main(int argc, char **argv) {
#if __riscv
    // Start and end points of the constructor list,
    // defined by the linker script.
    extern void (*__preinit_array_start)(void);
    extern void (*__preinit_array_end)(void);

    // Call each function in the list.
    // We have to take the address of the symbols, as __preinit_array_start *is*
    // the first function pointer, not the address of it.
    for (void (**p)(void) = &__preinit_array_start; p < &__preinit_array_end; ++p) {
        (*p)();
    }
    // Start and end points of the constructor list,
    // defined by the linker script.
    extern void (*__init_array_start)(void);
    extern void (*__init_array_end)(void);

    // Call each function in the list.
    // We have to take the address of the symbols, as __init_array_start *is*
    // the first function pointer, not the address of it.
    for (void (**p)(void) = &__init_array_start; p < &__init_array_end; ++p) {

        (*p)();
    }

#endif
#if PICO_ON_DEVICE
    stdio_init_all();
#endif
#if ARMULET_USE_ASM
    single_step_asm_hooks = varmulet_default_asm_hooks;
    install_varmulet_test_hooks(&single_step_asm_hooks);
#endif
    int i=0;
    printf("CPU at %p\n", &cpu);

    for(auto &test : tests) {
        printf("%d / %d, %s\n", i+1, (int)tests.size(), test.name.c_str());
        setup();
        test();
        teardown();
        printf("  OK\n");
        i++;
    }

    printf("Checking undefined 16\n");
    const char *undefined16[] =
    {
#if !ARMULET_FEATURE_ARMV8M_BASELINE_CBZ_CBNZ
            "10110001xxxxxxxx",
            "10110011xxxxxxxx",
#endif
            "1011011000xxxxxx",
            "10110110010xxxxx",
            "101101101xxxxxxx",
            "10110111xxxxxxxx",
#if !ARMULET_FEATURE_ARMV8M_BASELINE_CBZ_CBNZ
            "1011100xxxxxxxxx",
#else
            "10111000xxxxxxxx",
#endif
            "1011101010xxxxxx",
#if !ARMULET_FEATURE_ARMV8M_BASELINE_CBZ_CBNZ
            "10111011xxxxxxxx",
#endif
            "11011110xxxxxxxx", // UDF
            "10110110011xxx0x", // CPS
            "10110110011x0011", // CPS
            "10110110011x0110", // CPS
            "10110110011x0111", // CPS
            "10110110011x1010", // CPS
            "10110110011x1011", // CPS
            "10110110011x1110", // CPS
            "10110110011x1111", // CPS
    };
    for(int i=0;i<count_of(undefined16);i++) {
        const char *p = undefined16[i];
        printf("-> %s\n", p);
        for (int x = 0; x<512; x++) {
            setup();
            emitInstruction16(p, x);
//            printf("%04x\n", armulet_read_u16(m_emitAddress - 2));
            setExpectedRegisterValue(PC, INITIAL_PC);
            setExpectedStepReturn(PINKYSIM_STEP_UNDEFINED16);
            pinkySimStep(&m_context);
            teardown();
        }
    }
#ifdef __riscv
    printf("DONE on RISC-V\n");
#else
    printf("DONE\n");
#endif
}
