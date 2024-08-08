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

#define RAM_START_ADDRESS 0x20070000u

//#define printf(...) ((void)0)

const uint32_t r0 = 0;
const uint32_t r1 = 1;
const uint32_t r2 = 2;
const uint32_t r3 = 3;
const uint32_t r4 = 4;
const uint32_t r5 = 5;
const uint32_t r6 = 6;
const uint32_t r7 = 7;
const uint32_t r8 = 8;
const uint32_t r9 = 9;
const uint32_t r10 = 10;
const uint32_t r11 = 11;
const uint32_t r12 = 12;
const uint32_t ip = 12;
const uint32_t sp = 13;
const uint32_t lr = 14;
const uint32_t pc = 15;

const uint32_t VTOR = 0xe000ed08;
const uint32_t EXC_SVCALL = 11;

//describe('Cortex-M0+ Instruction Set', () => {
//let cpu: ICortexTestDriver;
//
//beforeEach(async () => {
//cpu =  createTestDriver();
//});
//
//afterEach(async () => {
// cpu.tearDown();
//});

struct cpu_test
{
    cpu_test(const std::string &name, std::function<void()> func) : name(name), func(func) {}

    void operator()() {
        func();
    }

    std::string name;
    std::function<void()> func;
};

template<typename T>
struct marked
{
    marked() : has_value(false) {};

    marked(T t) : value(t), has_value(true) {}

    marked<T> &operator=(T &t) {
        value = t;
        has_value = true;
    }

    operator T() const { return value; }

    bool set() const { return has_value; }

    T value;
    bool has_value;
};

struct RP2040
{
    struct registers
    {
        marked<uint32_t> r0;
        marked<uint32_t> r1;
        marked<uint32_t> r2;
        marked<uint32_t> r3;
        marked<uint32_t> r4;
        marked<uint32_t> r5;
        marked<uint32_t> r6;
        marked<uint32_t> r7;
        marked<uint32_t> r8;
        marked<uint32_t> r9;
        marked<uint32_t> r10;
        marked<uint32_t> r11;
        marked<uint32_t> r12;
        marked<uint32_t> sp;
        marked<uint32_t> lr;
        marked<uint32_t> pc;
        marked<bool> N;
        marked<bool> V;
        marked<bool> C;
        marked<bool> Z;
    };

    void setPC(uint32_t pc) { rp2040.regs[ARM_REG_PC] = pc; }

    void setPRIMASK(bool primask) {
        rp2040.primask = primask;
    }

    bool getPRIMASK() {
        return rp2040.primask;
    }

    void writeUint16(uint32_t addr, uint16_t value) {
        if (addr & 1) {
            armulet_write_u8(addr, value);
            armulet_write_u8(addr+1, value>>8u);
        } else {
            armulet_write_u16(addr, value);
        }
    }

    void writeUint32(uint32_t addr, uint32_t value) {
        if (addr & 3u) {
            armulet_write_u16(addr, value);
            armulet_write_u16(addr+2, value>>16);
        } else {
            armulet_write_u32(addr, value);
        }
    }

    uint8_t readUint8(uint32_t addr) {
        return armulet_read_u8(addr);
    }

    uint32_t readUint32(uint32_t addr) {
        return armulet_read_u32(addr);
    }

    void singleStep() { armulet_single_step(&rp2040); }

    registers readRegisters() {
        printf("a %p %08x: %08x\n", &rp2040, rp2040.regs[ARM_REG_PC], rp2040.regs[ARM_REG_R5]);
        return registers {
            r0: rp2040.regs[ARM_REG_R0],
            r1: rp2040.regs[ARM_REG_R1],
            r2: rp2040.regs[ARM_REG_R2],
            r3: rp2040.regs[ARM_REG_R3],
            r4: rp2040.regs[ARM_REG_R4],
            r5: rp2040.regs[ARM_REG_R5],
            r6: rp2040.regs[ARM_REG_R6],
            r7: rp2040.regs[ARM_REG_R7],
            r8: rp2040.regs[ARM_REG_R8],
            r9: rp2040.regs[ARM_REG_R9],
            r10: rp2040.regs[ARM_REG_R10],
            r11: rp2040.regs[ARM_REG_R11],
            r12: rp2040.regs[ARM_REG_R12],
            sp: rp2040.regs[ARM_REG_SP],
            lr: rp2040.regs[ARM_REG_LR],
            pc: rp2040.regs[ARM_REG_PC],
            N: armulet_get_N(&rp2040),
            V: armulet_get_V(&rp2040),
            C: rp2040.C,
            Z: armulet_get_Z(&rp2040),
        };
    }

    void setRegisters(registers regs) {
        if (regs.r0.set()) rp2040.regs[ARM_REG_R0] = regs.r0;
        if (regs.r1.set()) rp2040.regs[ARM_REG_R1] = regs.r1;
        if (regs.r2.set()) rp2040.regs[ARM_REG_R2] = regs.r2;
        if (regs.r3.set()) rp2040.regs[ARM_REG_R3] = regs.r3;
        if (regs.r4.set()) rp2040.regs[ARM_REG_R4] = regs.r4;
        if (regs.r5.set()) rp2040.regs[ARM_REG_R5] = regs.r5;
        if (regs.r6.set()) rp2040.regs[ARM_REG_R6] = regs.r6;
        if (regs.r7.set()) rp2040.regs[ARM_REG_R7] = regs.r7;
        if (regs.r8.set()) rp2040.regs[ARM_REG_R8] = regs.r8;
        if (regs.r9.set()) rp2040.regs[ARM_REG_R9] = regs.r9;
        if (regs.r10.set()) rp2040.regs[ARM_REG_R10] = regs.r10;
        if (regs.r11.set()) rp2040.regs[ARM_REG_R11] = regs.r11;
        if (regs.r12.set()) rp2040.regs[ARM_REG_R12] = regs.r12;
        if (regs.sp.set()) rp2040.regs[ARM_REG_SP] = regs.sp;
        if (regs.lr.set()) rp2040.regs[ARM_REG_LR] = regs.lr;
        if (regs.pc.set()) rp2040.regs[ARM_REG_PC] = regs.pc;
        if (regs.V.set()) armulet_set_V(&rp2040, regs.V);
        if (regs.C.set()) rp2040.C = regs.C;
        if (regs.N.set() || regs.Z.set()) {
            bool N = regs.N.set() ? (bool)regs.N : armulet_get_N(&rp2040);
            bool Z = regs.Z.set() ? (bool)regs.Z : armulet_get_Z(&rp2040);
            armulet_set_NZ(&rp2040, N, Z);
        }
    }

    void reset() {
        armulet_reset_cpu(&rp2040);
    }

    armulet_cpu_t rp2040;
};

RP2040 cpu;

template<typename T>
struct expecter
{
    explicit expecter(T t) : t(t) {}

    T t;

    void toEqual(T other) {
        if (t != other) {
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


template<typename T>
expecter<T> expect(marked<T> t) {
    return expecter<T>(t);
}

template<typename T>
expecter<T> expect(T t) {
    return expecter<T>(t);
}

std::vector<cpu_test> tests = {
        cpu_test("should execute `adcs r5, r4` instruction", [] {

            cpu.setPC(RAM_START_ADDRESS);

            cpu.writeUint16(RAM_START_ADDRESS, opcodeADCS(r5, r4));

            cpu.setRegisters({r4: 55, r5: 66, C: true});

            cpu.singleStep();
            auto registers =
                    cpu.readRegisters();
            expect(registers.r5).toEqual(122);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute `adcs r5, r4` instruction and set negative/overflow flags", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADCS(r5, r4));
            cpu.setRegisters({
                                     r4: 0x7fffffff, // Max signed INT32
                                     r5: 0,
                                     C: true,
                             });
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0x80000000);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(true);
        }),

        cpu_test("should not set the overflow flag when executing `adcs r3, r2` adding 0 to 0 with carry", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADCS(r3, r2));
            cpu.setRegisters({r2: 0, r3: 0, C: true, Z: true});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(1);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test(
                "should set the zero, carry and overflow flag when executing `adcs r0, r0` adding 0x80000000 to 0x80000000",
                [] {
                    cpu.setPC(RAM_START_ADDRESS);
                    cpu.writeUint16(RAM_START_ADDRESS, opcodeADCS(r0, r0));
                    cpu.setRegisters({r0: 0x80000000, C: false});
                    cpu.singleStep();
                    auto registers = cpu.readRegisters();
                    expect(registers.r0).toEqual(0);
                    expect(registers.N).toEqual(false);
                    expect(registers.Z).toEqual(true);
                    expect(registers.C).toEqual(true);
                    expect(registers.V).toEqual(true);
                }),

        cpu_test("should execute a `add sp, 0x10` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({sp: 0x10000040});
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDsp2(0x10));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.sp).toEqual(0x10000050);
        }),

        cpu_test("should execute a `add r1, sp, #4` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({sp: 0x54});
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDspPlusImm(r1, 0x10));
            cpu.setRegisters({r1: 0});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.sp).toEqual(0x54);
            expect(registers.r1).toEqual(0x64);
        }),

        cpu_test("should execute `adds r1, r2, #3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDS1(r1, r2, 3));
            cpu.setRegisters({r2: 2});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(5);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute `adds r1, #1` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDS2(r1, 1));
            cpu.setRegisters({r1: 0xffffffff});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(0);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(true);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute `adds r1, r2, r7` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDSreg(r1, r2, r7));
            cpu.setRegisters({r2: 2});
            cpu.setRegisters({r7: 27});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(29);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute `adds r4, r4, r2` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDSreg(r4, r4, r2));
            cpu.setRegisters({r2: 0x74bc8000, r4: 0x43740000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r4).toEqual(0xb8308000);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(true);
        }),

        cpu_test("should execute `adds r1, r1, r1` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDSreg(r1, r1, r1));
            cpu.setRegisters({r1: 0xbf8d1424, C: true});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(0x7f1a2848);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(true);
        }),

        cpu_test("should execute `add r1, ip` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDreg(r1, ip));
            cpu.setRegisters({r1: 66, r12: 44});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(110);
            // todo doesn't really update the flags
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should not update the flags following `add r3, r12` instruction (encoding T2)", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDreg(r3, r12));
            cpu.setRegisters({r3: 0x00002000, r12: 0xffffe000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute `add sp, r8` instruction and not update the flags", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDreg(sp, r8));
            cpu.setRegisters({r8: 0x13, sp: RAM_START_ADDRESS, Z: true });
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.sp).toEqual(RAM_START_ADDRESS + 0x10);
            expect(registers.Z).toEqual(true); // assert it didn't update the flags
        }),

        cpu_test("should execute `add pc, r8` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADDreg(pc, r8));
            cpu.setRegisters({r8: 0x11});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x14);
        }),

        cpu_test("should execute `adr r4, #0x50` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeADR(r4, 0x50));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r4).toEqual(RAM_START_ADDRESS + 0x54);
        }),

        cpu_test("should execute `ands r5, r0` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeANDS(r5, r0));
            cpu.setRegisters({r5: 0xffff0000});
            cpu.setRegisters({r0: 0xf00fffff});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0xf00f0000);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
        }),

        cpu_test("should execute an `asrs r3, r2, #31` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeASRS(r3, r2, 31));
            cpu.setRegisters({r2: 0x80000000, C: true});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0xffffffff);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
        }),

        cpu_test("should execute an `asrs r3, r2, #0` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeASRS(r3, r2, 0));
            cpu.setRegisters({r2: 0x80000000, C: true});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0xffffffff);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute an `asrs r3, r4` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeASRSreg(r3, r4));
            cpu.setRegisters({r3: 0x80000040});
            cpu.setRegisters({r4: 0xff500007});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0xff000000);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute an `asrs r3, r4` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeASRSreg(r3, r4));
            cpu.setRegisters({r3: 0x40000040, r4: 50, C: true});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(true);
            expect(registers.C).toEqual(false);
        }),

        cpu_test("should execute an `asrs r3, r4` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeASRSreg(r3, r4));
            cpu.setRegisters({r3: 0x40000040, r4: 31, C: true});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(true);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute an `asrs r3, r4` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeASRSreg(r3, r4));
            cpu.setRegisters({r3: 0x80000040, r4: 50, C: true});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0xffffffff);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute an `asrs r3, r4` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeASRSreg(r3, r4));
            cpu.setRegisters({r3: 0x80000040, r4: 0, C: true});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x80000040);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute `bics r0, r3` correctly", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({r0: 0xff});
            cpu.setRegisters({r3: 0x0f});
            cpu.writeUint16(RAM_START_ADDRESS, opcodeBICS(r0, r3));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0).toEqual(0xf0);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
        }),

        cpu_test("should execute `bics r0, r3` instruction and set the negative flag correctly", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({r0: 0xffffffff});
            cpu.setRegisters({r3: 0x0000ffff});
            cpu.writeUint16(RAM_START_ADDRESS, opcodeBICS(r0, r3));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0).toEqual(0xffff0000);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
        }),

        cpu_test("should decode `bkpt 0x34` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeBKPT(0x34));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            // todo which is it?
            //expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.pc).toEqual(RAM_START_ADDRESS);
        }),

        cpu_test("should execute `bl 0x34` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeBL(0x34));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x38);
            expect(registers.lr).toEqual(RAM_START_ADDRESS + 0x5);
        }),

        cpu_test("should execute `bl -0x10` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeBL(-0x10));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x4 - 0x10);
            expect(registers.lr).toEqual(RAM_START_ADDRESS + 0x5);
        }),

        cpu_test("should execute `bl -3242` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeBL(-3242));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x4 - 3242);
            expect(registers.lr).toEqual(RAM_START_ADDRESS + 0x5);
        }),

        cpu_test("should execute `200xxa1a: f7ff fefb bl 200xx814' instruction", [] {
            cpu.setPC(RAM_START_ADDRESS + 0xa1a);
            cpu.writeUint32(RAM_START_ADDRESS + 0xa1a, 0xfefbf7ff);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x814);
            expect(registers.lr).toEqual(RAM_START_ADDRESS + 0xa1f);
        }),

        cpu_test("should execute `blx r3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({r3: RAM_START_ADDRESS + 0x201});
            cpu.writeUint32(RAM_START_ADDRESS, opcodeBLX(r3));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x200);
            expect(registers.lr).toEqual(RAM_START_ADDRESS + 0x3);
        }),

        cpu_test("should execute a `b.n .-20` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS + 9 * 2);
            cpu.writeUint16(RAM_START_ADDRESS + 9 * 2, opcodeBT2(0xfec));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `bne.n .-6` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS + 9 * 2);
            cpu.setRegisters({Z: false});
            cpu.writeUint16(RAM_START_ADDRESS + 9 * 2, opcodeBT1(1, 0x1f8));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0xe);
        }),

        cpu_test("should execute a true `blt.n .-6` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS + 9 * 2);
            cpu.setRegisters({N: false, V: true});
            cpu.writeUint16(RAM_START_ADDRESS + 9 * 2, opcodeBT1(11, 0x1f8));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0xe);
        }),

        cpu_test("should execute a false `blt.n .-6` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS + 9 * 2);
            cpu.setRegisters({N: false, V: false});
            cpu.writeUint16(RAM_START_ADDRESS + 9 * 2, opcodeBT1(11, 0x1f8));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 10 * 2);
        }),

        cpu_test("should execute a true `bge.n .-6` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS + 9 * 2);
            cpu.setRegisters({N: false, V: false});
            cpu.writeUint16(RAM_START_ADDRESS + 9 * 2, opcodeBT1(10, 0x1f8));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0xe);
        }),

        cpu_test("should execute a `200xxc14: e7e1 b.n 200xxbda'", [] {
            cpu.setPC(RAM_START_ADDRESS + 0xc14);
            cpu.setRegisters({Z: false});
            cpu.writeUint16(RAM_START_ADDRESS + 0xc14, 0xe7e1);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0xbda);
        }),

        cpu_test("should execute `bx lr` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({lr: 0x10000201});
            cpu.writeUint32(RAM_START_ADDRESS, opcodeBX(lr));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(0x10000200);
        }),

        cpu_test("should execute an `cmn r5, r2` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMN(r7, r2));
            cpu.setRegisters({r2: 1});
            cpu.setRegisters({r7: -2});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(1);
            expect(registers.r7).toEqual(-2);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute an `cmp r5, #66` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPimm(r5, 66));
            cpu.setRegisters({r5: 60});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should correctly set carry flag when executing `cmp r0, #0`", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPimm(r0, 0));
            cpu.setRegisters({r0: 0x80010133});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute an `cmp r5, r0` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT1(r5, r0));
            cpu.setRegisters({r5: 60});
            cpu.setRegisters({r0: 56});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test(
                "should execute an `cmp r2, r0` instruction and not set any flags when r0=0xb71b0000 and r2=0x00b71b00",
                [] {
                    cpu.setPC(RAM_START_ADDRESS);
                    cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT1(r2, r0));
                    cpu.setRegisters({r0: 0xb71b0000, r2: 0x00b71b00});
                    cpu.singleStep();
                    auto registers = cpu.readRegisters();
                    expect(registers.N).toEqual(false);
                    expect(registers.Z).toEqual(false);
                    expect(registers.C).toEqual(false);
                    expect(registers.V).toEqual(false);
                }),

        cpu_test("should correctly set carry flag when executing `cmp r11, r3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT2(r11, r3));
            cpu.setRegisters({r3: 0x00000008, r11: 0xffffffff});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should correctly set carry flag when executing `cmp r7, r8` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT2(r7, r8));
            // set r0 to r3, to make sure r8 is not decoded as r0
            cpu.setRegisters({r0: 0xffffffff, r7: 0xffffffff, r8: 0x00000008});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute an `cmp ip, r6` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT2(ip, r6));
            cpu.setRegisters({r6: 56, r12: 60});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should set flags N C when executing `cmp r11, r3` instruction when r3=0 and r11=0x80000000", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT2(r11, r3));
            cpu.setRegisters({r3: 0, r11: 0x80000000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should set flags N V when executing `cmp r3, r7` instruction when r3=0 and r7=0x80000000", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT1(r3, r7));
            cpu.setRegisters({r3: 0, r7: 0x80000000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(true);
        }),

        cpu_test("should set flags N V when executing `cmp r11, r3` instruction when r3=0x80000000 and r11=0", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT2(r11, r3));
            cpu.setRegisters({r3: 0x80000000, r11: 0});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(true);
        }),

        cpu_test("should set flags N C when executing `cmp r3, r7` instruction when r3=0x80000000 and r7=0", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeCMPregT1(r3, r7));
            cpu.setRegisters({r3: 0x80000000, r7: 0});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should correctly decode a `cpsid` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeCPSID());
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(cpu.getPRIMASK()).toEqual(true);
            expect(registers.pc).toBe(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should correctly decode a `cpsie` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeCPSIE());
            cpu.setPRIMASK(true);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(cpu.getPRIMASK()).toEqual(false);
            expect(registers.pc).toBe(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should correctly decode a `dmb sy` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeDMBSY());
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toBe(RAM_START_ADDRESS + 0x4);
        }),

        cpu_test("should correctly decode a `dsb sy` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeDSBSY());
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toBe(RAM_START_ADDRESS + 0x4);
        }),

        cpu_test("should execute an `eors r1, r3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeEORS(r1, r3));
            cpu.setRegisters({r1: 0xf0f0f0f0});
            cpu.setRegisters({r3: 0x08ff3007});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(0xf80fc0f7);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
        }),

        cpu_test("should correctly decode a `isb sy` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeISBSY());
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toBe(RAM_START_ADDRESS + 0x4);
        }),

        cpu_test("should execute a `mov r3, r8` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMOV(r3, r8));
            cpu.setRegisters({r8: 55});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(55);
        }),

        cpu_test("should execute a `mov r3, pc` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMOV(r3, pc));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(RAM_START_ADDRESS + 0x4);
        }),

        cpu_test("should execute a `mov sp, r8` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMOV(r3, r8));
            cpu.setRegisters({r8: 55});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(55);
        }),

        cpu_test("should execute a `muls r0, r2` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMULS(r0, r2));
            cpu.setRegisters({r0: 5});
            cpu.setRegisters({r2: 1000000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(5000000);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
        }),

        cpu_test("should execute a muls instruction with large 32-bit numbers and produce the correct result", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMULS(r0, r2));
            cpu.setRegisters({r0: 2654435769});
            cpu.setRegisters({r2: 340573321});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(1);
        }),

        cpu_test("should execute a `muls r0, r2` instruction and set the Z flag when the result is zero", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMULS(r0, r2));
            cpu.setRegisters({r0: 0});
            cpu.setRegisters({r2: 1000000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(0);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(true);
        }),

        cpu_test("should execute a `muls r0, r2` instruction and set the N flag when the result is negative", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMULS(r0, r2));
            cpu.setRegisters({r0: -1});
            cpu.setRegisters({r2: 1000000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(-1000000);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
        }),

        cpu_test("should execute a `mvns r4, r3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMVNS(r4, r3));
            cpu.setRegisters({r3: 0x11115555});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r4).toEqual(0xeeeeaaaa);
            expect(registers.Z).toBe(false);
            expect(registers.N).toBe(true);
        }),

        cpu_test("should execute a `nop` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeNOP());
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute `orrs r5, r0` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeORRS(r5, r0));
            cpu.setRegisters({r5: 0xf00f0000});
            cpu.setRegisters({r0: 0xf000ffff});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0xf00fffff);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
        }),

        cpu_test("should execute a `push {r4, r5, r6, lr}` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({sp: RAM_START_ADDRESS + 0x100});
            cpu.writeUint16(RAM_START_ADDRESS, opcodePUSH(true, (1 << r4) | (1 << r5) | (1 << r6)));
            cpu.setRegisters({r4: 0x40, r5: 0x50, r6: 0x60, lr: 0x42});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
// assert that the values of r4, r5, r6, lr were pushed into the stack
            expect(registers.sp).toEqual(RAM_START_ADDRESS + 0xf0);
            expect(cpu.readUint8(RAM_START_ADDRESS + 0xf0)).toEqual(0x40);
            expect(cpu.readUint8(RAM_START_ADDRESS + 0xf4)).toEqual(0x50);
            expect(cpu.readUint8(RAM_START_ADDRESS + 0xf8)).toEqual(0x60);
            expect(cpu.readUint8(RAM_START_ADDRESS + 0xfc)).toEqual(0x42);
        }),

        cpu_test("should execute a `pop {r2}` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({sp: RAM_START_ADDRESS + 0xf0});
            cpu.writeUint16(RAM_START_ADDRESS, opcodePOP(false, (1 << r2)));
            cpu.writeUint32(RAM_START_ADDRESS + 0xf0, 0xaa332211);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.sp).toEqual(RAM_START_ADDRESS + 0xf4);
            expect(registers.r2).toEqual(0xaa332211);
            expect(registers.pc).toEqual(RAM_START_ADDRESS+2);
        }),

        cpu_test("should execute a `pop pc, {r4, r5, r6}` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({sp: RAM_START_ADDRESS + 0xf0});
            cpu.writeUint16(RAM_START_ADDRESS, opcodePOP(true, (1 << r4) | (1 << r5) | (1 << r6)));
            cpu.writeUint32(RAM_START_ADDRESS + 0xf0, 0x40);
            cpu.writeUint32(RAM_START_ADDRESS + 0xf4, 0x50);
            cpu.writeUint32(RAM_START_ADDRESS + 0xf8, 0x60);
            cpu.writeUint32(RAM_START_ADDRESS + 0xfc, 0x43);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.sp).toEqual(RAM_START_ADDRESS + 0x100);
// assert that the values of r4, r5, r6, pc were poped from the stack correctly
            expect(registers.r4).toEqual(0x40);
            expect(registers.r5).toEqual(0x50);
            expect(registers.r6).toEqual(0x60);
            expect(registers.pc).toEqual(0x42);
        }),

        cpu_test("should execute a `mrs r0, ipsr` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeMRS(r0, 5)); // 5 == ipsr
            cpu.setRegisters({r0: 55});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0).toEqual(0);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x4);
        }),

        cpu_test("should execute a `msr msp, r9` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint32(RAM_START_ADDRESS, opcodeMSR(8, r9)); // 8 == msp
            cpu.setRegisters({r9: 0x1235});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.sp).toEqual(0x1234);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x4);
        }),

        cpu_test("should execute a `movs r5, #128` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMOVS(r5, 128));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(128);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `ldmia r0!, {r1, r2}` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDMIA(r0, (1 << r1) | (1 << r2)));
            cpu.setRegisters({r0: RAM_START_ADDRESS + 0x10});
            cpu.writeUint32(RAM_START_ADDRESS + 0x10, 0xf00df00d);
            cpu.writeUint32(RAM_START_ADDRESS + 0x14, 0x4242);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.r0).toEqual(RAM_START_ADDRESS + 0x18);
            expect(registers.r1).toEqual(0xf00df00d);
            expect(registers.r2).toEqual(0x4242);
        }),

        cpu_test("should execute a `ldmia r5!, {r5}` instruction without writing back the address to r5", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDMIA(r5, 1 << r5));
            cpu.setRegisters({r5: RAM_START_ADDRESS + 0x10});
            cpu.writeUint32(RAM_START_ADDRESS + 0x10, 0xf00df00d);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.r5).toEqual(0xf00df00d);
        }),

        cpu_test("should execute an `ldr r0, [pc, #148]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRlit(r0, 148));
            cpu.writeUint32(RAM_START_ADDRESS + 152, 0x42);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0).toEqual(0x42);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a misaligned `ldr r0, [pc, #148]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS+2);
            cpu.writeUint16(RAM_START_ADDRESS+2, opcodeLDRlit(r0, 148));
            cpu.writeUint32(RAM_START_ADDRESS + 152, 0x42);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0).toEqual(0x42);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x4);
        }),

        cpu_test("should execute an `ldr r3, [r2, #24]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRimm(r3, r2, 24));
            cpu.setRegisters({r2: RAM_START_ADDRESS});
            cpu.writeUint32(RAM_START_ADDRESS + 24, 0x55);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x55);
        }),

        cpu_test("should execute an `ldr r3, [sp, #12]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({sp: RAM_START_ADDRESS});
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRsp(r3, 12));
            cpu.writeUint32(RAM_START_ADDRESS + 12, 0x55);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x55);
        }),

        cpu_test("should execute an `ldr r3, [r5, r6]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRreg(r3, r5, r6));
            cpu.setRegisters({r5: RAM_START_ADDRESS});
            cpu.setRegisters({r6: 0x8});
            cpu.writeUint32(RAM_START_ADDRESS + 0x8, 0xff554211);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0xff554211);
        }),

        cpu_test("should execute an `ldrb r4, [r2, #5]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRB(r4, r2, 5));
            cpu.setRegisters({r2: RAM_START_ADDRESS});
            cpu.writeUint16(RAM_START_ADDRESS + 0x5, 0x7766);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r4).toEqual(0x66);
        }),

        cpu_test("should execute an `ldrb r3, [r5, r6]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRBreg(r3, r5, r6));
            cpu.setRegisters({r5: RAM_START_ADDRESS});
            cpu.setRegisters({r6: 0x8});
            cpu.writeUint32(RAM_START_ADDRESS + 0x8, 0xff554211);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x11);
        }),

        cpu_test("should execute an `ldrh r3, [r7, #4]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRH(r3, r7, 4));
            cpu.setRegisters({r7: RAM_START_ADDRESS});
            cpu.writeUint32(RAM_START_ADDRESS + 0x4, 0xffff7766);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x7766);
        }),

        cpu_test("should execute an `ldrh r3, [r7, #6]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRH(r3, r7, 6));
            cpu.setRegisters({r7: RAM_START_ADDRESS});
            cpu.writeUint32(RAM_START_ADDRESS + 0x4, 0x33447766);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x3344);
        }),

        cpu_test("should execute an `ldrh r3, [r5, r6]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRHreg(r3, r5, r6));
            cpu.setRegisters({r5: RAM_START_ADDRESS, r6: 0x8});
            cpu.writeUint32(RAM_START_ADDRESS + 0x8, 0xff554211);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x4211);
        }),

        cpu_test("should execute an `ldrsb r5, [r3, r5]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRSB(r5, r3, r5));
            cpu.setRegisters({r3: RAM_START_ADDRESS, r5: 6});
            cpu.writeUint32(RAM_START_ADDRESS + 0x6, 0x85);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0xffffff85);
        }),

        cpu_test("should execute an `ldrsh r5, [r3, r5]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLDRSH(r5, r3, r5));
            cpu.setRegisters({r3: RAM_START_ADDRESS});
            cpu.setRegisters({r5: 6});
            cpu.writeUint16(RAM_START_ADDRESS + 0x6, 0xf055);
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0xfffff055);
        }),

//cpu_test("should execute a `udf 1` instruction", [] {
//const uint32_t breakMock = jest.fn();
//const uint32_t rp2040 = new RP2040();
//rp2040.core.PC = RAM_START_ADDRESS;
//rp2040.writeUint16(RAM_START_ADDRESS, opcodeUDF(0x1));
//rp2040.onBreak = breakMock;
//rp2040.step();
//expect(rp2040.core.PC).toEqual(RAM_START_ADDRESS + 0x2);
//expect(breakMock).toHaveBeenCalledWith(1);
//},

//cpu_test("should execute a `udf.w #0` (T2 encoding) instruction", [] {
//const uint32_t breakMock = jest.fn();
//const uint32_t rp2040 = new RP2040();
//rp2040.core.PC = RAM_START_ADDRESS;
//rp2040.writeUint32(RAM_START_ADDRESS, opcodeUDF2(0));
//rp2040.onBreak = breakMock;
//rp2040.step();
//expect(rp2040.core.PC).toEqual(RAM_START_ADDRESS + 0x4);
//expect(breakMock).toHaveBeenCalledWith(0);
//},

        cpu_test("should execute a `lsls r5, r5, #18` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSLSimm(r5, r5, 18));
            cpu.setRegisters({r5: 0b00000000000000000011});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0b11000000000000000000);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(false);
        }),
        cpu_test("should execute a `lsls r5, r5, #31` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSLSimm(r5, r5, 31));
            cpu.setRegisters({r5: 0b10000000000000000000000000000011});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0b10000000000000000000000000000000);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(true);
        }),
        cpu_test("should execute a `lsls r5, r5, #31` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSLSimm(r5, r5, 31));
            cpu.setRegisters({r5: 0b10000000000000000000000000000001});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0b10000000000000000000000000000000);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(false);
        }),
        cpu_test("should execute a `lsls r5, r0` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSLSreg(r5, r0));
            cpu.setRegisters({r5: 0b00000000000000000011});
            cpu.setRegisters({r0: 0xff003302}); // bottom byte: 02
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0b00000000000000001100);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(false);
        }),

        cpu_test("should execute a lsls r3, r4 instruction when shift >31", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSLSreg(r3, r4));
            cpu.setRegisters({r3: 1, r4: 0x20, C: false});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.N).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.Z).toEqual(true);
        }),

        cpu_test("should execute a `lsls r5, r5, #18` instruction with carry", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSLSimm(r5, r5, 18));
            cpu.setRegisters({r5: 0x00004001});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0x40000);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute a `lsrs r5, r0` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSRSreg(r5, r0));
            cpu.setRegisters({r5: 0xff00000f});
            cpu.setRegisters({r0: 0xff003302});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0x3fc00003);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should return zero for `lsrs r2, r3` with 32 bit shift", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSRSreg(r2, r3));
            cpu.setRegisters({r2: 10, r3: 32});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(0);
            expect(registers.Z).toEqual(true);
            expect(registers.C).toEqual(false);
        }),

        cpu_test("should execute a `lsrs r1, r1, #1` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSRS(r1, r1, 1));
            cpu.setRegisters({r1: 0b10});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(0b1);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(false);
        }),

        cpu_test("should execute a `lsrs r1, r1, #0` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSRS(r1, r1, 0));
            cpu.setRegisters({r1: 0xffffffff});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(0);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute a `lsrs r1, r2, #31` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSRS(r1, r2, 31));
            cpu.setRegisters({r2: 0xffffffff});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(1);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(true);
        }),
        cpu_test("should execute a `lsrs r1, r1, #31` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeLSRS(r1, r1, 31));
            cpu.setRegisters({r1: 0xbfffffff});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(1);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.C).toEqual(false);
        }),

        cpu_test("should keep lower 2 bits of sp clear when executing a `movs sp, r5` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMOV(sp, r5));
            cpu.setRegisters({r5: 0x53});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.sp).toEqual(0x50);
        }),

        cpu_test("should keep lower bit of pc clear when executing a `movs pc, r5` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMOV(pc, r5));
            cpu.setRegisters({r5: 0x53});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(0x52);
        }),

        cpu_test("should execute a `movs r6, r5` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeMOVSreg(r6, r5));
            cpu.setRegisters({r5: 0x50});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r6).toEqual(0x50);
        }),

        cpu_test("should execute a `rsbs r0, r3` instruction", [] {
// This instruction is also calledasync  `negs`
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeRSBS(r0, r3));
            cpu.setRegisters({r3: 100});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0 | 0).toEqual(-100);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute a `rev r3, r1` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeREV(r2, r3));
            cpu.setRegisters({r3: 0x11223344});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(0x44332211);
        }),

        cpu_test("should execute a `rev16 r0, r5` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeREV16(r0, r5));
            cpu.setRegisters({r5: 0x11223344});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0).toEqual(0x22114433);
        }),

        cpu_test("should execute a `revsh r1, r2` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeREVSH(r1, r2));
            cpu.setRegisters({r2: 0xeeaa55f0});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(0xfffff055);
        }),

        cpu_test("should execute a `ror r5, r3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeROR(r5, r3));
            cpu.setRegisters({r5: 0x12345678});
            cpu.setRegisters({r3: 0x2004});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x2004);
            expect(registers.r5).toEqual(0x81234567);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute a `ror r5, r3` instruction when r3 > 32", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeROR(r5, r3));
            cpu.setRegisters({r5: 0x12345678});
            cpu.setRegisters({r3: 0x2044});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x2044);
            expect(registers.r5).toEqual(0x81234567);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
        }),

        cpu_test("should execute a `rsbs r0, r3` instruction", [] {
// This instruction is also calledasync  `negs`
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeRSBS(r0, r3));
            cpu.setRegisters({r3: 0});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0 | 0).toEqual(0);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(true);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute a `sbcs r0, r3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSBCS(r0, r3));
            cpu.setRegisters({r0: 100, r3: 55, C: false});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0).toEqual(44);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute a `sbcs r0, r3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSBCS(r0, r3));
            cpu.setRegisters({r0: 0, r3: 0xffffffff, C: false});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r0).toEqual(0);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(true);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute a `stmia r0!, {r1, r2}` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTMIA(r0, (1 << r1) | (1 << r2)));
            cpu.setRegisters({r0: RAM_START_ADDRESS + 0x10, r1: 0xd00fd00f, r2: 0x4143});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
            expect(registers.r0).toEqual(RAM_START_ADDRESS + 0x18);
            expect(cpu.readUint32(RAM_START_ADDRESS + 0x10)).toEqual(0xd00fd00f);
            expect(cpu.readUint32(RAM_START_ADDRESS + 0x14)).toEqual(0x4143);
        }),

        cpu_test("should execute a `str r6, [r4, #20]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTR(r6, r4, 20));
            cpu.setRegisters({r4: RAM_START_ADDRESS + 0x20, r6: 0xf00d});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(cpu.readUint32(RAM_START_ADDRESS + 0x20 + 20)).toEqual(0xf00d);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `str r6, [r4, r5]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTRreg(r6, r4, r5));
            cpu.setRegisters({r4: RAM_START_ADDRESS + 0x20});
            cpu.setRegisters({r5: 20});
            cpu.setRegisters({r6: 0xf00d});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(cpu.readUint32(RAM_START_ADDRESS + 0x20 + 20)).toEqual(0xf00d);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute an `str r3, [sp, #12]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTRsp(r3, 12));
            cpu.setRegisters({r3: 0xaa55, sp: RAM_START_ADDRESS});
            cpu.singleStep();
            expect(cpu.readUint8(RAM_START_ADDRESS + 12)).toEqual(0x55);
            expect(cpu.readUint8(RAM_START_ADDRESS + 13)).toEqual(0xaa);
        }),

        cpu_test("should execute a `str r2, [r3, r1]` instruction where r1 + r3 > 32 bits", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTRreg(r2, r1, r3));
            cpu.setRegisters({r1: -4, r2: 0x4201337, r3: 0x20041e50});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(cpu.readUint32(0x20041e4c)).toEqual(0x4201337);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `strb r6, [r4, #20]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTRB(r6, r4, 0x1));
            cpu.writeUint32(RAM_START_ADDRESS + 0x20, 0xf5f4f3f2);
            cpu.setRegisters({r4: RAM_START_ADDRESS + 0x20, r6: 0xf055});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
// assert that the 2nd byte (at 0x21) changed to 0x55
            expect(cpu.readUint32(RAM_START_ADDRESS + 0x20)).toEqual(0xf5f455f2);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `strb r6, [r4, r5]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTRBreg(r6, r4, r5));
            cpu.writeUint32(RAM_START_ADDRESS + 0x20, 0xf5f4f3f2);
            cpu.setRegisters({r4: RAM_START_ADDRESS + 0x20});
            cpu.setRegisters({r5: 1});
            cpu.setRegisters({r6: 0xf055});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
// assert that the 2nd byte (at 0x21) changed to 0x55
            expect(cpu.readUint32(RAM_START_ADDRESS + 0x20)).toEqual(0xf5f455f2);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `strh r6, [r4, #20]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTRH(r6, r4, 0x2));
            cpu.writeUint32(RAM_START_ADDRESS + 0x20, 0xf5f4f3f2);
            cpu.setRegisters({r4: RAM_START_ADDRESS + 0x20});
            cpu.setRegisters({r6: 0x6655});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
// assert that the 3rd/4th byte (at 0x22) changed to 0x6655
            expect(cpu.readUint32(RAM_START_ADDRESS + 0x20)).toEqual(0x6655f3f2);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `strh r6, [r4, r1]` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSTRHreg(r6, r4, r1));
            cpu.writeUint32(RAM_START_ADDRESS + 0x20, 0xf5f4f3f2);
            cpu.setRegisters({r4: RAM_START_ADDRESS + 0x20});
            cpu.setRegisters({r1: 2});
            cpu.setRegisters({r6: 0x6655});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
// assert that the 3rd/4th byte (at 0x22) changed to 0x6655
            expect(cpu.readUint32(RAM_START_ADDRESS + 0x20)).toEqual(0x6655f3f2);
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `sub sp, 0x10` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.setRegisters({sp: 0x10000040});
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSUBsp(0x10));
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.sp).toEqual(0x10000030);
        }),

        cpu_test("should execute a `subs r1, #1` instruction with overflow", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSUBS2(r1, 1));
            cpu.setRegisters({r1: -0x80000000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r1).toEqual(0x7fffffff);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(true);
        }),

        cpu_test("should execute a `subs r5, r3, 5` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSUBS1(r5, r3, 5));
            cpu.setRegisters({r3: 0});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5 | 0).toEqual(-5);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute a `subs r5, r3, r2` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSUBSreg(r5, r3, r2));
            cpu.setRegisters({r3: 6});
            cpu.setRegisters({r2: 5});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(1);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute a `subs r3, r3, r2` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSUBSreg(r3, r3, r2));
            cpu.setRegisters({r2: 8, r3: 0xffffffff});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0xfffffff7);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

        cpu_test("should execute a `subs r5, r3, r2` instruction and set N V flags", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSUBSreg(r5, r3, r2));
            cpu.setRegisters({r3: 0});
            cpu.setRegisters({r2: 0x80000000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0x80000000);
            expect(registers.N).toEqual(true);
            expect(registers.Z).toEqual(false);
            expect(registers.C).toEqual(false);
            expect(registers.V).toEqual(true);
        }),

        cpu_test("should execute a `subs r5, r3, r2` instruction and set Z C flags", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSUBSreg(r5, r3, r2));
            cpu.setRegisters({r2: 0x80000000});
            cpu.setRegisters({r3: 0x80000000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0);
            expect(registers.N).toEqual(false);
            expect(registers.Z).toEqual(true);
            expect(registers.C).toEqual(true);
            expect(registers.V).toEqual(false);
        }),

//cpu_test("should raise an SVCALL exception when `svc` instruction runs", [] {
//const uint32_t SVCALL_HANDLER = 0x20032000;
// cpu.setRegisters({ sp: 0x20034000 });
// cpu.setPC(0x20034000);
// cpu.writeUint16(0x20034000, opcodeSVC(10));
// cpu.setRegisters({ r0: 0x44 });
// cpu.writeUint32(VTOR, 0x20040000);
// cpu.writeUint32(0x20040000 + EXC_SVCALL * 4, SVCALL_HANDLER);
// cpu.writeUint16(SVCALL_HANDLER, opcodeMOVS(r0, 0x55));
//
// cpu.singleStep();
//if (cpu instanceof RP2040TestDriver) {
//expect(cpu.rp2040.core.pendingSVCall).toEqual(true);
//}
//
// cpu.singleStep(); // SVCall handler should run here
//auto registers2 =  cpu.readRegisters();
//if (cpu instanceof RP2040TestDriver) {
//expect(cpu.rp2040.core.pendingSVCall).toEqual(false);
//}
//expect(registers2.pc).toEqual(SVCALL_HANDLER + 2);
//expect(registers2.r0).toEqual(0x55);
//},

        cpu_test("should execute a `sxtb r2, r2` instruction with sign bit 1", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSXTB(r2, r2));
            cpu.setRegisters({r2: 0x22446688});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(0xffffff88);
        }),

        cpu_test("should execute a `sxtb r2, r2` instruction with sign bit 0", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSXTB(r2, r2));
            cpu.setRegisters({r2: 0x12345678});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(0x78);
        }),

        cpu_test("should execute a `sxth r2, r5` instruction with sign bit 1", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeSXTH(r2, r5));
            cpu.setRegisters({r5: 0x22448765});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r2).toEqual(0xffff8765);
        }),

        cpu_test("should execute an `tst r1, r3` instruction when the result is negative", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeTST(r1, r3));
            cpu.setRegisters({r1: 0xf0000000});
            cpu.setRegisters({r3: 0xf0004000});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.N).toEqual(true);
        }),

        cpu_test("should execute an `tst r1, r3` instruction when the registers are different", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeTST(r1, r3));
            cpu.setRegisters({r1: 0xf0, r3: 0x0f});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.Z).toEqual(true);
        }),

        cpu_test("should execute an `uxtb r5, r3` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeUXTB(r5, r3));
            cpu.setRegisters({r3: 0x12345678});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r5).toEqual(0x78);
        }),

        cpu_test("should execute an `uxth r3, r1` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeUXTH(r3, r1));
            cpu.setRegisters({r1: 0x12345678});
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.r3).toEqual(0x5678);
        }),

        cpu_test("should execute a `wfi` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeWFI());
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        }),

        cpu_test("should execute a `yield` instruction", [] {
            cpu.setPC(RAM_START_ADDRESS);
            cpu.writeUint16(RAM_START_ADDRESS, opcodeYIELD());
            cpu.singleStep();
            auto registers = cpu.readRegisters();
            expect(registers.pc).toEqual(RAM_START_ADDRESS + 0x2);
        })
};

uint32_t armulet_cb_special_read(uint32_t addr, int size) {
    return 0;
}

void armulet_cb_special_write(uint32_t addr, int size, uint32_t value) {
}

// 4547 cmp r7, r8
#if PICO_ON_DEVICE
#include "pico/stdlib.h"
#endif
int main(int argc, char **argv) {
#if PICO_ON_DEVICE
    stdio_init_all();
#endif
#if ARMULET_USE_ASM
    single_step_asm_hooks = varmulet_default_asm_hooks;
    install_varmulet_test_hooks(&single_step_asm_hooks);
#endif
    int i=0;
    printf("CPU at %p\n", &cpu.rp2040);
    for(auto &test : tests) {
        printf("%d / %d, %s\n", i+1, (int)tests.size(), test.name.c_str());
#if !PICO_ON_DEVICE
        static uint8_t xx;
        armulet_zap(++xx);
#endif
        cpu.reset();
        test();
        printf("  OK\n");
        i++;
    }
#ifdef __riscv
    printf("DONE on RISC-V\n");
#else
    printf("DONE\n");
#endif
}