#include <string.h>
#include <stdio.h>
#include "armulet.h"
#include "armulet_accessors.h"

#if __clang__
#pragma clang diagnostic ignored "-Wgnu-binary-literal"
#endif

#if !ARMULET_USE_REAL_MEMORY
uint8_t rom_memory[A_ROM_END - A_ROM_START];
uint8_t ram_memory[A_RAM_END - A_RAM_START];
uint8_t flash_memory[A_FLASH_END - A_FLASH_START];
#endif

#if ARMULET_TRACE_PRINTF
#define trace_printf printf
#else
#define trace_printf(fmt, ...) ((void)0)
#endif
#define debug_printf printf

static void warn_unsupported(const char *msg) {
#if ARMULET_DEBUG
    debug_printf("Skipping %s\n", msg);
#else
    panic(msg);
#endif
}

#if ARMULET_DEBUG

static void unsupported_instruction(armulet_cpu_t *cpu, uint32_t op16) {
#if ARMULET_FEATURE_STEP_STATUS
    cpu->step_status = ARMULET_IST_UNDEFINED16;
    cpu->regs[ARM_REG_PC] -= 2;
#else
    panic("Unknown instruction at address %08x: %04x\n", armulet_get_pc(cpu), op16);
#endif
}

static void unsupported_instruction32(armulet_cpu_t *cpu, uint32_t op16, uint32_t op16b) {
#if ARMULET_FEATURE_STEP_STATUS
    cpu->step_status = ARMULET_IST_UNDEFINED32;
    cpu->regs[ARM_REG_PC] -= 2;
#else
    panic("Unknown instruction at address %08x: %04x%04x\n", armulet_get_pc(cpu), op16, op16b);
#endif
}

#else
#define unsupported_instruction(cpu, op16) __breakpoint()
#define unsupported_instruction32(cpu, op16, op16b) __breakpoint()
#endif

static uint32_t add_update_flags(armulet_cpu_t *cpu, uint32_t a, uint32_t b) {
    uint32_t result = a + b;
    cpu->C = result < a;
#if ARMULET_USE_LAZY_Z
    cpu->lazy_v0 = result;
    cpu->lazy_v1 = a;
    cpu->lazy_v2 = b;
#else
    cpu->_V = (((int32_t) (result ^ b)) < 0) && (((int32_t) (a ^ b)) >= 0);
#endif
    armulet_update_nz(cpu, result);
    return result;
}

static uint32_t adc_update_flags(armulet_cpu_t *cpu, uint32_t a, uint32_t b, bool c) {
    uint32_t bc = b + c;
    uint32_t result = a + bc;
    cpu->C = result < a || (c && !bc);
#if ARMULET_USE_LAZY_Z
    cpu->lazy_v0 = result;
    cpu->lazy_v1 = a;
    cpu->lazy_v2 = b;
#else
    cpu->_V = (((int32_t) (result ^ b)) < 0) && (((int32_t) (a ^ b)) >= 0);
#endif
    armulet_update_nz(cpu, result);
    return result;
}

static uint32_t sub_update_flags(armulet_cpu_t *cpu, uint32_t a, uint32_t b) {
    uint32_t result = a - b;
    cpu->C = a >= b;
#if ARMULET_USE_LAZY_Z
    cpu->lazy_v0 = a;
    cpu->lazy_v1 = result;
    cpu->lazy_v2 = b;
#else
    cpu->_V = (((int32_t) (a ^ b)) < 0) && (((int32_t) (result ^ b)) >= 0);
#endif
    armulet_update_nz(cpu, result);
    return result;
}

static void check_exec_return(armulet_cpu_t *cpu, uint32_t new_pc) {
    if (0xf == new_pc >> 28u) {
#if ARMULET_FEATURE_CALL
        if (cpu->armulet_call && new_pc == ARMULET_CALL_RETURN_ADDRESS) {
            longjmp(cpu->jmpbuf, 1);
        }
#endif
        panic("Exception Return");
    }
}

static void do_shift(armulet_cpu_t *cpu, uint32_t type, uint32_t rd, uint32_t rm, uint32_t shift) {
    if (type == 0b000) {
        // lsls
        if (shift <= 32) {
            if (shift) cpu->C = (cpu->regs[rm] >> (32u - shift)) & 1u;
            cpu->regs[rd] = (shift < 32) ? cpu->regs[rm] << shift : 0;
        } else {
            cpu->regs[rd] = 0;
            cpu->C = 0;
        }
    } else if (type == 0b001) {
        // lsrs
        if (shift <= 32) {
            if (shift) cpu->C = (cpu->regs[rm] >> (shift - 1)) & 1u;
            cpu->regs[rd] = (shift < 32) ? cpu->regs[rm] >> shift : 0;
        } else {
            cpu->regs[rd] = 0;
            cpu->C = 0;
        }
    } else {
        static_assert((int32_t) (-1) >> 1 == (int32_t) (-1), "");
        // asrs
        if (shift < 32) {
            if (shift) cpu->C = (cpu->regs[rm] >> (shift - 1)) & 1u;
            cpu->regs[rd] = (uint32_t) (((int32_t) cpu->regs[rm]) >> shift);
        } else {
            cpu->C = ((int32_t) cpu->regs[rm]) < 0;
            cpu->regs[rd] = (uint32_t) (((int32_t) cpu->regs[rm]) >> 31);
        }
    }
    armulet_update_nz(cpu, cpu->regs[rd]);
}

static void execute_00(armulet_cpu_t *cpu, uint32_t op16) {
    uint32_t prefix = (op16 >> 11u) & 0x7u;
    switch (prefix) {
        case 0b000:
        case 0b001:
        case 0b010: {
            // lsls, lsrs, asrs
            uint32_t rd = op16 & 0x7u;
            uint32_t rm = (op16 >> 3u) & 0x7u;
            uint32_t shift = (op16 >> 6u) & 0x1fu;
            if (prefix && !shift) shift = 32;
            do_shift(cpu, prefix, rd, rm, shift);
            break;
        }
        case 0b011: {
            uint32_t rn = (op16 >> 3u) & 0x7u;
            uint32_t rd = op16 & 0x7u;
            uint32_t value = op16 & 0x400u ? (op16 >> 6u) & 0x7u : cpu->regs[(op16 >> 6u) & 0x7u];
            if (op16 & 0x200) {
                // subs
                cpu->regs[rd] = sub_update_flags(cpu, cpu->regs[rn], value);
            } else {
                // adds
                cpu->regs[rd] = add_update_flags(cpu, cpu->regs[rn], value);
            }
            break;
        }
        case 0b100: {
            // movs
            uint32_t rd = (op16 >> 8u) & 0x7u;
            cpu->regs[rd] = op16 & 0xffu;
            armulet_update_nz(cpu, cpu->regs[rd]);
            break;
        }
        case 0b101: {
            // cmp;
            uint32_t rn = (op16 >> 8u) & 0x7u;
            sub_update_flags(cpu, cpu->regs[rn], op16 & 0xffu);
            break;
        }
        case 0b110:
        case 0b111: {
            // adds, subs
            uint32_t rdn = (op16 >> 8u) & 0x7u;
            uint32_t imm8 = op16 & 0xffu;
            if (prefix & 1) {
                cpu->regs[rdn] = sub_update_flags(cpu, cpu->regs[rdn], imm8);
            } else {
                cpu->regs[rdn] = add_update_flags(cpu, cpu->regs[rdn], imm8);
            }
            break;
        }
    }
}

static void __noinline check_update_sp_pc(armulet_cpu_t *cpu, uint32_t reg) {
    if (reg >= ARM_REG_SP) {
        if (reg == ARM_REG_SP) {
            cpu->regs[ARM_REG_SP] &= ~3u;
        } else if (reg == ARM_REG_PC) {
            cpu->regs[ARM_REG_PC] &= ~1u;
            cpu->pc_delta = 0;
        }
    }
}

static uint32_t get_lo_hi_reg(armulet_cpu_t *cpu, uint32_t reg) {
    return cpu->regs[reg] + (reg == ARM_REG_PC ? 4 : 0);
}

static void execute_0100(armulet_cpu_t *cpu, uint32_t op16) {
    switch ((op16 >> 10u) & 0x3u) {
        case 0b00: {
            uint32_t rdn = op16 & 0x7u;
            uint32_t rm = (op16 >> 3u) & 0x7u;

            uint32_t prefix = (op16 >> 6u) & 0xfu;
            switch (prefix) {
                case 0b0000:
                    // ands
                    cpu->regs[rdn] &= cpu->regs[rm];
                    break;
                case 0b0001:
                    // eors
                    cpu->regs[rdn] ^= cpu->regs[rm];
                    break;
                case 0b0010:
                case 0b0011:
                case 0b0100: {
                    // lsls, lsrs, asrs
                    uint32_t shift = cpu->regs[rm] & 0xffu;
                    do_shift(cpu, prefix - 0b0010, rdn, rdn, shift);
                    return;
                }
                case 0b0101: {
                    // adcs
                    cpu->regs[rdn] = adc_update_flags(cpu, cpu->regs[rdn], cpu->regs[rm], cpu->C);
                    return;
                }
                case 0b0110: {
                    // sbcs
                    cpu->regs[rdn] = adc_update_flags(cpu, cpu->regs[rdn], ~cpu->regs[rm], cpu->C);
                    return;
                }
                case 0b0111: {
                    // lion goes...
                    uint32_t shift = cpu->regs[rm] & 0xff;
                    if (shift) {
                        shift &= 0x1f;
                        cpu->regs[rdn] = (cpu->regs[rdn] >> shift) | (cpu->regs[rdn] << (32 - shift));
                        cpu->C = cpu->regs[rdn] >> 31u;
                    }
                    break;
                }
                case 0b1000: {
                    // tsts
                    armulet_update_nz(cpu, cpu->regs[rdn] & cpu->regs[rm]);
                    return;
                }
                case 0b1001: {
                    // rsbs, #0
                    cpu->regs[rdn] = sub_update_flags(cpu, 0, cpu->regs[rm]);
                    break;
                }
                case 0b1010: {
                    sub_update_flags(cpu, cpu->regs[rdn], cpu->regs[rm]);
                    return;
                }
                case 0b1011:
                    // cmn
                    add_update_flags(cpu, cpu->regs[rdn], cpu->regs[rm]);
                    return;
                case 0b1100:
                    // orrs
                    cpu->regs[rdn] |= cpu->regs[rm];
                    break;
                case 0b1101:
                    // muls
                    cpu->regs[rdn] *= cpu->regs[rm];
                    break;
                case 0b1110:
                    // bics
                    cpu->regs[rdn] &= ~cpu->regs[rm];
                    break;
                case 0b1111:
                    // mvns
                    cpu->regs[rdn] = ~cpu->regs[rm];
                    break;
                default:
                    unsupported_instruction(cpu, op16);
            }
            armulet_update_nz(cpu, cpu->regs[rdn]);
            break;
        }
        case 0b01: {
            uint32_t prefix = (op16 >> 6u) & 0xfu;
            switch (prefix >> 2) {
                case 0b00: {
                    // add
                    uint32_t rdn = (op16 & 0x7u) + ((op16 >> 4u & 0x8u));
                    uint32_t rm = (op16 >> 3u) & 0xfu;
                    uint32_t vdn = get_lo_hi_reg(cpu, rdn);
                    uint32_t vm = get_lo_hi_reg(cpu, rm);
                    cpu->regs[rdn] = vdn + vm;
                    check_update_sp_pc(cpu, rdn);
                    break;
                }
                case 0b01: {
                    uint32_t rn = ((op16 >> 4u) & 0x8u) | (op16 & 0x7u);
                    uint32_t rm = (op16 >> 3) & 0xfu;
                    uint32_t vn = get_lo_hi_reg(cpu, rn);
                    uint32_t vm = get_lo_hi_reg(cpu, rm);
                    sub_update_flags(cpu, vn, vm);
                    break;
                }
                case 0b10: {
                    // mov
                    uint32_t rd = ((op16 >> 4u) & 0x8u) | (op16 & 0x7u);
                    uint32_t rm = (op16 >> 3) & 0xfu;
                    cpu->regs[rd] = get_lo_hi_reg(cpu, rm);
                    check_update_sp_pc(cpu, rd);
                    break;
                }
                case 0b11: {
                    // bx / blx
                    uint32_t rm = (op16 >> 3u) & 0xfu;
                    uint32_t next_pc = get_lo_hi_reg(cpu, rm);
                    if (prefix & 2) {
                        cpu->regs[ARM_REG_LR] = cpu->regs[ARM_REG_PC] + 3;
                    } else {
                        check_exec_return(cpu, next_pc);
                    }
                    armulet_update_pc(cpu, next_pc);
                    break;
                }
                default:
                    unsupported_instruction(cpu, op16);
            }
            break;
        }
        case 0b10:
        case 0b11: {
            // ldr (literal)
            uint32_t rt = (op16 >> 8u) & 0x7u;
            uint32_t imm8 = (op16 & 0xffu);
            uint32_t addr = ((cpu->regs[ARM_REG_PC] >> 2u) + 1u + imm8) << 2u;
            cpu->regs[rt] = armulet_read_u32(addr);
            break;
        }
    }
}

static void execute_0110_1000(armulet_cpu_t *cpu, uint32_t op16) {
    uint32_t imm5 = (op16 >> 6u) & 0x1fu;
    uint32_t rn = (op16 >> 3u) & 0x7u;
    uint32_t rt = op16 & 0x7u;
    bool load = op16 & 0x800u;
    switch (op16 >> 12) {
        case 0b0110: {
            // ldr/str imm5
            uint32_t addr = cpu->regs[rn] + (imm5 << 2u);
            if (load) {
                cpu->regs[rt] = armulet_read_u32(addr);
            } else {
                armulet_write_u32(addr, cpu->regs[rt]);
            }
            break;
        }
        case 0b0111: {
            // ldrb/strb imm5
            uint32_t addr = cpu->regs[rn] + imm5;
            if (load) {
                cpu->regs[rt] = armulet_read_u8(addr);
            } else {
                armulet_write_u8(addr, (uint8_t) cpu->regs[rt]);
            }
            break;
        }
        case 0b1000: {
            // ldrh/strh imm5
            uint32_t addr = cpu->regs[rn] + (imm5 << 1u);
            if (load) {
                cpu->regs[rt] = armulet_read_u16(addr);
            } else {
                armulet_write_u16(addr, (uint16_t) cpu->regs[rt]);
            }
            break;
        }
    }
}

static void execute_1001(armulet_cpu_t *cpu, uint32_t op16) {
    uint32_t rt = (op16 >> 8u) & 0x7u;
    uint32_t imm8 = (op16 & 0xffu);
    uint32_t addr = cpu->regs[ARM_REG_SP] + (imm8 << 2u);
    if (op16 & 0x800u) {
        // ldr sp
        cpu->regs[rt] = armulet_read_u32(addr);
    } else {
        // str sp
        armulet_write_u32(addr, cpu->regs[rt]);
    }
}

static void execute_1010(armulet_cpu_t *cpu, uint32_t op16) {
    uint32_t rd = (op16 >> 8u) & 0x7u;
    uint32_t imm8 = op16 & 0xffu;
    if (op16 & 0x800u) {
        cpu->regs[rd] = cpu->regs[ARM_REG_SP] + (imm8 << 2u);
    } else {
        cpu->regs[rd] = ((cpu->regs[ARM_REG_PC] + 4) & ~3u) + (imm8 << 2u);
    }
}

static void reflect_primask(const armulet_cpu_t *cpu) {
#if PICO_ON_DEVICE && !ARMULET_DEBUG
#if __riscv
    ((void)cpu);
    needs_love_riscv_skip();
#else
    if (cpu->primask) __asm volatile ("cpsid i");
    else              __asm volatile ("cpsie i");
#endif
#endif
}

static void execute_1011(armulet_cpu_t *cpu, uint32_t op16) {
    uint32_t prefix = (op16 >> 8u) & 0xfu;
    switch (prefix) {
        case 0b0000: {
            uint32_t imm7 = (op16 & 0x7fu) << 2;
            if (op16 & 0x80u)
                cpu->regs[ARM_REG_SP] -= imm7;
            else
                cpu->regs[ARM_REG_SP] += imm7;
            break;
        }
        case 0b0010: {
            uint32_t rm = (op16 >> 3u) & 0x7u;
            uint32_t rd = op16 & 0x7u;
            switch ((op16 >> 6u) & 0x3u) {
                case 0b00: // sxth
                    cpu->regs[rd] = (uint32_t) (int32_t) (int16_t) cpu->regs[rm];
                    break;
                case 0b01: // sxtb
                    cpu->regs[rd] = (uint32_t) (int32_t) (int8_t) cpu->regs[rm];
                    break;
                case 0b10: // uxth
                    cpu->regs[rd] = (uint32_t) (uint16_t) cpu->regs[rm];
                    break;
                case 0b11: // uxtb
                    cpu->regs[rd] = (uint32_t) (uint8_t) cpu->regs[rm];
                    break;
            }
            break;
        }
        case 0b0100:
        case 0b0101: {
            // push
            // pre-decrement ARM_REG_SP as a thought towards interrupts (if they share the same stack)
            cpu->regs[ARM_REG_SP] -= (uint32_t) __builtin_popcount(op16 & 0x1ffu) << 2u;
            __compiler_memory_barrier();
            uint32_t addr = cpu->regs[ARM_REG_SP];
            for (uint i = 0; i < 8; i++) {
                if (op16 & (1u << i)) {
                    armulet_write_u32(addr, cpu->regs[i]);
                    addr += 4;
                }
            }
            if (prefix & 1) {
                armulet_write_u32(addr, cpu->regs[ARM_REG_LR]);
            }
            break;
        }
        case 0b0110: {
            if ((op16 >> 5u) == 0b10110110011 && 2 == (op16 & 15)) {
                // cps
                cpu->primask = (bool) (op16 & 0x10u);
                reflect_primask(cpu);
            } else {
                unsupported_instruction(cpu, op16);
            }
            break;
        }
        case 0b1010: {
            uint32_t rm = (op16 >> 3u) & 0x7u;
            uint32_t rd = op16 & 0x7u;
            uint32_t src = cpu->regs[rm];
            uint32_t result;
            switch ((op16 >> 6u) & 0x3u) {
                case 0b00: // rev
                    result = __builtin_bswap32(src);
                    break;
                case 0b01: // rev16
                    result = (uint32_t) __builtin_bswap16((uint16_t) src) |
                             ((uint32_t) __builtin_bswap16((uint16_t) (src >> 16)) << 16u);
                    break;
                case 0b10:
                    unsupported_instruction(cpu, op16);
                    return;
                case 0b11: // revsh
                    result = (uint32_t) (int32_t) (int16_t) __builtin_bswap16((uint16_t) src);
                    break;
            }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
            cpu->regs[rd] = result;
#pragma GCC diagnostic pop
            break;
        }
        case 0b1100:
        case 0b1101: {
            // pop
            uint32_t addr = cpu->regs[ARM_REG_SP];
            for (uint i = 0; i < 8; i++) {
                if (op16 & (1u << i)) {
                    cpu->regs[i] = armulet_read_u32(addr);
                    addr += 4;
                }
            }
            if (prefix & 1) {
                uint32_t next_pc = armulet_read_u32(addr);
                check_exec_return(cpu, next_pc);
                armulet_update_pc(cpu, next_pc);
                addr += 4;
            }
            __compiler_memory_barrier();
            cpu->regs[ARM_REG_SP] = addr;
            break;
        }
        case 0b1110: {
#if ARMULET_FEATURE_STEP_STATUS
            cpu->step_status = ARMULET_IST_BREAKPOINT;
#endif
#if ARMULET_DEBUG
            puts("ignoring BKPT");
#else
            // bkpt
            __breakpoint();
#endif
            cpu->pc_delta = 0;
            break;
        }
        case 0b1111: {
            // hint instructions
            if (op16 & 0xf) {
                unsupported_instruction(cpu, op16);
            } else {
                uint32_t opA = (op16 >> 4) & 0xf;
                switch (opA) {
                    case 0b0010:
#if PICO_ON_DEVICE
                        //                        __wfe();
#endif
                        break;
                    case 0b0100:
#if PICO_ON_DEVICE
                        //                        __sev();
#endif
                        break;
                    default:
                        warn_unsupported("wfi etc");
                }
            }
            break;
        }
        default:
            unsupported_instruction(cpu, op16);
    }
}

static void execute_11(armulet_cpu_t *cpu, uint32_t op16) {
    uint32_t prefix = (op16 >> 11u) & 0x7u;
    switch (prefix) {
        case 0b000: {
            // stmia
            uint32_t rn = (op16 >> 8u) & 0x7u;
            uint32_t addr = cpu->regs[rn];
            for (uint i = 0; i < 8; i++) {
                if (op16 & (1u << i)) {
                    armulet_write_u32(addr, cpu->regs[i]);
                    addr += 4;
                }
            }
            cpu->regs[rn] = addr;
            break;
        }
        case 0b001: {
            // ldmia
            uint32_t rn = (op16 >> 8u) & 0x7u;
            uint32_t addr = cpu->regs[rn];
            for (uint i = 0; i < 8; i++) {
                if (op16 & (1u << i)) {
                    cpu->regs[i] = armulet_read_u32(addr);
                    addr += 4;
                }
            }
            if (!(op16 & (1u << rn))) cpu->regs[rn] = addr;
            break;
        }
        case 0b010:
        case 0b011: {
            // b condition
            uint32_t opcode = (op16 >> 8u) & 0xfu;
            bool take_branch;
            switch (opcode >> 1) {
                case 0b000: // eq / ne
                    take_branch = armulet_get_Z(cpu);
                    break;
                case 0b001: // cs / cc
                    take_branch = cpu->C;
                    break;
                case 0b010: // mi / pl
                    take_branch = armulet_get_N(cpu);
                    break;
                case 0b011: // vs / vc
                    take_branch = armulet_get_V(cpu);
                    break;
                case 0b100: // hi / ls
                    take_branch = cpu->C && !armulet_get_Z(cpu);
                    break;
                case 0b101: // ge / lt
                    take_branch = armulet_get_N(cpu) == armulet_get_V(cpu);
                    break;
                case 0b110: // gt / le
                    take_branch = !armulet_get_Z(cpu) && armulet_get_N(cpu) == armulet_get_V(cpu);
                    break;
                case 0b111:
                    if (opcode == 0b1111) {
                        // svc
#if ARMULET_FEATURE_STEP_STATUS
                        cpu->step_status = ARMULET_IST_SVC;
#else
                        unsupported_instruction(cpu, op16);
#endif
                    } else {
                        // udf
                        unsupported_instruction(cpu, op16);
                    }
                    return;
            }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
            if (opcode & 1) take_branch = !take_branch;
            if (take_branch) {
                uint32_t delta = (uint32_t) ((((int32_t) op16) << 24u) >> 23u);
                cpu->pc_delta = 4 + delta;
            }
#pragma GCC diagnostic pop
            break;
        }
        case 0b100: {
            // b
            uint32_t delta = (uint32_t) ((((int32_t) op16) << 21u) >> 20u);
            cpu->pc_delta = 4 + delta;
            break;
        }
        case 0b110: {
            uint32_t op16_2 = armulet_read_u16(cpu->regs[ARM_REG_PC] + 2);
            if (op16_2 & 0x8000u) {
                uint32_t op2 = (op16_2 >> 12u) & 0x7u;
                if ((op2 & 0x5u) == 0x5u) {
                    // bl
                    uint32_t s = (op16 >> 10u) & 1u;
                    uint32_t j1 = (op16_2 >> 13u) & 1u;
                    uint32_t j2 = (op16_2 >> 11u) & 1u;
                    uint32_t i1 = (j1 ^ s) ? 0 : 2;
                    uint32_t i2 = (j2 ^ s) ? 0 : 1;
                    uint32_t offset = (uint32_t) (((int32_t) ((s * 4u | i1 | i2) << 29u)) >> 7u);
                    offset |= (op16 & 0x3ffu) << 12;
                    offset |= (op16_2 & 0x7ffu) << 1;
                    cpu->regs[ARM_REG_LR] = cpu->regs[ARM_REG_PC] + 5;
                    cpu->pc_delta = 4 + offset;
                } else {
                    uint32_t op1 = (op16 >> 4u) & 0x7fu;
                    if (op1 == 0b0111011) {
                        // todo dmb etc.
                        cpu->pc_delta = 4;
                    } else if (op1 == 0b0111110) {
                        // mrs
                        uint32_t SYSm = op16_2 & 0xffu;
                        uint32_t rd = (op16_2 >> 8u) & 0xfu;
                        uint32_t value = 0;
                        switch (SYSm >> 3) {
                            case 0:
                                if (SYSm & 1) value = cpu->ipsr;
                                if (SYSm < 4) {
                                    if (armulet_get_N(cpu)) value |= 0x80000000;
                                    if (armulet_get_Z(cpu)) value |= 0x40000000;
                                    if (cpu->C) value |= 0x20000000;
                                    if (armulet_get_V(cpu)) value |= 0x10000000;
                                }
                                break;
                            case 1:
                                value = cpu->regs[ARM_REG_SP]; // todo correct sp
                                break;
                            case 2:
                                if (!(SYSm & 7u)) {
                                    value = cpu->primask; // todo unpriveleged can't read
                                } else {
                                    // warn_unsupported("mrs ctrl");
                                    // todo need
                                }
                                break;
                            default:
                                unsupported_instruction32(cpu, op16, op16_2);
                                break;
                        }
                        cpu->regs[rd] = value;
                        cpu->pc_delta = 4;
//                        check_update_sp_pc(cpu, rd); // unpredictable
                    } else if (op1 == 0b0111000) {
                        // msr
                        uint32_t SYSm = op16_2 & 0xffu;
                        uint32_t rd = op16 & 0xfu;
                        uint32_t val = get_lo_hi_reg(cpu, rd);
                        switch (SYSm >> 3) {
                            case 0:
                                if (SYSm < 4) {
                                    bool n = val & 0x80000000;
                                    bool z = val & 0x40000000;
                                    if (n && z) n = false;
                                    armulet_set_NZ(cpu, n, z);
                                    cpu->C = val & 0x20000000;
                                    armulet_set_V(cpu, val & 0x10000000);
                                }
                                break;
                            case 1:
//                                warn_unsupported("msr sp");
                                // todo MARM_REG_SP/PARM_REG_SP
                                cpu->regs[ARM_REG_SP] = val & ~3u; // todo correct ARM_REG_SP
                                break;
                            case 2:
                                if (!(SYSm & 7u)) {
                                    cpu->primask = val & 1;
                                    reflect_primask(cpu);
                                } else {
                                    warn_unsupported("msr ctrl");
                                }
                                break;
                            default:
                                unsupported_instruction32(cpu, op16, op16_2);
                        }
                        cpu->pc_delta = 4;
                    } else {
                        unsupported_instruction32(cpu, op16, op16_2);
                    }
                }
            } else {
                unsupported_instruction(cpu, op16);
            }
            break;
        }
        default:
            unsupported_instruction(cpu, op16);
    }
}

void execute_0101(armulet_cpu_t *cpu, uint32_t op16) {
    uint32_t opb = (op16 >> 9u) & 0x7u;
    uint32_t rm = (op16 >> 6u) & 0x7u;
    uint32_t rn = (op16 >> 3u) & 0x7u;
    uint32_t rt = op16 & 0x7u;
    uint32_t addr = cpu->regs[rn] + cpu->regs[rm];
    switch (opb) {
        case 0b0000:
            // str
            armulet_write_u32(addr, cpu->regs[rt]);
            break;
        case 0b0001:
            // strh
            armulet_write_u16(addr, (uint16_t) cpu->regs[rt]);
            break;
        case 0b0010:
            // str
            armulet_write_u8(addr, (uint8_t) cpu->regs[rt]);
            break;
        case 0b0011:
            // ldrsb
            cpu->regs[rt] = (uint32_t) (int32_t) (int8_t) armulet_read_u8(addr);
            break;
        case 0b0100:
            // ldr
            cpu->regs[rt] = armulet_read_u32(addr);
            break;
        case 0b0101:
            // ldrh
            cpu->regs[rt] = armulet_read_u16(addr);
            break;
        case 0b0110:
            // ldrb
            cpu->regs[rt] = armulet_read_u8(addr);
            break;
        case 0b0111:
            // ldrsh
            cpu->regs[rt] = (uint32_t) (int32_t) (int16_t) armulet_read_u16(addr);
            break;
        default:
            unsupported_instruction(cpu, op16);
    }
}

/*

armulet_read_u16 address_reg, out_reg:
    bge \address_reg, water_mark_reg armulet_read_u16_special
    lh \out_reg, \address_reg

finish_instruction:
    add r_pc, r_pc_delta
step:
    armulet_read_u16 r_pc, r_inst
    li r_pc_delta, 2
    srli r_tmp0, r_inst, 12
    sh2add r_tmp1, r_main_decode, r_tmp0
    jr r_tmp1

*/

void carmulet_single_step(armulet_cpu_t *cpu) {
    uint32_t pc = armulet_get_pc(cpu);
            hard_assert(armulet_is_valid_pc(pc));
    uint32_t op16 = armulet_read_u16(pc);
    uint32_t prefix = (op16 >> 12u) & 0xfu;
    cpu->pc_delta = 2;
    switch (prefix) {
        case 0b0000:
        case 0b0001:
        case 0b0010:
        case 0b0011:
            execute_00(cpu, op16);
            break;
        case 0b0100:
            execute_0100(cpu, op16);
            break;
        case 0b0101:
            execute_0101(cpu, op16);
            break;
        case 0b0110:
        case 0b0111:
        case 0b1000:
            execute_0110_1000(cpu, op16);
            break;
        case 0b1001:
            execute_1001(cpu, op16);
            break;
        case 0b1010:
            execute_1010(cpu, op16);
            break;
        case 0b1011:
            execute_1011(cpu, op16);
            break;
        case 0b1100:
        case 0b1101:
        case 0b1110:
        case 0b1111:
            execute_11(cpu, op16);
            break;
    }
    cpu->regs[ARM_REG_PC] += cpu->pc_delta;
}

void __attribute__((noreturn)) carmulet_run(armulet_cpu_t *cpu) {
    while (true) {
        carmulet_single_step(cpu);
    }
}

void carmulet_jump(armulet_cpu_t *cpu, uint32_t addr) {
#if ARMULET_FEATURE_CALL
    cpu->armulet_call = false;
#endif
    armulet_update_pc(cpu, addr);
    carmulet_run(cpu);
}

#if ARMULET_FEATURE_CALL
uint32_t carmulet_call(armulet_cpu_t *cpu, uint32_t addr) {
    if (setjmp(cpu->jmpbuf)) {
        return cpu->regs[R0];
    }
    cpu->armulet_call = true;
    cpu->regs[LR] = ARMULET_CALL_RETURN_ADDRESS;
    armulet_update_pc(cpu, addr);
    run(cpu);
}
#endif


