#include <stdint.h>
#include <stdbool.h>

static inline uint32_t opcodeADCS(uint32_t Rdn, uint32_t Rm) {
  return (0b0100000101 << 6) | ((Rm & 7) << 3) | (Rdn & 7);
}

static inline uint32_t opcodeADDS1(uint32_t Rd, uint32_t Rn, uint32_t imm3) {
  return (0b0001110 << 9) | ((imm3 & 0x7) << 6) | ((Rn & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeADDS2(uint32_t Rdn, uint32_t imm8) {
  return (0b00110 << 11) | ((Rdn & 7) << 8) | (imm8 & 0xff);
}

static inline uint32_t opcodeADDspPlusImm(uint32_t Rd, uint32_t imm8) {
  return (0b10101 << 11) | ((Rd & 7) << 8) | ((imm8 >> 2) & 0xff);
}

static inline uint32_t opcodeADDsp2(uint32_t imm) {
  return (0b101100000 << 7) | ((imm >> 2) & 0x7f);
}

static inline uint32_t opcodeADDSreg(uint32_t Rd, uint32_t Rn, uint32_t Rm) {
  return (0b0001100 << 9) | ((Rm & 0x7) << 6) | ((Rn & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeADDreg(uint32_t Rdn, uint32_t Rm) {
  return (0b01000100 << 8) | ((Rdn & 0x8) << 4) | ((Rm & 0xf) << 3) | (Rdn & 0x7);
}

static inline uint32_t opcodeADR(uint32_t Rd, uint32_t imm8) {
  return (0b10100 << 11) | ((Rd & 7) << 8) | ((imm8 >> 2) & 0xff);
}

static inline uint32_t opcodeANDS(uint32_t Rn, uint32_t Rm) {
  return (0b0100000000 << 6) | ((Rm & 7) << 3) | (Rn & 0x7);
}

static inline uint32_t opcodeASRS(uint32_t Rd, uint32_t Rm, uint32_t imm5) {
  return (0b00010 << 11) | ((imm5 & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rd & 0x7);
}

static inline uint32_t opcodeASRSreg(uint32_t Rdn, uint32_t Rm) {
  return (0b0100000100 << 6) | ((Rm & 0x7) << 3) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}

static inline uint32_t opcodeBT1(uint32_t cond, uint32_t imm8) {
  return (0b1101 << 12) | ((cond & 0xf) << 8) | ((imm8 >> 1) & 0x1ff);
}

static inline uint32_t opcodeBT2(uint32_t imm11) {
  return (0b11100 << 11) | ((imm11 >> 1) & 0x7ff);
}

static inline uint32_t opcodeBICS(uint32_t Rdn, uint32_t Rm) {
  return (0b0100001110 << 6) | ((Rm & 7) << 3) | (Rdn & 7);
}

static inline uint32_t opcodeBKPT(uint32_t imm) {
    const uint32_t num = imm & 0xff;
    const uint32_t opcode = (0b10111110 << 8) | num;
    return opcode;
}

static inline uint32_t opcodeBL(int32_t imm) {
  const uint32_t imm11 = (imm >> 1) & 0x7ff;
  const uint32_t imm10 = (imm >> 12) & 0x3ff;
  const uint32_t s = imm < 0 ? 1 : 0;
  const uint32_t j2 = 1 - (((imm >> 22) & 0x1) ^ s);
  const uint32_t j1 = 1 - (((imm >> 23) & 0x1) ^ s);
  const uint32_t opcode =
    (0b1101 << 28) | (j1 << 29) | (j2 << 27) | (imm11 << 16) | (0b11110 << 11) | (s << 10) | imm10;
  return opcode;
}

static inline uint32_t opcodeBLX(uint32_t Rm) {
  return (0b010001111 << 7) | (Rm << 3);
}

static inline uint32_t opcodeBX(uint32_t Rm) {
  return (0b010001110 << 7) | (Rm << 3);
}

static inline uint32_t opcodeCMN(uint32_t Rn, uint32_t Rm) {
  return (0b0100001011 << 6) | ((Rm & 0x7) << 3) | (Rn & 0x7);
}

static inline uint32_t opcodeCMPimm(uint32_t Rn, uint32_t Imm8) {
  return (0b00101 << 11) | ((Rn & 0x7) << 8) | (Imm8 & 0xff);
}

static inline uint32_t opcodeCMPregT1(uint32_t Rn, uint32_t Rm) {
  return (0b0100001010 << 6) | ((Rm & 0x7) << 3) | (Rn & 0x7);
}

static inline uint32_t opcodeCMPregT2(uint32_t Rn, uint32_t Rm) {
  return (0b01000101 << 8) | (((Rn >> 3) & 0x1) << 7) | ((Rm & 0xf) << 3) | (Rn & 0x7);
}

static inline uint32_t opcodeCPSID() {
    return 0b1011011001110010;
}

static inline uint32_t opcodeCPSIE() {
    return 0b1011011001100010;
}

static inline uint32_t opcodeDMBSY() {
  return 0x8f50f3bf;
}

static inline uint32_t opcodeDSBSY() {
  return 0x8f4ff3bf;
}

static inline uint32_t opcodeEORS(uint32_t Rdn, uint32_t Rm) {
  return (0b0100000001 << 6) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}

static inline uint32_t opcodeISBSY() {
  return 0x8f6ff3bf;
}

static inline uint32_t opcodeLDMIA(uint32_t Rn, uint32_t registers) {
  return (0b11001 << 11) | ((Rn & 0x7) << 8) | (registers & 0xff);
}

static inline uint32_t opcodeLDRreg(uint32_t Rt, uint32_t Rn, uint32_t Rm) {
  return (0b0101100 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeLDRimm(uint32_t Rt, uint32_t Rn, uint32_t imm5) {
  return (0b01101 << 11) | (((imm5 >> 2) & 0x1f) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeLDRlit(uint32_t Rt, uint32_t imm8) {
  return (0b01001 << 11) | ((imm8 >> 2) & 0xff) | ((Rt & 0x7) << 8);
}

static inline uint32_t opcodeLDRB(uint32_t Rt, uint32_t Rn, uint32_t imm5) {
  return (0b01111 << 11) | ((imm5 & 0x1f) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeLDRsp(uint32_t Rt, uint32_t imm8) {
  return (0b10011 << 11) | ((Rt & 7) << 8) | ((imm8 >> 2) & 0xff);
}

static inline uint32_t opcodeLDRBreg(uint32_t Rt, uint32_t Rn, uint32_t Rm) {
  return (0b0101110 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeLDRH(uint32_t Rt, uint32_t Rn, uint32_t imm5) {
  return (0b10001 << 11) | (((imm5 >> 1) & 0xf) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeLDRHreg(uint32_t Rt, uint32_t Rn, uint32_t Rm) {
  return (0b0101101 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeLDRSB(uint32_t Rt, uint32_t Rn, uint32_t Rm) {
  return (0b0101011 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeLDRSH(uint32_t Rt, uint32_t Rn, uint32_t Rm) {
  return (0b0101111 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeLSLSreg(uint32_t Rdn, uint32_t Rm) {
  return (0b0100000010 << 6) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}

static inline uint32_t opcodeLSLSimm(uint32_t Rd, uint32_t Rm, uint32_t Imm5) {
  return (0b00000 << 11) | ((Imm5 & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rd & 0x7);
}

static inline uint32_t opcodeLSRS(uint32_t Rd, uint32_t Rm, uint32_t imm5) {
  return (0b00001 << 11) | ((imm5 & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rd & 0x7);
}

static inline uint32_t opcodeLSRSreg(uint32_t Rdn, uint32_t Rm) {
  return (0b0100000011 << 6) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}

static inline uint32_t opcodeMOV(uint32_t Rd, uint32_t Rm) {
  return (0b01000110 << 8) | ((Rd & 0x8 ? 1 : 0) << 7) | (Rm << 3) | (Rd & 0x7);
}

static inline uint32_t opcodeMOVS(uint32_t Rd, uint32_t imm8) {
  return (0b00100 << 11) | ((Rd & 0x7) << 8) | (imm8 & 0xff);
}

static inline uint32_t opcodeMOVSreg(uint32_t Rd, uint32_t Rm) {
  return (0b000000000 << 6) | ((Rm & 0x7) << 3) | (Rd & 0x7);
}
static inline uint32_t opcodeMRS(uint32_t Rd, uint32_t specReg) {
  return (
    ((0b1000 << 28) | ((Rd & 0xf) << 24) | ((specReg & 0xff) << 16) | 0b1111001111101111)
  );
}

static inline uint32_t opcodeMSR(uint32_t specReg, uint32_t Rn) {
  return ((0b10001000 << 24) | ((specReg & 0xff) << 16) | (0b111100111000 << 4) | (Rn & 0xf));
}

static inline uint32_t opcodeMULS(uint32_t Rn, uint32_t Rdm) {
  return (0b0100001101 << 6) | ((Rn & 7) << 3) | (Rdm & 7);
}

static inline uint32_t opcodeMVNS(uint32_t Rd, uint32_t Rm) {
  return (0b0100001111 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeNOP() {
  return 0b1011111100000000;
}

static inline uint32_t opcodeORRS(uint32_t Rn, uint32_t Rm) {
  return (0b0100001100 << 6) | ((Rm & 0x7) << 3) | (Rn & 0x7);
}

static inline uint32_t opcodePOP(bool P, uint32_t registerList) {
  return (0b1011110 << 9) | ((P ? 1 : 0) << 8) | registerList;
}

static inline uint32_t opcodePUSH(bool M, uint32_t registerList) {
  return (0b1011010 << 9) | ((M ? 1 : 0) << 8) | registerList;
}

static inline uint32_t opcodeREV(uint32_t Rd, uint32_t Rn) {
  return (0b1011101000 << 6) | ((Rn & 0x7) << 3) | (Rd & 0x7);
}

static inline uint32_t opcodeREV16(uint32_t Rd, uint32_t Rn) {
  return (0b1011101001 << 6) | ((Rn & 0x7) << 3) | (Rd & 0x7);
}

static inline uint32_t opcodeREVSH(uint32_t Rd, uint32_t Rn) {
  return (0b1011101011 << 6) | ((Rn & 0x7) << 3) | (Rd & 0x7);
}

static inline uint32_t opcodeROR(uint32_t Rdn, uint32_t Rm) {
  return (0b0100000111 << 6) | ((Rm & 0x7) << 3) | (Rdn & 0x7);
}

static inline uint32_t opcodeRSBS(uint32_t Rd, uint32_t Rn) {
  return (0b0100001001 << 6) | ((Rn & 0x7) << 3) | (Rd & 0x7);
}

static inline uint32_t opcodeSBCS(uint32_t Rn, uint32_t Rm) {
  return (0b0100000110 << 6) | ((Rm & 0x7) << 3) | (Rn & 0x7);
}

static inline uint32_t opcodeSTMIA(uint32_t Rn, uint32_t registers) {
  return (0b11000 << 11) | ((Rn & 0x7) << 8) | (registers & 0xff);
}

static inline uint32_t opcodeSTR(uint32_t Rt, uint32_t Rm, uint32_t imm5) {
  return (0b01100 << 11) | (((imm5 >> 2) & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeSTRsp(uint32_t Rt, uint32_t imm8) {
  return (0b10010 << 11) | ((Rt & 7) << 8) | ((imm8 >> 2) & 0xff);
}

static inline uint32_t opcodeSTRreg(uint32_t Rt, uint32_t Rn, uint32_t Rm) {
  return (0b0101000 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeSTRB(uint32_t Rt, uint32_t Rm, uint32_t imm5) {
  return (0b01110 << 11) | ((imm5 & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeSTRBreg(uint32_t Rt, uint32_t Rn, uint32_t Rm) {
  return (0b0101010 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeSTRH(uint32_t Rt, uint32_t Rm, uint32_t imm5) {
  return (0b10000 << 11) | (((imm5 >> 1) & 0x1f) << 6) | ((Rm & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeSTRHreg(uint32_t Rt, uint32_t Rn, uint32_t Rm) {
  return (0b0101001 << 9) | ((Rm & 0x7) << 6) | ((Rn & 0x7) << 3) | (Rt & 0x7);
}

static inline uint32_t opcodeSUBS1(uint32_t Rd, uint32_t Rn, uint32_t imm3) {
  return (0b0001111 << 9) | ((imm3 & 0x7) << 6) | ((Rn & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeSUBS2(uint32_t Rdn, uint32_t imm8) {
  return (0b00111 << 11) | ((Rdn & 7) << 8) | (imm8 & 0xff);
}

static inline uint32_t opcodeSUBSreg(uint32_t Rd, uint32_t Rn, uint32_t Rm) {
  return (0b0001101 << 9) | ((Rm & 0x7) << 6) | ((Rn & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeSUBsp(uint32_t imm) {
  return (0b101100001 << 7) | ((imm >> 2) & 0x7f);
}

static inline uint32_t opcodeSVC(uint32_t imm8) {
  return (0b11011111 << 8) | (imm8 & 0xff);
}

static inline uint32_t opcodeSXTB(uint32_t Rd, uint32_t Rm) {
  return (0b1011001001 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeSXTH(uint32_t Rd, uint32_t Rm) {
  return (0b1011001000 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeTST(uint32_t Rm, uint32_t Rn) {
  return (0b0100001000 << 6) | ((Rn & 7) << 3) | (Rm & 7);
}

static inline uint32_t opcodeUXTB(uint32_t Rd, uint32_t Rm) {
  return (0b1011001011 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeUDF(uint32_t imm8) {
  return ((0b11011110 << 8) | (imm8 & 0xff));
}

static inline uint32_t opcodeUDF2(uint32_t imm16) {
  const uint32_t imm12 = imm16 & 0xfff;
  const uint32_t imm4 = (imm16 >> 12) & 0xf;
  return ((0b111101111111 << 4) | imm4 | (0b1010 << 28) | (imm12 << 16));
}

static inline uint32_t opcodeUXTH(uint32_t Rd, uint32_t Rm) {
  return (0b1011001010 << 6) | ((Rm & 7) << 3) | (Rd & 7);
}

static inline uint32_t opcodeWFI() {
  return 0b1011111100110000;
}

static inline uint32_t opcodeYIELD() {
  return 0b1011111100010000;
}
