# Armulet

`ARMULET` is a C ARMv6M / ARMv8M-baseline emulator

`VARMULET` is a ARMv6M / ARMv8M-baselineARMv6M emulator
The goals of `VARMULET`:

* small code size (currently 3K) can be placed in ROM
* ? fast enough to run USB boot code under RISC-V (which will also be a non-secure ARMv6M binary under ARM).
  `VARMULET` seems to be about 3x faster than `ARMULET` on RISC-V for now for one particular use case which is printf
  heavy. We will need to test with the boot code (and also SVC calls for things like memcpy, memset)
* TODO extensible by non ROM code, with no ROM specific functionality baked in (e.g. handling of priv mode, IRQ,
  breakpoints, SVC etc). It should basically be possible to use the emulator on Amy RISC-V programs, and also to extend
  it to support other 32 bit instructions for example

