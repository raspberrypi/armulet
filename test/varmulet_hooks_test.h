#pragma once

#include "varmulet.h"

#ifdef __cplusplus
extern "C" {
#endif

#if ARMULET_USE_ASM
void install_varmulet_test_hooks(struct varmulet_asm_hooks *hooks);
#endif

#ifdef __cplusplus
}
#endif
