add_executable(instruction_test instruction_test.cpp
        $<$<BOOL:${PICO_RISCV}>:varmulet_hooks_test.S>
        )
target_link_libraries(instruction_test armulet pico_stdlib)
message("PICO_RISCV ${PICO_RISCV}")
target_compile_definitions(instruction_test PRIVATE
        ARMULET_DEBUG=1
        ARMULET_FEATURE_STEP=1
        ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM=1
        ARMULET_FEATURE_ARMV8M_BASELINE=1
        $<$<BOOL:${PICO_RISCV}>:ARMULET_USE_ASM=1>
        )
pico_add_extra_outputs(instruction_test)

add_executable(instruction_test2
        instruction_test2.cpp
        $<$<BOOL:${PICO_RISCV}>:varmulet_hooks_test.S>
        )
target_link_libraries(instruction_test2 armulet pico_stdlib)
target_compile_definitions(instruction_test2 PRIVATE
        ARMULET_DEBUG=1
        ARMULET_FEATURE_STEP=1
        ARMULET_FEATURE_STEP_STATUS=1
        ARMULET_FEATURE_ARMV8M_BASELINE_MSPLIM=1
        ARMULET_FEATURE_ARMV8M_BASELINE=1
        VARMULET_USE_EARLY_INSTR_READ=0 # get two extra tests this way - todo revisit if we test access faults
        $<$<BOOL:${PICO_RISCV}>:ARMULET_USE_ASM=1>
        EARLY_READ=0
        )
pico_add_extra_outputs(instruction_test2)

add_executable(instruction_test_nf instruction_test.cpp)
target_link_libraries(instruction_test_nf armulet pico_stdlib)
if (PICO_RISCV)
#        pico_set_binary_type(instruction_test no_flash)
#        pico_set_binary_type(instruction_test2 no_flash)
else()
        target_compile_definitions(instruction_test_nf PRIVATE ARMULET_DEBUG=1)
        pico_add_extra_outputs(instruction_test_nf)
        pico_set_binary_type(instruction_test_nf no_flash)
endif()
