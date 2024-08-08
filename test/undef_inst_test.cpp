#include <string>
#include <vector>

std::vector<std::string> undefined = {
        // 16 bit ==================================================

        // Shift (immediate), add, subtract, move, and compare -----
        // 00xxxx

        // Data processing -----------------------------------------
        // 010000

        // "010000 0100 " // unpredictable

        // Special data instructions and branch and exchange -------
        // 010001

        // LDR (literal) -------------------------------------------
        // 01001x

        // Load/store single data item -----------------------------
        // 0101xx
        // 011xxx
        // 100xxx

        // ADR -----------------------------------------------------
        // 10100x

        // ADD (SP plus immediate) ---------------------------------
        // 10101x

        // Miscellaneous 16-bit instructions -----------------------
        // 1011 xx
        //      00000xx
        //      00001xx
        "  1011 0001xxx",
        //      001000x
        //      001001x
        //      001010x
        //      001011x
        "  1011 0011xxx",
        //      010xxxx
        "  1011 011000x",
        "  1011 0110010",
        //      0110011
        "  1011 01101xx",
        "  1011 0111xxx",
        "  1011 100xxxx",
        //      101000x
        // 1011 101001x
        "  1011 101010x",
        //      101011x
        "  1011 1011xxx",
        //      110xxxx
        //      1110xxx
        //      1111xxx

        // STM, STMIA, STMEA ---------------------------------------
        // 11000x

        // LDM, LDMIA, LDMFD ---------------------------------------
        // 11001x

        // Conditional branch, and Supervisor Call -----------------
        // 1101 xx

        "1101 1110", // UDF

        // Unconditional branch ------------------------------------
        // 1100x

        // 32 bit ==================================================
        // 11101
        // 11110
        // 11111
};