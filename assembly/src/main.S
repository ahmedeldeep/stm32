

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

#include "arithmetic_logical_examples.S"

.global Reset_Handler

/* Define vector section, to be first thing in the image */
.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object

/* Vector table start */
g_pfnVectors:
    .word _estack
    .word Reset_Handler

/* Start text section */
.section .text.Reset_Handler
.type Reset_Handler, %function


Reset_Handler:

    bl add_two_64_bits
    bl matrix_add
    bl count_ones

    b Reset_Handler




