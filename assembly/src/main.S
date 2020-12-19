

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

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

    bl move_examples
    bl memory_access_examples

    b Reset_Handler




