

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

#include "multiply_saturate_examples.S"
#include "branch_and_control_examples.S"

.global Reset_Handler

/* Define vector section, to be first thing in the image */
.section .isr_vector,"a",%progbits

/* Vector table start */
.type g_pfnVectors, %object
g_pfnVectors:
    .word _estack
    .word Reset_Handler
    .word NMI_Handler
    .word HardFault_Handler
    .word MemManage_Handler
    .word BusFault_Handler
    .word UsageFault_Handler

/* Start text section */
.section .text.Reset_Handler


.type Reset_Handler, %function
Reset_Handler:

    bl multiply_examples
    bl saturate_examples

    ldr r0, =branch_example
    blx r0

    bl max_number_example
    bl max_number_example_with_it
    bl table_branch_example


    b Reset_Handler

NMI_Handler:
  bl NMI_Handler

HardFault_Handler:
  bl HardFault_Handler

MemManage_Handler:
  bl MemManage_Handler

BusFault_Handler:
  bl BusFault_Handler

UsageFault_Handler:
  bl UsageFault_Handler

