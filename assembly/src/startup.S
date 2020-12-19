

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb


.global Reset_Handler

/* Define vector section, to be first thing in the image */
.section .isr_vector,"a",%progbits

/* Vector table start */
.type g_pfnVectors, %object
g_pfnVectors:
    .word  _estack
    .word  Reset_Handler
    .word  NMI_Handler
    .word  HardFault_Handler
    .word  MemManage_Handler
    .word  BusFault_Handler
    .word  UsageFault_Handler
    .word  0
    .word  0
    .word  0
    .word  0
    .word  SVC_Handler
    .word  DebugMon_Handler
    .word  0
    .word  PendSV_Handler
    .word  SysTick_Handler

/* Start text section */
.section .text.Reset_Handler
.global matrix_add_asm

.type Reset_Handler, %function
Reset_Handler:

    mov r0, 0x10
    mov r1, 0x20
    mov r2, 0x30
    mov r3, 0x40
    mov r12, 0x50

    /* Supervisor Call */
    svc #0xAB

    /* Copy the data segment initializers from flash to SRAM */
    movs  r1, #0
    b  LoopCopyDataInit

CopyDataInit:
    ldr  r3, =_sidata
    ldr  r3, [r3, r1]
    str  r3, [r0, r1]
    adds  r1, r1, #4

LoopCopyDataInit:
    ldr  r0, =_sdata
    ldr  r3, =_edata
    adds  r2, r0, r1
    cmp  r2, r3
    bcc  CopyDataInit
    ldr  r2, =_sbss
    b  LoopFillZerobss

/* Zero fill the bss segment. */
FillZerobss:
    movs  r3, #0
    str  r3, [r2], #4

LoopFillZerobss:
    ldr  r3, = _ebss
    cmp  r2, r3
    bcc  FillZerobss

    bl main

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

.type SVC_Handler, %function
SVC_Handler:

    /* Check EXC_RETURN in link register,
    The lowest five bits of this value provide information
    on the return stack and processor mode.

    10001 Handler mode, non-floating-point state from MSP
    11001 Thread mode, non-floating-point state from MSP
    11101 Thread mode, non-floating-point state from PSP

    00001 Handler mode, floating-point state from MSP
    01001 Thread mode, floating-point state from MSP
    01101 Thread mode, floating-point state from PSP
    bits[31:5] set to one */

    /* Check the used stack */
    tst lr, #0x4    /* lr AND 4, set zero flag Z also EQ flag */

    ite eq
    mrseq r0, msp /* Current is main stack */
    mrsne r0, psp /* Current is process stack */

    /* The PC is stacked 6 word away from sp,
    Check Exception stack frame
    R0        @SP
    R1        @SP+4
    R2        @SP+8
    R3        @SP+12
    R12       @SP+16
    LR        @SP+20
    PC        @SP+24
    xPSR      @SP+28
    {aligner}
     */

    /* Read stacked PC, address of the instruction after svc */
    ldr r1, [r0, #24]

    /* Read SVC imm */
    ldrb r0, [r1, #-2]

    bx lr

DebugMon_Handler:
  bl DebugMon_Handler

PendSV_Handler:
  bl PendSV_Handler

SysTick_Handler:
  bl SysTick_Handler


