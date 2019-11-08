
.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global move_examples


.text

.equ some_const, 0x20
.equ hword_01, 0x2010
.equ hword_02, 0x4030

move_examples:


    mov r0, #0x12         /* encoded in 32-bit */
    mov r0, #0x1234       /* changed to movw because 16-bit constant */

    movs r0, #0xFA        /* encoded in 16-bit */
    movs r0, #0xFA00
    movs r0, #0x00FA00FA

    mov r1, #0xFFFFFFF0   /* changed to mvn */
    mov r1, #0x7FFFFFFF   /* changed to mvn */
    mov r1, #0xFFFFFFFF   /* constant considered -1 */
    mov r2, #-1           /* same as previous */

    mvn r3, #0x0

    movs r1, #'A'         /* ascii */
    movs r2, #some_const  /* constant definition */

    mov r5, #0x0          /* Don’t update APSR */
    movs r5, #0x0         /* Update APSR */
    mov r6, pc           /* read current pc */

    movs.w r6, #1
    movs r7, r6, lsl #4   /* Register with shift */

    movw r8, #hword_01    /* move top */
    movt r8, #hword_02

    mrs r9, faultmask     /* read current faultmask */
    movs r10, #1
    msr primask, r10      /* disable interrupts */

    bx lr

