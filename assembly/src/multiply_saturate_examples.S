
.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.data

.text

.type multiply_examples, %function
multiply_examples:

    movs r2, #0x20
    movs r3, #-0x10
    movs r4, #0x5

    /* Multiply */
    mul r0, r2, r3

    /* Signed Most Significant Word Multiply */
    smmul r0, r2, r3

    /* Multiply with accumulate, y = ax + b */
    mla r0, r2, r3, r4

    /* Unsigned Long Multiply */
    umull r0, r1, r2, r3

    /* Signed Long Multiply */
    smull r0, r1, r2, r3

    movw r5, #0x2
    movt r5, #0x3
    movw r6, #0x4
    movt r6, #0x5

    /* Signed Multiply (halfwords) */
    smulbt r0, r5, r6
    smultt r0, r5, r6
    smulbb r0, r5, r6
    smultb r0, r5, r6

    /* Signed Dual Multiply Add */
    smuad r0, r5, r6

    /* Signed Dual Multiply Add Reversed */
    smuadx r0, r5, r6

    /* Signed Dual Multiply Subtract */
    smusd r0, r5, r6

    /* Signed Dual Multiply Subtract Reversed */
    smusdx r0, r5, r6

    /* Signed Divide (result is rounded towards zero) */
    sdiv r0, r2, r3

    /* Unsigned Divide (result is rounded towards zero) */
    udiv r0, r3, r2

    bx lr

.type saturate_examples, %function
saturate_examples:

    mov r1, #0x20202020
    mov r2, #0x80808080

    /* Signed saturates */
    ssat r0, #16, r1
    ssat r0, #16, r2
    ssat16 r0, #8, r1

    /* Unigned saturates */
    usat r0, #16, r1
    usat r0, #16, r2
    usat16 r0, #8, r1

    bx lr


