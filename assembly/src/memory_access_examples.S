
.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global memory_access_examples

.text

MyNumber_01:
    .byte 0x10, 0x20, 0x30, 0x40, 0xF0, 0xE0, 0xD0, 0xC0

memory_access_examples:

    /* Generate PC-relative address */
    adr r0, MyNumber_01
    ldr r0, =MyNumber_01
    adr r1, MyNumber_02

    /* LDR and STR, immediate offset */
    /* op{type}{cond} Rt, [Rn {, #offset}] */
    ldrb r2, [r0, #4]
    ldrsb r2, [r0, #4]

    /* LDR and STR, pre-indexed */
    /* op{type}{cond} Rt, [Rn, #offset]! */
    ldr r3, =var_01
    ldr r4, =_sidata

    ldrh r5, [r4, #2]!
    strh r5, [r3, #2]!

    ldrh r5, [r4, #2]!
    strh r5, [r3, #2]!

    /* LDR and STR, post-indexed */
    /* op{type}{cond} Rt, [Rn], #offset */
    ldr r3, =var_01
    ldr r4, =_sidata

    ldrh r5, [r4], #2
    strh r5, [r3], #2

    ldrh r5, [r4], #2
    strh r5, [r3], #2

    /* LDM and STM, Load and Store Multiple registers */
    /* op{addr_mode}{cond} Rn{!}, reglist */
    ldr r3, =var_01
    ldr r4, =_sidata

    ldm r4!, {r5, r6}
    stmia r3!, {r5, r6}

    mov r3, sp
    ldr r4, =_sidata

    stmdb r3!, {r5, r6}

    push {r5, r6}
    push {r5, r6}
    pop {r5, r6}


    bx lr

MyNumber_02:
    .byte 0x50, 0x60, 0x70, 0x80
    .word 0xA0B0C0D0

.data
var_01:
    .hword 0x0123, 0x4567, 0x89AB, 0xCDEF




