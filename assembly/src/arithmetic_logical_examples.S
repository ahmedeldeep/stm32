
.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb


.data
    res: .quad 0x0000000000000000
    matrix_add_res: .byte 0x00, 0x00, 0x00, 0x00
                    .byte 0x00, 0x00, 0x00, 0x00
                    .byte 0x00, 0x00, 0x00, 0x00
                    .byte 0x00, 0x00, 0x00, 0x00
.text
    num_01: .quad 0xAA55AA66FFFFFFFF
    num_02: .quad 0xAA77AA77FFFFFFFF

/* Double word addition */
add_two_64_bits:

    ldrd r0, r1, num_01 /* load double low registers */
    ldrd r2, r3, num_02 /* load double high registers */

    adds r4, r0, r2     /* add low 32bits registers */
    adc r5, r1, r3      /* add high 32bits registers */

    ldr r6, =res        /* get ram address to store the result */
    strd r4, r5, [r6]   /* store 64bits result */

    bx lr               /* return */

matrix_1: .byte 0x11, 0x12, 0x13, 0x14
          .byte 0x21, 0x22, 0x23, 0x24
          .byte 0x31, 0x32, 0x33, 0x34
          .byte 0x41, 0x42, 0x43, 0x44

matrix_2: .byte 0x51, 0x52, 0x53, 0x54
          .byte 0x61, 0x62, 0x63, 0x64
          .byte 0x71, 0x72, 0x73, 0x74
          .byte 0x81, 0x82, 0x83, 0x84

/* Add two 4x4 matrices */
matrix_add:

    movs r0, #16            /* loop index */
    adr r1, matrix_1        /* load matrix 1 address */
    adr r2, matrix_2        /* load matrix 2 address */
    ldr r6, =matrix_add_res /* where to store the results */

matrix_add_end:
    cbnz r0, matrix_add_loop /* compare and branch if non zero */
    bx lr                    /* return */

matrix_add_loop:

    sub r0, #4               /* decrement the index by 4 */
    ldr r3, [r1, r0]         /* get one raw from matrix_1 */
    ldr r4, [r2, r0]         /* get one raw from matrix_2 */

    uadd8 r5, r3, r4         /* unsigned add 4 bytes in paraller (SIMD) */
    rev r5, r5               /* reverse byte order */

    str r5, [r6, r0]         /* store the results */
    b matrix_add_end         /* loop */

count_ones_num: .word 0xAA55AA10

/* Count number of ones inside 32bits value,
   useful for parity calculatio */
count_ones:

    ldr r0, count_ones_num         /* load address of the 32bits value */
    movs r1, #32                   /* number of rotations */
    movs r2, #0                    /* reset r2 */

count_ones_start:
    cbnz r1, count_ones_loop       /* compare and branch if non zero */
    and r3, r2, #1                 /* and with 1 to check even or odd  */
    bx lr                          /* return */

count_ones_loop:
    rors r0, r0, #1                /* rotate right */
    sub r1, #1                     /* decrement the index */
    adc r2, #0                     /* add r2 + carry flag */
    b count_ones_start             /* loop */




