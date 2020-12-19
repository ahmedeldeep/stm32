
.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.data

.text
    numbers: .byte 0x30, 0x10, 0x60, 0x80, 0x40, 0x90, 0x20, 0x00

.type max_number_example, %function
max_number_example:

//  maximum_num = num[0];
//
//  for (idx = 1; idx < size; idx++)
//  {
//    if (num[idx] > maximum_num)
//    {
//       maximum_num  = num[idx];
//       maximum_location = idx;
//    }
//  }

    ldrb r0, numbers    /* maximum_num = num[0]; */
    mov r1, #0          /* maximum_location = 0; */
    mov r2, #0          /* Loop index */
    adr r3, numbers     /* array address */

loop:
    ldrb r4, [r3, r2]   /* Get number from array */
    cmp r4, r0          /* if (num[idx] > maximum_num) */
    bgt new_max         /* new max found */

continue:
    add r2, #1          /* Increment index */
    cmp r2, #8          /* Check index */
    beq end             /* All array numbers are checked */
    bne loop

new_max:
    mov r0, r4          /* maximum_num  = num[idx]; */
    mov r1, r2          /* maximum_location = idx; */
    b continue

end:
    bx lr               /* return */

.type branch_example, %function
branch_example:
    push {lr}
    bl nested_branch
    pop {pc}            /* pop lr + bx lr */

nested_branch:
    movs r0, #0
    movs r1, #0
    movs r2, #0
    movs r3, #0
    bx lr

.type max_number_example_with_it, %function
max_number_example_with_it:

    ldrb r0, numbers    /* maximum_num = num[0]; */
    mov r1, #0          /* maximum_location = 0; */
    mov r2, #0          /* Loop index */
    adr r3, numbers     /* array address */

loop_2:
    ldrb r4, [r3, r2]   /* Get number from array */
    cmp r4, r0          /* if (num[idx] > maximum_num) */
    itt gt
    movgt r0, r4        /* maximum_num  = num[idx]; */
    movgt r1, r2        /* maximum_location = idx; */
    add r2, #1          /* Increment index */
    cmp r2, #8          /* Check index */
    it ne
    bne loop_2
    bx lr               /* return */

table_branch_example:
    adr r0, table_branch_byte
    mov r1, #0

switch:
    tbb [r0, r1]       /* switch (r1) */

cont:
    add r1, #1
    cmp r1, #4
    bne switch
    bx lr

case0:
    mov r2, #0x10
    b cont

case1:
    mov r2, #0x20
    b cont

case2:
    mov r2, #0x30
    b cont

case3:
    mov r2, #0x40
    b cont

table_branch_byte:
    .byte ((case0 - cont) / 2)
    .byte ((case1 - cont) / 2)
    .byte ((case2 - cont) / 2)
    .byte ((case3 - cont) / 2)



