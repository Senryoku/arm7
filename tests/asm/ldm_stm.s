	.global _start 
_start:
	mov r0, #0x2000
	mov r1, #1
	mov r2, #2
	mov r3, #3
	mov r4, #4
	mov r5, #5
	mov r6, #6
	mov r7, #7
	mov r8, #8
	mov r9, #9
	mov r10, #10
	mov r11, #11
	mov r12, #12
	mov sp, #13
	mov lr, #14
	stmdb r0!, {r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,sp,lr}
	mov r0, #0x4000
	ldmdb r0, {r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,sp,lr}
	mov r0, #0x2000
	sub r0, #56
	ldmia r0, {r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,sp,lr}
