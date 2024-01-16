	.global _start 

_start:
	mov r0, #1
	mov r0, #0
	mov r1, #1
	mov r1, #0
	mov r0, #0x20000
	mov r1, #0xC0
	str r1, [r0]
	add r1, #1
	strb r1, [r0]

	strb r1, [r0, #1]
	strb r1, [r0, #2]
	strb r1, [r0, #3]
