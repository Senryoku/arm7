# Part of arm7wrestler_dc by snickerbockers https://github.com/snickerbockers/dc-arm7wrestler/
# Adpated (WIP) to run on just the arm7, not the full system.

.equ BAD_Rd,	0x10
.equ BAD_Rn,	0x20

_start:
	stmfd sp!,{lr}
	
	ldr r0,=szALU1
	mov r1,#56
	mov r2,#1
	mov r3,#4
	bl DrawText
	

	@ ADC
	mov 	r1,#0
	mov 	r2,#0x80000000
	mov 	r3,#0xF
	adds 	r9,r9,r9	@ clear carry
	adcs 	r2,r2,r3
	orrcs 	r1,r1,#1
	orrpl 	r1,r1,#2
	orrvs 	r1,r1,#4
	orreq 	r1,r1,#8
	adcs 	r2,r2,r2	
	orrcc 	r1,r1,#1
	orrmi 	r1,r1,#2
	adc 	r3,r3,r3
	cmp 	r3,#0x1F
	orrne 	r1,r1,#BAD_Rd
	
	adds 	r9,r9,r9	@ clear carry
	mov 	r0,#0
	mov 	r2,#1
	adc 	r0,r0,r2,lsr#1
	cmp 	r0,#1
	@orrne 	r1,r1,#BAD_Rd
	
	ldr 	r0,=szADC
	bl 	DrawResult
	add 	r8,r8,#8

	@ ADD
	mov 	r1,#0
	ldr 	r2,=0xFFFFFFFE
	mov 	r3,#1
	adds 	r2,r2,r3
	orrcs 	r1,r1,#1
	orrpl 	r1,r1,#2
	orrvs 	r1,r1,#4
	orreq 	r1,r1,#8
	adds 	r2,r2,r3	
	orrcc 	r1,r1,#1
	orrmi 	r1,r1,#2
	orrvs 	r1,r1,#4

	orrne 	r1,r1,#8
	ldr 	r0,=szADD
	bl 	DrawResult
	add 	r8,r8,#8

	

	@ AND
	mov 	r1,#0
	mov 	r2,#2
	mov 	r3,#5
	ands 	r2,r2,r3,lsr#1
	orrcc 	r1,r1,#1
	orreq 	r1,r1,#8
	cmp 	r2,#2
	orrne 	r1,r1,#BAD_Rd
	mov 	r2,#0xC00
	mov 	r3,r2

	mov 	r4,#0x80000000
	ands 	r2,r2,r4,asr#32
	orrcc 	r1,r1,#1
	orrmi 	r1,r1,#2
	orreq 	r1,r1,#8
	cmp 	r2,r3
	orrne 	r1,r1,#BAD_Rd
	ldr 	r0,=szAND
	bl 	DrawResult
	add 	r8,r8,#8


	@ BIC
	mov 	r1,#0
	adds 	r9,r9,r9 @ clear carry
	ldr 	r2,=0xFFFFFFFF
	ldr 	r3,=0xC000000D
	bics 	r2,r2,r3,asr#1
	orrcc 	r1,r1,#1
	orrmi 	r1,r1,#2	
	orreq 	r1,r1,#8
	ldr 	r3,=0x1FFFFFF9
	cmp 	r2,r3
	orrne 	r1,r1,#16
	ldr 	r0,=szBIC
	bl 	DrawResult
	add 	r8,r8,#8
	
	@ CMN
	mov 	r1,#0
	adds 	r9,r9,r9 @ clear carry
	ldr 	r2,=0x7FFFFFFF
	ldr 	r3,=0x70000000
	cmns 	r2,r3
	orrcs 	r1,r1,#1
	orrpl 	r1,r1,#2
	orrvc 	r1,r1,#4
	orreq 	r1,r1,#8
	ldr 	r3,=0x7FFFFFFF
	cmp 	r2,r3
	orrne 	r1,r1,#16
	ldr 	r0,=szCMN
	bl 	DrawResult
	add 	r8,r8,#8

	
	@ EOR
	mov 	r1,#0
	mov 	r2,#1
	mov 	r3,#3
	eors 	r2,r2,r3,lsl#31
	eors 	r2,r2,r3,lsl#0
	orrcc 	r1,r1,#1
	orrpl 	r1,r1,#2
	orreq 	r1,r1,#8
	ldr 	r4,=0x80000002
	cmp 	r4,r2
	orrne 	r1,r1,#BAD_Rd
	ldr 	r0,=szEOR
	bl 	DrawResult
	add 	r8,r8,#8

	
	@ MOV
	mov 	r1,#0
	ldr 	r2,=labelone
	mov 	r3,r15
	cmp 	r2,r3
labelone:
	orrne 	r1,r1,#BAD_Rd
	ldr 	r2,=labeltwo
	mov 	r3,#0

	@ XXX IDK if this is due to being on a different CPU or not,
	@     but GNU AS says using R15 leads to unpredictable behavior
	movs 	r4,r15,lsl r3	@ 0
	orreq 	r1,r1,#8	@ 4
	cmp 	r4,r2		@ 8
labeltwo: 			@ 12	
	orrne 	r1,r1,#BAD_Rd
	ldr 	r2,=0x80000001
	movs 	r3,r2,lsr#32
	orrcc 	r1,r1,#1
	orrmi 	r1,r1,#2
	
	orrne 	r1,r1,#8
	cmp 	r3,#0
	orrne 	r1,r1,#BAD_Rd

	@ Test ASR by reg==0
	mov 	r3,#3
	movs 	r4,r3,lsr#1	@ set carry 	
	mov 	r2,#0
	movs 	r3,r4,asr r2
	orrcc 	r1,r1,#1
	cmp 	r3,#1
	orrne 	r1,r1,#16

	@ Test ASR by reg==33
	ldr 	r2,=0x80000000
	mov 	r3,#33
	movs 	r2,r2,asr r3
	orrcc 	r1,r1,#1
	ldr 	r4,=0xFFFFFFFF
	cmp 	r2,r4
	orrne 	r1,r1,#16
	
	ldr 	r0,=szMOV
	bl 	DrawResult
	add 	r8,r8,#8


	@ MVN
	mov 	r1,#0
	ldr 	r2,=labelthree	
	ldr 	r3,=0xFFFFFFFF
	eor 	r2,r2,r3
	mvn 	r3,r15
	cmp 	r3,r2
labelthree:	
	orrne 	r1,r1,#BAD_Rd
	ldr 	r0,=szMVN
	bl 	DrawResult
	add 	r8,r8,#8
	
	@ ORR
	mov 	r1,#0
	mov 	r2,#2
	mov 	r3,#3
	movs 	r4,r3,lsr#1	@ set carry 
	orrs 	r3,r3,r2,rrx
	orrcs 	r1,r1,#1
	orrpl 	r1,r1,#2
	orreq 	r1,r1,#8
	ldr 	r4,=0x80000003
	cmp 	r4,r3
	orrne 	r1,r1,#BAD_Rd
	ldr 	r0,=szORR
	bl 	DrawResult
	add 	r8,r8,#8
	
	
	@ RSC
	mov 	r1,#0
	mov 	r2,#2
	mov 	r3,#3
	adds 	r9,r9,r9	@ clear carry
	rscs 	r3,r2,r3
	orrcc 	r1,r1,#1
	orrmi 	r1,r1,#2
	orrne 	r1,r1,#8
	cmp 	r2,#2
	orrne 	r1,r1,#BAD_Rd
	ldr 	r0,=szRSC
	bl 	DrawResult
	add 	r8,r8,#8

	
	@ SBC
	mov 	r1,#0
	ldr 	r2,=0xFFFFFFFF
	adds 	r3,r2,r2	@ set carry
	sbcs 	r2,r2,r2
	orrcc 	r1,r1,#1
	orrmi 	r1,r1,#2
	orrne 	r1,r1,#8
	adds 	r9,r9,r9	@ clear carry
	sbcs 	r2,r2,#0
	orreq 	r1,r1,#8
	orrcs 	r1,r1,#1
	orrpl 	r1,r1,#2
	ldr 	r0,=szSBC
	bl 	DrawResult
	add 	r8,r8,#8
	

	@ MLA
	mov 	r1,#0
	ldr 	r2,=0xFFFFFFF6
	mov 	r3,#0x14
	ldr 	r4,=0xD0
	mlas 	r2,r3,r2,r4
	orrmi 	r1,r1,#2
	orreq 	r1,r1,#8
	cmp 	r2,#8
	orrne 	r1,r1,#16
	ldr 	r0,=szMLA
	bl 	DrawResult
	add 	r8,r8,#8


	@ MUL
	mov 	r1,#0
	ldr 	r2,=0xFFFFFFF6
	mov 	r3,#0x14
	ldr 	r4,=0xFFFFFF38
	muls 	r2,r3,r2
	orrpl 	r1,r1,#2
	orreq 	r1,r1,#8
	cmp 	r2,r4
	orrne 	r1,r1,#16
	ldr 	r0,=szMUL
	bl 	DrawResult
	add 	r8,r8,#8

	@@ XXX UMULL and SMULL were both removed because they don't exist on
	@@ Dreamcast's ARM

	ldmfd 	sp!,{lr}
	mov 	pc,lr
.pool
.align

DrawText: 
	mov 	pc,lr

DrawResult:
	mov 	pc,lr

szADC:		.asciz "ADC"
szADD:		.asciz "ADD"
szAND:		.asciz "AND"
szBIC:		.asciz "BIC"
szCMN:		.asciz "CMN"
szCMP:		.asciz "CMP"
szEOR:		.asciz "EOR"
szMOV:		.asciz "MOV"
szMVN:		.asciz "MVN"
szORR:		.asciz "ORR"
szRSB:		.asciz "RSB"
szRSC:		.asciz "RSC"
szSBC:		.asciz "SBC"
szSUB:		.asciz "SUB"
szTEQ:		.asciz "TEQ"
szTST:		.asciz "TST"

szMLA:		.asciz "MLA"
szMUL:		.asciz "MUL"
szUMULL: 	.asciz "UMULL"
szUMLAL: 	.asciz "UMLAL"
szSMULL: 	.asciz "SMULL"
szSMLAL:	.asciz "SMLAL"


szALU1:		.asciz  "ALU TESTS PART 1"
szALU2:		.asciz  "ALU PT 2 / MISC"
szLS1: 		.asciz   "LOAD TESTS PART 1"
szLS2: 		.asciz   "LOAD TESTS PART 2"
szLS3: 		.asciz   "LOAD TESTS PART 3"
szLS4: 		.asciz   "LOAD TESTS PART 4"
szLS5: 		.asciz   "LOAD TESTS PART 5"
szLS6: 		.asciz   "LOAD TESTS PART 6"
szLS7: 		.asciz   "LOAD TESTS PART 7"
szLS8: 		.asciz   "LOAD TESTS PART 8"
szLDM1:		.asciz 	"LDM/STM TESTS 1"
szAV5:		.asciz  "ARM V5TE TESTS"