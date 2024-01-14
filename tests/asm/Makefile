# Original Makefile from https://github.com/snickerbockers/dc-arm7wrestler

ARM_AS=arm-linux-gnueabi-as
ARM_LD=arm-linux-gnueabi-ld
ARM_OBJCOPY=arm-linux-gnueabi-objcopy

all: basic.bin

clean:
		rm -f basic.o basic.elf basic.bin

basic.o: ./basic.s
		$(ARM_AS) -EL -mcpu=arm7di -o basic.o ./basic.s

basic.elf: basic.o
		$(ARM_LD) -Ttext 0x00000000 basic.o -o basic.elf

basic.bin: basic.elf
		$(ARM_OBJCOPY) -O binary -j .text -j .data -j .bss -j .rodata --set-section-flags .data=alloc,load,contents basic.elf basic.bin
		rm basic.o
		rm basic.elf
		mv basic.bin ../bin/basic.bin