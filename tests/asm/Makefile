# Original Makefile from https://github.com/snickerbockers/dc-arm7wrestler

ARM_AS=arm-linux-gnueabi-as
ARM_LD=arm-linux-gnueabi-ld
ARM_OBJCOPY=arm-linux-gnueabi-objcopy

asm_files = $(wildcard *.s)
o_files := $(patsubst %.s, %.o, $(asm_files))
elf_files := $(patsubst %.s, %.elf, $(asm_files))
bin_files := $(patsubst %.s, %.bin, $(asm_files))

all: ${bin_files}

clean:
		rm -f *.o *.elf *.bin

%.o: ./%.s
		$(ARM_AS) -EL -mcpu=arm7di -o $@ ./$<

%.elf: %.o
		$(ARM_LD) -Ttext 0x00000000 $< -o $@
		rm $<

%.bin: %.elf
		$(ARM_OBJCOPY) -O binary -j .text -j .data -j .bss -j .rodata --set-section-flags .data=alloc,load,contents $< $@
		rm $<
		mv $@ ../bin/$@
