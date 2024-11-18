const std = @import("std");
const builtin = @import("builtin");
const arm7_interpreter_log = std.log.scoped(.arm7_interpreter);

const arm7 = @import("arm7.zig");

pub fn execute(self: *arm7.ARM7, instr: u32) void {
    if (!handle_condition(self, arm7.ARM7.get_instr_condition(instr)))
        return;
    const tag = arm7.ARM7.get_instr_tag(instr);
    InstructionHandlers[arm7.JumpTable[tag]](self, instr);
}

pub fn tick(self: *arm7.ARM7) void {
    const instr = self.instruction_pipeline[0];
    self.instruction_pipeline[0] = self.fetch();
    execute(self, instr);
    self.check_fiq();
}

pub fn is_valid(instr: u32) bool {
    const tag = arm7.ARM7.get_instr_tag(instr);
    const handler = InstructionHandlers[arm7.JumpTable[tag]];
    return handler != handle_undefined and handler != handle_invalid;
}

pub const InstructionHandlers = [_]*const fn (cpu: *arm7.ARM7, instruction: u32) void{
    handle_branch_and_exchange,
    handle_block_data_transfer,
    handle_branch,
    handle_software_interrupt,
    handle_undefined,
    handle_single_data_transfer,
    handle_single_data_swap,
    handle_multiply,
    handle_multiply_long,
    handle_halfword_data_transfer_register_offset,
    handle_halfword_data_transfer_immediate_offset,
    handle_coprocessor_data_transfer,
    handle_coprocessor_data_operation,
    handle_coprocessor_register_transfer,
    handle_mrs,
    handle_msr,
    handle_data_processing,

    handle_invalid,
};

pub inline fn handle_condition(cpu: *arm7.ARM7, cond: arm7.Condition) bool {
    switch (cond) {
        .EQ => return cpu.cpsr.z,
        .NE => return !cpu.cpsr.z,
        .CS => return cpu.cpsr.c,
        .CC => return !cpu.cpsr.c,
        .MI => return cpu.cpsr.n,
        .PL => return !cpu.cpsr.n,
        .VS => return cpu.cpsr.v,
        .VC => return !cpu.cpsr.v,
        .HI => return cpu.cpsr.c and !cpu.cpsr.z,
        .LS => return !cpu.cpsr.c or cpu.cpsr.z,
        .GE => return cpu.cpsr.n == cpu.cpsr.v,
        .LT => return cpu.cpsr.n != cpu.cpsr.v,
        .GT => return !cpu.cpsr.z and (cpu.cpsr.n == cpu.cpsr.v),
        .LE => return cpu.cpsr.z or (cpu.cpsr.n != cpu.cpsr.v),
        .AL => return true,
        .Invalid => @panic("Invalid condition"),
    }
    return true;
}

pub const ShiftType = enum(u2) {
    LSL = 0b00,
    LSR = 0b01,
    ASR = 0b10,
    ROR = 0b11, // or RRX
};

pub const ScaledRegisterOffset = packed struct(u12) {
    rm: u4,
    register_specified: u1,
    shift_type: ShiftType,
    shift_amount: packed union {
        imm: u5,
        reg: packed struct {
            _: u1 = 0,
            rs: u4,
        },
    },
};

// When R15 is the source register (Rd) of a register store (STR) instruction, the stored value will be address of the instruction plus 12.
// NOTE: It is actually implementation defined, and maybe should be exposed as a compile time option?
//       Also, it applies to STM too and PC is already +8 due to pipelining.
const STR_STM_store_R15_plus_4 = true;

fn handle_branch_and_exchange(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    const inst: arm7.BranchAndExchangeInstruction = @bitCast(instruction);
    _ = inst;

    std.debug.print("Unimplemented branch and exchange\n", .{});
    @panic("Unimplemented branch and exchange");
}

// LDM, STM
fn handle_block_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.BlockDataTransferInstruction = @bitCast(instruction);

    const base = cpu.r[inst.rn];

    var addr = base;

    const stride: u32 = @bitCast(if (inst.u == 1) @as(i32, 4) else -4);

    // The lowest-numbered register is stored at the lowest memory address and
    // the highest-numbered register at the highest memory address.

    // For simplicity we'll always loop through registers in ascending order,
    // adjust the address to the base of the range when decrementing.
    if (inst.u == 0) addr +%= stride *% (@popCount(inst.reg_list) - 1);

    // Pre indexing
    if (inst.p == 1) addr +%= stride;

    // Writeback
    // NOTE: A load including rn in the register list will always overwrite this.
    //       However, a store will store the unchanged value if it is the first register
    //       in the list, and the updated value otherwise.
    if (inst.w == 1)
        cpu.r[inst.rn] +%= stride *% @popCount(inst.reg_list);

    var first_store = true;

    // The address should normally be a word aligned quantity and non-word aligned addresses do not affect the
    // instruction. However, the bottom 2 bits of the address will appear on A[1:0] and might be interpreted by
    // the memory system.
    addr &= 0xFFFFFFFC;

    // LDM (1) / STM (1)
    if (inst.s == 0) {
        if (inst.l == 1) {
            // Load LDM(1)
            inline for (0..15) |i| {
                if (inst.reg(i)) {
                    cpu.r[i] = cpu.read(u32, addr);
                    addr += 4;
                }
            }
            if (inst.reg(15)) {
                const value = cpu.read(u32, addr);
                cpu.set_pc(value);
                // cpu.cpsr.t = value & 1 == 1; // FIXME: If can't find the source of this anymore...
                addr += 4;
            }
        } else {
            // Store STM(1)
            inline for (0..16) |i| {
                if (inst.reg(i)) {
                    var val = cpu.r[i];
                    if (STR_STM_store_R15_plus_4 and i == 15) val += 4;

                    // See Writeback above.
                    const value = if (first_store and i == inst.rn) base else val;
                    first_store = false;

                    cpu.write(u32, addr, value);
                    addr += 4;
                }
            }
        }
    } else {
        if (inst.l == 1) {
            if (!inst.reg(15)) {
                // LDM (2) - Loads User mode registers when the processor is in a privileged mode.
                if (cpu.cpsr.m == .FastInterrupt) {
                    inline for (0..8) |i| {
                        if (inst.reg(i)) {
                            cpu.r[i] = cpu.read(u32, addr);
                            addr += 4;
                        }
                    }
                    inline for (0..5) |i| {
                        if (inst.reg(8 + i)) {
                            cpu.r_fiq_8_12[i] = cpu.read(u32, addr);
                            addr += 4;
                        }
                    }
                } else {
                    inline for (0..13) |i| {
                        if (inst.reg(i)) {
                            cpu.r[i] = cpu.read(u32, addr);
                            addr += 4;
                        }
                    }
                }
                if (inst.reg(13)) {
                    cpu.banked_regs(cpu.cpsr.m)[0] = cpu.read(u32, addr);
                    if (cpu.cpsr.m == .User or cpu.cpsr.m == .System) cpu.r[13] = cpu.banked_regs(.User)[0];
                    addr += 4;
                }
                if (inst.reg(14)) {
                    cpu.banked_regs(cpu.cpsr.m)[1] = cpu.read(u32, addr);
                    if (cpu.cpsr.m == .User or cpu.cpsr.m == .System) cpu.r[14] = cpu.banked_regs(.User)[1];
                    addr += 4;
                }
            } else {
                // LDM (3) - Loads a subset, or possibly all, of the general-purpose registers and the PC from sequential memory
                // locations. Also, the SPSR of the current mode is copied to the CPSR. This is useful for returning from an exception.
                inline for (0..15) |i| {
                    if (inst.reg(i)) {
                        cpu.r[i] = cpu.read(u32, addr);
                        addr += 4;
                    }
                }
                cpu.restore_cpsr();
                cpu.set_pc(cpu.read(u32, addr) & 0xFFFFFFFC);
            }
        } else {
            // STM with s=1 - User bank transfer
            // User bank registers are transferred rather than the register bank.
            // std.debug.assert(inst.w == 0);
            inline for (0..16) |i| {
                if (inst.reg(i)) {
                    var val = if (i == 13 or i == 14 and cpu.cpsr.m != .User) cpu.banked_regs(cpu.cpsr.m)[i - 13] else cpu.r[i];
                    if (STR_STM_store_R15_plus_4 and i == 15) val += 4;

                    // See Writeback above.
                    const value = if (first_store and i == inst.rn) base else val;
                    first_store = false;

                    cpu.write(u32, addr, value);
                    addr += 4;
                }
            }
        }
    }
}

fn handle_branch(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.BranchInstruction = @bitCast(instruction);
    const offset = arm7.sign_extend(u26, @as(u26, @intCast(inst.offset)) << 2);

    // Branch with link, Saves PC to R14 (LR)
    // "Branch with Link (BL) writes the old PC into the link register (R14) of the current bank.
    //  The PC value written into R14 is adjusted to allow for the prefetch, and contains the
    //  address of the instruction following the branch and link instruction. Note that the CPSR
    //  is not saved with the PC and R14[1:0] are always cleared.""
    if (inst.l == 1) {
        cpu.set_lr((cpu.pc() - 4) & 0xFFFFFFFC);
    }

    // NOTE: The value in PC is actually this instruction address + 8 (due to pipelining), this is intended.
    // "The branch offset must take account of the prefetch operation, which causes the PC to be 2 words (8 bytes) ahead of the current instruction"
    cpu.set_pc(cpu.pc() +% offset);
}

fn handle_software_interrupt(cpu: *arm7.ARM7, _: u32) void {
    const spc = cpu.pc() - 4;
    cpu.spsr_svc = cpu.cpsr;
    cpu.change_mode(.Supervisor);
    cpu.cpsr.t = false; // Execute in ARM state (NOTE: We don't have a way to execute in THUMB state)
    cpu.cpsr.i = true; // Disable normal interrupts
    cpu.r[14] = spc; // R14_svc = address of next instruction after the SWI instruction

    // FIXME: Something about high vectors? I guess the interrupt vector can be located at 0xFFFF0000 instead of 0x00000000 in some cases. we don't handle it.
    cpu.set_pc(0x00000008);
}

fn handle_undefined(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    const inst: arm7.UndefinedInstruction = @bitCast(instruction);
    _ = inst;

    std.debug.print("Undefined instruction: {X:0>8}\n", .{instruction});
    @panic("Undefined instruction");
}

fn handle_single_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.SingleDataTransferInstruction = @bitCast(instruction);
    std.debug.assert(inst._tag == 0b01);

    std.debug.assert(builtin.is_test or inst.rn != 15 or inst.w == 0); // Write-back must not be specified if R15 is specified as the base register (Rn)

    var offset: u32 = inst.offset;
    if (inst.i == 1) { // Offset is a register
        const sro: ScaledRegisterOffset = @bitCast(inst.offset);
        std.debug.assert(builtin.is_test or sro.rm != 15); // R15 must not be specified as the register offset (Rm).
        std.debug.assert(builtin.is_test or sro.register_specified == 0); // Register specified shift amounts are not available in this instruction class
        offset = offset_from_register(cpu, inst.offset).shifter_operand;
    }

    const signed_offset: u32 = if (inst.u == 1) offset else (~offset +% 1);

    const base = cpu.r[inst.rn];
    const addr: u32 = @bitCast(if (inst.p == 1) base +% signed_offset else base);

    // In the case of post-indexed addressing (p == 0), the write back bit is redundant and must be set to zero,
    // since the old base value can be retained by setting the offset to zero.
    // The only use of the W bit in a post-indexed data transfer is in privileged mode
    // code, where setting the W bit forces non-privileged mode for the transfer, allowing the
    // operating system to generate a user address in a system where the memory
    // management hardware makes suitable use of this hardware.
    if (inst.w == 1 and inst.p == 0) {
        // ?? Probably doesn't apply to our use case.
    }

    if (inst.l == 0) {
        // Store to memory
        var val = cpu.r[inst.rd];

        if (STR_STM_store_R15_plus_4 and inst.rd == 15) val += 4;

        if (inst.b == 1)
            cpu.write(u8, addr, @truncate(val))
        else {
            // A word store (STR) should generate a word aligned address. The word presented to
            // the data bus is not affected if the address is not word aligned.
            cpu.write(u32, addr, val);
        }
        // Post-indexed data transfers always write back the modified base.
        if (inst.w == 1 or inst.p == 0)
            cpu.r[inst.rn] +%= signed_offset;
    } else {
        if (inst.w == 1 or inst.p == 0)
            cpu.r[inst.rn] +%= signed_offset;

        // Load from memory
        if (inst.b == 1) {
            cpu.r[inst.rd] = cpu.read(u8, addr);
        } else {
            cpu.r[inst.rd] = cpu.read(u32, addr);
        }
        if (inst.rd == 15) cpu.reset_pipeline();
    }
}

fn handle_single_data_swap(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.SingleDataSwapInstruction = @bitCast(instruction);

    std.debug.assert(builtin.is_test or inst.rd != 15);
    std.debug.assert(builtin.is_test or inst.rn != 15);
    std.debug.assert(builtin.is_test or inst.rs != 15);

    var addr = cpu.r[inst.rn];
    if (STR_STM_store_R15_plus_4 and inst.rn == 15) addr += 4;
    var reg = cpu.r[inst.rm];
    if (inst.b == 1) {
        cpu.r[inst.rd] = cpu.read(u8, addr);
        if (STR_STM_store_R15_plus_4 and inst.rm == 15) reg += 4;
        cpu.write(u8, addr, @truncate(reg));
    } else {
        cpu.r[inst.rd] = cpu.read(u32, addr);
        if (STR_STM_store_R15_plus_4 and inst.rm == 15) reg += 4;
        cpu.write(u32, addr, reg);
    }

    // NOTE: I think this should be an illegal instruction, but we'll handle it
    //       for easier testing.
    if (inst.rd == 15) {
        cpu.reset_pipeline();
    }
}

fn handle_multiply(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MultiplyInstruction = @bitCast(instruction);

    if (!builtin.is_test) {
        std.debug.assert(inst.rd != inst.rm); // NOTE: Per the docs, this case should return 0 when A == 0
        std.debug.assert(inst.rd != 15);
        std.debug.assert(inst.rm != 15);
        std.debug.assert(inst.rs != 15);
    }

    var op1 = cpu.r[inst.rm];
    var op2 = cpu.r[inst.rs];
    var op3 = cpu.r[inst.rn];
    if (inst.rm == 15) op1 += 4;
    if (inst.rs == 15) op2 += 4;
    if (inst.rn == 15) op3 += 4;

    var result: u32 = op1 *% op2;

    if (inst.a == 1) {
        std.debug.assert(builtin.is_test or inst.rn != 15);
        result +%= op3;
    }

    cpu.r[inst.rd] = result;

    if (inst.s == 1) {
        cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
        cpu.cpsr.z = cpu.r[inst.rd] == 0;
    }

    if (inst.rd == 15) cpu.reset_pipeline();
}

fn handle_multiply_long(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    const inst: arm7.MultiplyLongInstruction = @bitCast(instruction);
    _ = inst;

    std.debug.print("Unimplemented multiply long\n", .{});
    @panic("Unimplemented multiply long");
}

fn handle_halfword_data_transfer_register_offset(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    const inst: arm7.HalfwordDataTransferRegisterOffsetInstruction = @bitCast(instruction);
    _ = inst;

    std.debug.print("Unimplemented halfword data transfer register offset\n", .{});
    @panic("Unimplemented halfword data transfer register offset");
}

fn handle_halfword_data_transfer_immediate_offset(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    const inst: arm7.HalfwordDataTransferImmediateOffsetInstruction = @bitCast(instruction);
    _ = inst;

    std.debug.print("Unimplemented halfword data transfer immediate offset\n", .{});
    @panic("Unimplemented halfword data transfer immediate offset");
}

fn handle_coprocessor_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    const inst: arm7.CoprocessorDataTransferInstruction = @bitCast(instruction);
    _ = inst;

    std.debug.print("Unimplemented coprocessor data transfer\n", .{});
    @panic("Unimplemented coprocessor data transfer");
}

fn handle_coprocessor_data_operation(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    const inst: arm7.CoprocessorDataOperationInstruction = @bitCast(instruction);
    _ = inst;

    std.debug.print("Unimplemented coprocessor data operation\n", .{});
    @panic("Unimplemented coprocessor data operation");
}

fn handle_coprocessor_register_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    const inst: arm7.CoprocessorRegisterTransferInstruction = @bitCast(instruction);
    _ = inst;

    std.debug.print("Unimplemented coprocessor register transfer\n", .{});
    @panic("Unimplemented coprocessor register transfer");
}

// Move PSR to general-purpose register
fn handle_mrs(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MRSInstruction = @bitCast(instruction);

    std.debug.assert(inst.rd != 15);

    if (inst.r == 1) {
        cpu.r[inst.rd] = @bitCast(cpu.spsr().*);
    } else {
        cpu.r[inst.rd] = @bitCast(cpu.cpsr);
    }
}

// Move to Status Register
fn handle_msr(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MSRInstruction = @bitCast(instruction);

    const UnallocMask: u32 = 0x06F0FC00;
    const UserMask: u32 = 0xF80F0200;
    const PrivMask: u32 = 0x000001DF;
    const StateMask: u32 = 0x01000020;

    const operand = if (inst.i == 1) immediate_shifter_operand(inst.source_operand) else cpu.r[inst.source_operand & 0x1F];

    std.debug.assert(operand & UnallocMask == 0);
    const byte_mask: u32 = (if (inst.field_mask.c == 1) @as(u32, 0x000000FF) else 0x00000000) |
        (if (inst.field_mask.x == 1) @as(u32, 0x0000FF00) else 0x00000000) |
        (if (inst.field_mask.s == 1) @as(u32, 0x00FF0000) else 0x00000000) |
        (if (inst.field_mask.f == 1) @as(u32, 0xFF000000) else 0x00000000);
    if (inst.r == 0) {
        std.debug.assert(!cpu.in_a_privileged_mode() or operand & StateMask == 0);
        const mask = byte_mask & if (cpu.in_a_privileged_mode()) (UserMask | PrivMask) else UserMask;
        cpu.set_cpsr(@bitCast((@as(u32, @bitCast(cpu.cpsr)) & ~mask) | (operand & mask)));
    } else {
        const mask = byte_mask & (UserMask | PrivMask | StateMask);
        cpu.spsr().* = @bitCast((@as(u32, @bitCast(cpu.spsr().*)) & ~mask) | (operand & mask));
    }
}

inline fn n_flag(v: u32) bool {
    return (v & 0x80000000) != 0;
}

// Returns 1 if the addition or subtraction specified as its parameter caused a 32-bit signed overflow. Addition
// generates an overflow if both operands have the same sign (bit[31]), and the sign of the result is different to
// the sign of both operands. Subtraction causes an overflow if the operands have different signs, and the first
// operand and the result have different signs.
inline fn overflow_from_add(op1: u32, op2: u32) bool {
    const r: u32 = @bitCast(@as(i32, @bitCast(op1)) +% @as(i32, @bitCast(op2)));
    return op1 & 0x80000000 == op2 & 0x80000000 and op1 & 0x80000000 != r & 0x80000000;
}

inline fn overflow_from_addc(op1: u32, op2: u32, carry: u32) bool {
    const op = [3]i32{ @bitCast(op1), @bitCast(op2), @bitCast(carry) };
    const r0 = @addWithOverflow(op[0], op[1]);
    const r1 = @addWithOverflow(r0[0], op[2]);
    return r0[1] == 1 or r1[1] == 1;
}

inline fn overflow_from_sub(op1: u32, op2: u32) bool {
    const r: u32 = @bitCast(@as(i32, @bitCast(op1)) -% @as(i32, @bitCast(op2)));
    return op1 & 0x80000000 != op2 & 0x80000000 and op1 & 0x80000000 != r & 0x80000000;
}

inline fn overflow_from_subc(op1: u32, op2: u32, carry: u32) bool {
    const op = [3]i32{ @bitCast(op1), @bitCast(op2), @bitCast(carry) };
    const r0 = @subWithOverflow(op[0], op[1]);
    const r1 = @subWithOverflow(r0[0], op[2]);
    return r0[1] == 1 or r1[1] == 1;
}

// Returns 1 if the addition specified as its parameter caused a carry (true result is bigger than 2^32−1, where
// the operands are treated as unsigned integers), and returns 0 in all other cases. This delivers further
// information about an addition which occurred earlier in the pseudo-code. The addition is not repeated.
fn carry_from(op1: u32, op2: u32) bool {
    return @as(u64, op1) + @as(u64, op2) > 0xFFFFFFFF;
}

fn carry_from_addc(op1: u64, op2: u64, c: u64) bool {
    return op1 + op2 + c > 0xFFFFFFFF;
}

// Returns 1 if the subtraction specified as its parameter caused a borrow (the true result is less than 0, where
// the operands are treated as unsigned integers), and returns 0 in all other cases. This delivers further
// information about a subtraction which occurred earlier in the pseudo-code. The subtraction is not repeated.
fn borrow_from(op1: u32, op2: u32) bool {
    return op2 > op1;
}

fn borrow_from_subc(op1: u32, op2: u32, c: u32) bool {
    return op1 < @as(u64, op2) + c;
}

fn handle_data_processing(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.DataProcessingInstruction = @bitCast(instruction);

    // TODO: See "Writing to R15"

    const op1 = cpu.r[inst.rn];

    const shifter_result = if (inst.i == 0)
        offset_from_register(cpu, inst.operand2)
    else
        operand_2_immediate(cpu, inst.operand2);

    var op2 = shifter_result.shifter_operand;

    // TODO: Yes, this can and must be refactored.
    switch (inst.opcode) {
        .AND => {
            cpu.r[inst.rd] = op1 & op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .EOR => {
            cpu.r[inst.rd] = op1 ^ op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .SUB => {
            cpu.r[inst.rd] = op1 -% op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = !borrow_from(op1, op2);
                cpu.cpsr.v = overflow_from_sub(op1, op2);
            }
        },
        .RSB => {
            cpu.r[inst.rd] = op2 -% op1;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = !borrow_from(op2, op1);
                cpu.cpsr.v = overflow_from_sub(op2, op1);
            }
        },
        .ADD => {
            cpu.r[inst.rd] = op1 +% op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = carry_from(op1, op2);
                cpu.cpsr.v = overflow_from_add(op1, op2);
            }
        },
        .ADC => {
            const carry: u32 = if (cpu.cpsr.c) 1 else 0;
            op2 +%= carry;
            cpu.r[inst.rd] = op1 +% op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = carry_from_addc(op1, op2, carry);
                cpu.cpsr.v = overflow_from_addc(op1, op2, carry);
            }
        },
        .SBC => {
            const carry: u32 = if (cpu.cpsr.c) 0 else 1;
            cpu.r[inst.rd] = op1 -% op2 -% carry;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = !borrow_from_subc(op1, op2, carry);
                cpu.cpsr.v = overflow_from_subc(op1, op2, carry);
            }
        },
        .RSC => {
            const carry: u32 = if (cpu.cpsr.c) 0 else 1;
            cpu.r[inst.rd] = op2 -% op1 -% carry;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = !borrow_from_subc(op2, op1, carry);
                cpu.cpsr.v = overflow_from_subc(op2, op1, carry);
            }
        },
        .TST => {
            const r = op1 & op2;
            std.debug.assert(inst.s == 1);
            cpu.cpsr.n = n_flag(r);
            cpu.cpsr.z = r == 0;
            cpu.cpsr.c = shifter_result.shifter_carry_out;
        },
        .TEQ => {
            const r = op1 ^ op2;
            std.debug.assert(inst.s == 1);
            cpu.cpsr.n = n_flag(r);
            cpu.cpsr.z = r == 0;
            cpu.cpsr.c = shifter_result.shifter_carry_out;
        },
        .CMP => {
            const r = op1 -% op2;
            std.debug.assert(inst.s == 1);
            cpu.cpsr.n = n_flag(r);
            cpu.cpsr.z = r == 0;
            cpu.cpsr.c = !borrow_from(op1, op2);
            cpu.cpsr.v = overflow_from_sub(op1, op2);
        },
        .CMN => {
            const r = op1 +% op2;
            std.debug.assert(inst.s == 1);
            cpu.cpsr.n = n_flag(r);
            cpu.cpsr.z = r == 0;
            cpu.cpsr.c = carry_from(op1, op2);
            cpu.cpsr.v = overflow_from_add(op1, op2);
        },
        .ORR => {
            cpu.r[inst.rd] = op1 | op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .MOV => {
            cpu.r[inst.rd] = op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .BIC => {
            cpu.r[inst.rd] = op1 & ~op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .MVN => {
            cpu.r[inst.rd] = ~op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.restore_cpsr();
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r[inst.rd]);
                cpu.cpsr.z = cpu.r[inst.rd] == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
    }

    // If PC as been written to, flush pipeline and refill it.
    switch (inst.opcode) {
        .TST, .TEQ, .CMP, .CMN => {},
        else => if (inst.rd == 15) cpu.reset_pipeline(),
    }
}

fn handle_invalid(cpu: *arm7.ARM7, instruction: u32) void {
    arm7_interpreter_log.err("Invalid Instruction @{X:0>8}: {X:0>8} {b:0>32}", .{ cpu.pc() - 8, instruction, instruction });
}

const barrel_shifter_result = struct { shifter_operand: u32, shifter_carry_out: bool };

pub inline fn offset_from_register(cpu: *arm7.ARM7, operand2: u12) barrel_shifter_result {
    const sro: ScaledRegisterOffset = @bitCast(operand2);

    var rm = cpu.r[sro.rm];

    // The PC value will be the address of the instruction, plus 8 or 12 bytes due to instruction
    // prefetching. If the shift amount is specified in the instruction, the PC will be 8 bytes
    // ahead. If a register is used to specify the shift amount the PC will be 12 bytes ahead.
    if (sro.rm == 15 and sro.register_specified == 1)
        rm += 4;

    const shift_amount: u8 = if (sro.register_specified == 0)
        // Fixed shift
        sro.shift_amount.imm
    else
        // Shift amount from register (Rs) - Bottom byte only
        @truncate(cpu.r[sro.shift_amount.reg.rs] & 0xFF);

    // NOTE: This is just here to help the compiler optimize (unless it comes from a register, the shift_amount is always < 32).
    std.debug.assert(sro.register_specified != 0 or shift_amount < 32);

    // If this byte [shift loaded from register] is zero, the unchanged contents of Rm will be used as the second operand,
    // and the old value of the CPSR C flag will be passed on as the shifter carry output.
    if (sro.register_specified == 1 and shift_amount == 0) {
        return .{
            .shifter_operand = rm,
            .shifter_carry_out = cpu.cpsr.c,
        };
    }

    switch (sro.shift_type) {
        .LSL => {
            if (shift_amount == 32) {
                return .{
                    .shifter_operand = 0,
                    .shifter_carry_out = rm & 1 != 0,
                };
            } else if (shift_amount > 32) {
                return .{
                    .shifter_operand = 0,
                    .shifter_carry_out = false,
                };
            } else {
                return .{
                    .shifter_operand = rm << @intCast(shift_amount),
                    .shifter_carry_out = if (shift_amount == 0) cpu.cpsr.c else (rm << @intCast(shift_amount - 1)) & 0x80000000 != 0,
                };
            }
        },
        .LSR => {
            if (shift_amount == 0 or shift_amount == 32) {
                // The form of the shift field which might be expected to correspond to LSR #0 is used to
                // encode LSR #32, which has a zero result with bit 31 of Rm as the carry output. Logical
                // shift right zero is redundant as it is the same as logical shift left zero, so the assembler
                // will convert LSR #0 (and ASR #0 and ROR #0) into LSL #0, and allow LSR #32 to be
                // specified.
                return .{
                    .shifter_operand = 0,
                    .shifter_carry_out = rm & 0x80000000 != 0,
                };
            } else if (shift_amount > 32) {
                return .{
                    .shifter_operand = 0,
                    .shifter_carry_out = false,
                };
            } else return .{
                .shifter_operand = rm >> @intCast(shift_amount),
                .shifter_carry_out = (rm >> @intCast(shift_amount - 1)) & 1 != 0,
            };
        },
        .ASR => {
            if (shift_amount == 0 or shift_amount >= 32) {
                // The form of the shift field which might be expected to give ASR #0 is used to encode
                // ASR #32. Bit 31 of Rm is again used as the carry output, and each bit of operand 2 is
                // also equal to bit 31 of Rm. The result is therefore all ones or all zeros, according to the
                // value of bit 31 of Rm.
                return .{
                    .shifter_operand = if (rm & 0x80000000 != 0) 0xFFFFFFFF else 0,
                    .shifter_carry_out = rm & 0x80000000 != 0,
                };
            } else {
                const sa: u5 = @truncate(shift_amount);
                return .{
                    .shifter_operand = arithmetic_shift_right(rm, sa),
                    .shifter_carry_out = (rm >> (sa - 1)) & 1 != 0,
                };
            }
        },
        .ROR => {
            if (shift_amount == 0 and sro.register_specified == 0) {
                // The form of the shift field which might be expected to give ROR #0 is used to encode
                // a special function of the barrel shifter, rotate right extended (RRX). This is a rotate right
                // by one bit position of the 33 bit quantity formed by appending the CPSR C flag to the
                // most significant end of the contents of Rm.
                return .{
                    .shifter_operand = (if (cpu.cpsr.c) @as(u32, 0x80000000) else 0) | rm >> 1,
                    .shifter_carry_out = rm & 1 == 1,
                };
            } else if ((shift_amount % 32) == 0) {
                return .{
                    .shifter_operand = rm,
                    .shifter_carry_out = rm & 0x80000000 != 0,
                };
            } else {
                // ROR by n where n is greater than 32 will give the same result and carry out as ROR by n-32
                const sa: u5 = @truncate(shift_amount);
                return .{
                    .shifter_operand = std.math.rotr(u32, rm, sa),
                    .shifter_carry_out = (rm >> (sa - 1)) & 1 == 1,
                };
            }
        },
    }
}

pub inline fn immediate_shifter_operand(operand2: u12) u32 {
    const rotate_imm = operand2 >> 8;
    return std.math.rotr(u32, arm7.zero_extend(operand2 & 0xFF), 2 * rotate_imm);
}

pub inline fn operand_2_immediate(cpu: *arm7.ARM7, operand2: u12) barrel_shifter_result {
    const shifter_operand = immediate_shifter_operand(operand2);
    return .{
        .shifter_operand = shifter_operand,
        .shifter_carry_out = if (operand2 >> 8 == 0) cpu.cpsr.c else (shifter_operand >> 31) == 1,
    };
}

// Arithmetic Shift Right. Register contents are treatedas two's complement signed integers. The sign bit is copied into vacated bits.
inline fn arithmetic_shift_right(val: u32, shift_amount: u5) u32 {
    if (val & 0x80000000 != 0) {
        return ~(~val >> shift_amount);
    } else {
        return val >> shift_amount;
    }
}
