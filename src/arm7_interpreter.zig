const std = @import("std");
const arm7_interpreter_log = std.log.scoped(.arm7_interpreter);

const arm7 = @import("arm7.zig");

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

inline fn handle_condition(cpu: *arm7.ARM7, cond: arm7.Condition) bool {
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
        .Invalid => unreachable,
    }
    return true;
}

pub const LogicalShift = enum(u2) {
    LSL = 0b00,
    LSR = 0b01,
    ASR = 0b10,
    ROR = 0b11, // or RRX
};

pub const ScaledRegisterOffset = packed struct(u12) {
    rm: u4,
    _: u1 = 0,
    shift: LogicalShift,
    shift_imm: u5,
};

fn handle_branch_and_exchange(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.BranchAndExchangeInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
    @panic("Unimplemented branch and exchange");
}

fn handle_block_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.BlockDataTransferInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
    @panic("Unimplemented block data transfer");
}

fn handle_branch(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.BranchInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    const offset = arm7.sign_extend_u26(inst.offset) << 2; // NOTE: The value in PC in actually this instruction address + 8 (due to pipelining)

    // Branch with link
    // Saves PC to R14 (LR)
    if (inst.l == 1) {
        cpu.lr().* = (cpu.pc().* - 4) & 0xFFFFFFFC;
    }

    if (offset & 0x80000000 != 0) {
        cpu.pc().* += (offset ^ 0xFFFFFFFF) + 1;
    } else {
        cpu.pc().* += offset;
    }

    cpu.reset_pipeline();
}

fn handle_software_interrupt(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.SoftwareInterruptInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented software interrupt");
}

fn handle_undefined(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.UndefinedInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
    @panic("Undefined instruction");
}

fn handle_single_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.SingleDataTransferInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    std.debug.assert(inst.rn != 15 or inst.w == 0); // Write-back must not be specified if R15 is specified as the base register (Rn)

    var offset: u32 = inst.offset;
    if (inst.i == 1) {
        const sro: ScaledRegisterOffset = @bitCast(inst.offset);
        std.debug.assert(sro.rm != 15); // R15 must not be specified as the register offset (Rm).
        offset = switch (sro.shift) {
            .LSL => cpu.r(sro.rm).* << sro.shift_imm,
            .LSR => if (sro.shift_imm == 0) 0 else cpu.r(sro.rm).* >> sro.shift_imm,
            .ASR => if (sro.shift_imm == 0) (if (cpu.r(sro.rm).* & 0x80000000 != 0) 0xFFFFFFFF else 0) else cpu.r(sro.rm).* >> sro.shift_imm, // FIXME: This is an "arithmetic shift right", don't know what that means yet. Does it update flags?
            .ROR => if (sro.shift_imm == 0) ((@as(u32, (if (cpu.cpsr.c) 1 else 0)) << 31) | (cpu.r(sro.rm).* >> 1)) else std.math.rotr(u32, cpu.r(sro.rm).*, sro.shift_imm),
        };
    }

    const base = cpu.r(inst.rn).*;

    const offset_addr = if (inst.u == 1) base + offset else base - offset;

    const addr = if (inst.p == 1) offset_addr else base;

    // When R15 is the source register (Rd) of a register store (STR) instruction, the stored value will be address of the instruction plus 12.
    // FIXME: I think it's +8 with the current implementation? Maybe?

    if (inst.l == 0) {
        if (inst.b == 1)
            cpu.write(u8, addr, @truncate(cpu.r(inst.rd).*))
        else
            cpu.write(u32, addr, cpu.r(inst.rd).*);
    } else {
        cpu.r(inst.rd).* = if (inst.b == 1) cpu.read(u8, addr) else cpu.read(u32, addr);
        if (inst.rd == 15) cpu.reset_pipeline();
    }

    if (inst.w == 1) cpu.r(inst.rn).* = offset_addr;
}

fn handle_single_data_swap(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.SingleDataSwapInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented single data swap");
}

fn handle_multiply(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MultiplyInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented multiply");
}

fn handle_multiply_long(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MultiplyLongInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented multiply long");
}

fn handle_halfword_data_transfer_register_offset(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.HalfwordDataTransferRegisterOffsetInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented halfword data transfer register offset");
}

fn handle_halfword_data_transfer_immediate_offset(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.HalfwordDataTransferImmediateOffsetInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented halfword data transfer immediate offset");
}

fn handle_coprocessor_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.CoprocessorDataTransferInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented coprocessor data transfer");
}

fn handle_coprocessor_data_operation(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.CoprocessorDataOperationInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented coprocessor data operation");
}

fn handle_coprocessor_register_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.CoprocessorRegisterTransferInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    @panic("Unimplemented coprocessor register transfer");
}

// Move PSR to general-purpose register
fn handle_mrs(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MRSInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    if (inst.r == 1) {
        cpu.r(inst.rd).* = @bitCast(cpu.spsr().*);
    } else {
        cpu.r(inst.rd).* = @bitCast(cpu.cpsr);
    }
}

// Move to Status Register
fn handle_msr(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MSRInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    const UnallocMask: u32 = 0x06F0FC00;
    const UserMask: u32 = 0xF80F0200;
    const PrivMask: u32 = 0x000001DF;
    const StateMask: u32 = 0x01000020;

    const operand = if (inst.i == 1) immediate_shifter_operand(inst.source_operand) else cpu.r(@truncate(inst.source_operand)).*;

    std.debug.assert(operand & UnallocMask == 0);
    const byte_mask: u32 = (if (inst.field_mask.c == 1) @as(u32, 0x000000FF) else 0x00000000) |
        (if (inst.field_mask.x == 1) @as(u32, 0x0000FF00) else 0x00000000) |
        (if (inst.field_mask.s == 1) @as(u32, 0x00FF0000) else 0x00000000) |
        (if (inst.field_mask.f == 1) @as(u32, 0xFF000000) else 0x00000000);
    if (inst.r == 0) {
        std.debug.assert(!cpu.in_a_privileged_mode() or operand & StateMask == 0);
        const mask = byte_mask & if (cpu.in_a_privileged_mode()) (UserMask | PrivMask) else UserMask;
        cpu.cpsr = @bitCast((@as(u32, @bitCast(cpu.cpsr)) & ~mask) | (operand & mask));
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
    const r: u32 = @bitCast(@as(i32, @bitCast(op1)) + @as(i32, @bitCast(op2)));
    return op1 & 0x80000000 == op2 & 0x80000000 and op1 & 0x80000000 != r & 0x80000000;
}
inline fn overflow_from_sub(op1: u32, op2: u32) bool {
    const r: u32 = @bitCast(@as(i32, @bitCast(op1)) - @as(i32, @bitCast(op2)));
    return op1 & 0x80000000 != op2 & 0x80000000 and op1 & 0x80000000 != r & 0x80000000;
}

// Returns 1 if the addition specified as its parameter caused a carry (true result is bigger than 232−1, where
// the operands are treated as unsigned integers), and returns 0 in all other cases. This delivers further
// information about an addition which occurred earlier in the pseudo-code. The addition is not repeated.
fn carry_from(op1: u32, op2: u32) bool {
    return @as(u64, op1) + @as(u64, op2) > 0xFFFFFFFF;
}

// Returns 1 if the subtraction specified as its parameter caused a borrow (the true result is less than 0, where
// the operands are treated as unsigned integers), and returns 0 in all other cases. This delivers further
// information about a subtraction which occurred earlier in the pseudo-code. The subtraction is not repeated.
fn borrow_from(op1: u32, op2: u32) bool {
    return op2 > op1;
}

fn handle_data_processing(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.DataProcessingInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    // TODO: See "Writing to R15"

    const op1 = cpu.r(inst.rn).*;

    const shifter_result = if (inst.i == 0)
        operand_2_register(cpu, inst.operand2)
    else
        operand_2_immediate(cpu, inst.operand2);

    var op2 = shifter_result.shifter_operand;

    // TODO: Yes, this can and must be refactored.
    switch (inst.opcode) {
        .AND => {
            cpu.r(inst.rd).* = op1 & op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .EOR => {
            cpu.r(inst.rd).* = op1 ^ op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .SUB => {
            cpu.r(inst.rd).* = op1 -% op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = !borrow_from(op1, op2);
                cpu.cpsr.v = overflow_from_sub(op1, op2);
            }
        },
        .RSB => {
            cpu.r(inst.rd).* = op2 -% op1;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = !borrow_from(op2, op1);
                cpu.cpsr.v = overflow_from_sub(op2, op1);
            }
        },
        .ADD => {
            cpu.r(inst.rd).* = op1 +% op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = carry_from(op1, op2);
                cpu.cpsr.v = overflow_from_add(op1, op2);
            }
        },
        .ADC => {
            op2 += if (cpu.cpsr.c) 1 else 0; // FIXME: What if it overflows here?
            cpu.r(inst.rd).* = op1 +% op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = carry_from(op1, op2);
                cpu.cpsr.v = overflow_from_add(op1, op2);
            }
        },
        .SBC => {
            op2 += if (cpu.cpsr.c) 1 else 0; // FIXME: What if it overflows here?
            cpu.r(inst.rd).* = op1 -% op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = !borrow_from(op1, op2);
                cpu.cpsr.v = overflow_from_sub(op1, op2);
            }
        },
        .RSC => {
            const op1_c = op1 + if (cpu.cpsr.c) @as(u32, 1) else 0; // FIXME: What if it overflows here?
            cpu.r(inst.rd).* = op2 -% op1_c;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = !borrow_from(op2, op1_c);
                cpu.cpsr.v = overflow_from_sub(op2, op1_c);
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
            cpu.r(inst.rd).* = op1 | op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .MOV => {
            cpu.r(inst.rd).* = op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .BIC => {
            cpu.r(inst.rd).* = op1 & ~op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
                cpu.cpsr.c = shifter_result.shifter_carry_out;
                // V Unaffected
            }
        },
        .MVN => {
            cpu.r(inst.rd).* = ~op2;
            if (inst.s == 1 and inst.rd == 15) {
                cpu.cpsr = cpu.spsr().*;
            } else if (inst.s == 1) {
                cpu.cpsr.n = n_flag(cpu.r(inst.rd).*);
                cpu.cpsr.z = cpu.r(inst.rd).* == 0;
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
    arm7_interpreter_log.err("Invalid Instruction @{X:0>8}: {X:0>8} {b:0>32}", .{ cpu.pc().* - 8, instruction, instruction });
}

const barrel_shifter_result = struct { shifter_operand: u32, shifter_carry_out: bool };

pub inline fn operand_2_register(cpu: *arm7.ARM7, operand2: u12) barrel_shifter_result {
    const shift_imm = (operand2 >> 7) & 0b11111;
    if (shift_imm == 0) {
        return .{
            .shifter_operand = cpu.r(@truncate(operand2 & 0xF)).*,
            .shifter_carry_out = cpu.cpsr.c,
        };
    } else {
        return .{
            .shifter_operand = cpu.r(@truncate(operand2 & 0xF)).* << @intCast(shift_imm),
            .shifter_carry_out = (cpu.r(@truncate(operand2 & 0xF)).* << @intCast(shift_imm - 1)) >> 31 == 1,
        };
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
