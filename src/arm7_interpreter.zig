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
    handle_data_processing,

    handle_invalid,
};

inline fn handle_condition(cpu: *arm7.ARM7, cond: u4) bool {
    _ = cond;
    _ = cpu;

    return true;
}

fn handle_branch_and_exchange(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    _ = instruction;
}

fn handle_block_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    _ = cpu;
    _ = instruction;
}

fn handle_branch(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.BranchInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;

    const offset = arm7.sign_extend_u26(@as(u26, inst.offset) << 2) + 8;

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
}

fn handle_undefined(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.UndefinedInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_single_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.SingleDataTransferInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_single_data_swap(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.SingleDataSwapInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_multiply(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MultiplyInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_multiply_long(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.MultiplyLongInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_halfword_data_transfer_register_offset(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.HalfwordDataTransferRegisterOffsetInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_halfword_data_transfer_immediate_offset(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.HalfwordDataTransferImmediateOffsetInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_coprocessor_data_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.CoprocessorDataTransferInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_coprocessor_data_operation(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.CoprocessorDataOperationInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
}

fn handle_coprocessor_register_transfer(cpu: *arm7.ARM7, instruction: u32) void {
    const inst: arm7.CoprocessorRegisterTransferInstruction = @bitCast(instruction);
    if (!handle_condition(cpu, inst.cond))
        return;
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

    const op2 = shifter_result.shifter_operand;

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
        .CMP => {
            const r = op1 -% op2;
            std.debug.assert(inst.s == 1);
            cpu.cpsr.n = n_flag(r);
            cpu.cpsr.z = r == 0;
            cpu.cpsr.c = !borrow_from(op1, op2);
            cpu.cpsr.v = overflow_from_sub(op1, op2);
        },
        else => {
            arm7_interpreter_log.warn("DataProcessing: Unhandled opcode {any}", .{inst.opcode});
        },
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
    return std.math.rotr(u32, arm7.zero_extend(operand2 & 0xFF), rotate_imm);
}

pub inline fn operand_2_immediate(cpu: *arm7.ARM7, operand2: u12) barrel_shifter_result {
    const shifter_operand = immediate_shifter_operand(operand2);
    return .{
        .shifter_operand = shifter_operand,
        .shifter_carry_out = if (operand2 >> 8 == 0) cpu.cpsr.c else (shifter_operand >> 31) == 1,
    };
}
