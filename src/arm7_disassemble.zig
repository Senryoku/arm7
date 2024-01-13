const std = @import("std");
const arm7 = @import("arm7.zig");

const arm7_interpreter = @import("arm7_interpreter.zig");

var disassemble_temp: [256]u8 = undefined;

fn disassemble_register(reg: u4) []const u8 {
    return switch (reg) {
        0b0000 => "r0",
        0b0001 => "r1",
        0b0010 => "r2",
        0b0011 => "r3",
        0b0100 => "r4",
        0b0101 => "r5",
        0b0110 => "r6",
        0b0111 => "r7",
        0b1000 => "r8",
        0b1001 => "r9",
        0b1010 => "r10",
        0b1011 => "r11",
        0b1100 => "r12",
        0b1101 => "sp",
        0b1110 => "lr",
        0b1111 => "pc",
    };
}

fn disassemble_condition(cond: u4) []const u8 {
    switch (cond) {
        0b0000 => return "eq",
        0b0001 => return "ne",
        0b0010 => return "cs",
        0b0011 => return "cc",
        0b0100 => return "mi",
        0b0101 => return "pl",
        0b0110 => return "vs",
        0b0111 => return "vc",
        0b1000 => return "hi",
        0b1001 => return "ls",
        0b1010 => return "ge",
        0b1011 => return "lt",
        0b1100 => return "gt",
        0b1101 => return "le",
        0b1110 => return "", // al / always
        else => return "und",
    }
}

fn disassemble_branch_and_exchange(instruction: u32) []const u8 {
    _ = instruction;

    return "BranchAndExchange";
}

fn disassemble_block_data_transfer(instruction: u32) []const u8 {
    _ = instruction;

    return "BlockDataTransfer";
}

fn disassemble_branch(instruction: u32) []const u8 {
    const inst: arm7.BranchInstruction = @bitCast(instruction);

    const offset = arm7.sign_extend_u26(@as(u26, inst.offset) << 2) + 8; // FIXME: This is PC relative.
    const cond = disassemble_condition(inst.cond);

    if (inst.l == 0) {
        return std.fmt.bufPrint(&disassemble_temp, "b{s} 0x{x}", .{ cond, offset }) catch unreachable;
    } else {
        return std.fmt.bufPrint(&disassemble_temp, "bl{s} 0x{x}", .{ cond, offset }) catch unreachable;
    }
}

fn disassemble_software_interrupt(instruction: u32) []const u8 {
    _ = instruction;

    return "SoftwareInterrupt";
}

fn disassemble_undefined(instruction: u32) []const u8 {
    _ = instruction;

    return "Undefined";
}

fn disassemble_single_data_transfer(instruction: u32) []const u8 {
    const instr: arm7.SingleDataTransferInstruction = @bitCast(instruction);

    const cond = disassemble_condition(instr.cond);
    const expr = "[TODO]"; // TODO

    return std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s}{s} {s},{s}{s}", .{
        if (instr.l == 0) "ltr" else "str",
        cond,
        if (instr.b == 1) "b" else "",
        if (instr.w == 1 and instr.p == 1) "t" else "", // FIXME: I don't how it works yet.
        disassemble_register(instr.rd),
        expr,
        if (instr.w == 1) "!" else "",
    }) catch unreachable;
}

fn disassemble_single_data_swap(instruction: u32) []const u8 {
    _ = instruction;

    return "SingleDataSwap";
}

fn disassemble_multiply(instruction: u32) []const u8 {
    _ = instruction;

    return "Multiply";
}

fn disassemble_multiply_long(instruction: u32) []const u8 {
    _ = instruction;

    return "MultiplyLong";
}

fn disassemble_halfword_data_transfer_register_offset(instruction: u32) []const u8 {
    _ = instruction;

    return "HalfwordDataTransferRegisterOffset";
}

fn disassemble_halfword_data_transfer_immediate_offset(instruction: u32) []const u8 {
    _ = instruction;

    return "HalfwordDataTransferImmediateOffset";
}

fn disassemble_coprocessor_data_transfer(instruction: u32) []const u8 {
    _ = instruction;

    return "CoprocessorDataTransfer";
}

fn disassemble_coprocessor_data_operation(instruction: u32) []const u8 {
    _ = instruction;

    return "CoprocessorDataOperation";
}

fn disassemble_coprocessor_register_transfer(instruction: u32) []const u8 {
    _ = instruction;

    return "CoprocessorRegisterTransfer";
}

fn disassemble_data_processing(instruction: u32) []const u8 {
    const instr: arm7.DataProcessingInstruction = @bitCast(instruction);

    var op2_buf: [32]u8 = undefined;
    const op2 = (if (instr.i == 0)
        std.fmt.bufPrint(&op2_buf, "R{d}{{,{d}}}", .{ instr.operand2 & 0xF, instr.operand2 >> 7 })
    else
        std.fmt.bufPrint(&op2_buf, "#{X:0>3}", .{arm7_interpreter.immediate_shifter_operand(instr.operand2)})) catch unreachable;

    return switch (instr.opcode) {
        .MOV, .MVN => std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s} {s},{s}", .{
            @tagName(instr.opcode),
            disassemble_condition(instr.cond),
            if (instr.s == 1) "S" else "",
            disassemble_register(instr.rd),
            op2,
        }),
        .CMP, .CMN, .TEQ, .TST => std.fmt.bufPrint(&disassemble_temp, "{s}{s} {s},{s}", .{
            @tagName(instr.opcode),
            disassemble_condition(instr.cond),
            disassemble_register(instr.rn),
            op2,
        }),
        else => std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s} {s},{s},{s}", .{
            @tagName(instr.opcode),
            disassemble_condition(instr.cond),
            if (instr.s == 1) "S" else "",
            disassemble_register(instr.rd),
            disassemble_register(instr.rn),
            op2,
        }),
    } catch unreachable;
}

fn disassemble_invalid(_: u32) []const u8 {
    return "Invalid";
}

pub const DisassembleTable = [_]*const fn (u32) []const u8{
    disassemble_branch_and_exchange,
    disassemble_block_data_transfer,
    disassemble_branch,
    disassemble_software_interrupt,
    disassemble_undefined,
    disassemble_single_data_transfer,
    disassemble_single_data_swap,
    disassemble_multiply,
    disassemble_multiply_long,
    disassemble_halfword_data_transfer_register_offset,
    disassemble_halfword_data_transfer_immediate_offset,
    disassemble_coprocessor_data_transfer,
    disassemble_coprocessor_data_operation,
    disassemble_coprocessor_register_transfer,
    disassemble_data_processing,

    disassemble_invalid,
};
