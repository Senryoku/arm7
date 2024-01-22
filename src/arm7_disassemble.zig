const std = @import("std");
const arm7 = @import("arm7.zig");

const arm7_interpreter = @import("arm7_interpreter.zig");

var disassemble_temp: [256]u8 = undefined;
var disassemble_addr_mode_temp: [256]u8 = undefined;
var disassemble_sro_temp: [256]u8 = undefined;

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

fn disassemble_reg_list(reg_list: u16) []const u8 {
    @memset(&disassemble_addr_mode_temp, 0);
    disassemble_addr_mode_temp[0] = '{';
    var cursor: usize = 1;
    for (0..16) |i| {
        if ((reg_list >> @intCast(i)) & 1 == 1) {
            if (cursor != 1) {
                disassemble_addr_mode_temp[cursor] = ',';
                cursor += 1;
            }
            const r = disassemble_register(@truncate(i));
            @memcpy(disassemble_addr_mode_temp[cursor .. cursor + r.len], r);
            cursor += r.len;
        }
    }
    disassemble_addr_mode_temp[cursor] = '}';
    return disassemble_addr_mode_temp[0 .. cursor + 1];
}

fn disassemble_condition(cond: arm7.Condition) []const u8 {
    return switch (cond) {
        .EQ => "eq",
        .NE => "ne",
        .CS => "cs",
        .CC => "cc",
        .MI => "mi",
        .PL => "pl",
        .VS => "vs",
        .VC => "vc",
        .HI => "hi",
        .LS => "ls",
        .GE => "ge",
        .LT => "lt",
        .GT => "gt",
        .LE => "le",
        .AL => "",
        else => return "und",
    };
}

fn disassemble_opcode(opcode: arm7.Opcode) []const u8 {
    return switch (opcode) {
        .AND => "and",
        .EOR => "eor",
        .SUB => "sub",
        .RSB => "rsb",
        .ADD => "add",
        .ADC => "adc",
        .SBC => "sbc",
        .RSC => "rsc",
        .TST => "tst",
        .TEQ => "teq",
        .CMP => "cmp",
        .CMN => "cmn",
        .ORR => "orr",
        .MOV => "mov",
        .BIC => "bic",
        .MVN => "mvn",
    };
}

fn dissasemble_sro(sro: arm7_interpreter.ScaledRegisterOffset) []const u8 {
    return (if (sro.register_specified == 1)
        std.fmt.bufPrint(&disassemble_sro_temp, "{s},{s} {s}", .{ disassemble_register(sro.rm), @tagName(sro.shift_type), disassemble_register(sro.shift_amount.reg.rs) })
    else if (sro.shift_amount.imm == 0)
        switch (sro.shift_type) {
            .LSL => std.fmt.bufPrint(&disassemble_sro_temp, "{s}", .{disassemble_register(sro.rm)}),
            .LSR => std.fmt.bufPrint(&disassemble_sro_temp, "{s},{s}#32", .{ disassemble_register(sro.rm), @tagName(sro.shift_type) }),
            .ASR => std.fmt.bufPrint(&disassemble_sro_temp, "{s},{s}#32", .{ disassemble_register(sro.rm), @tagName(sro.shift_type) }),
            .ROR => std.fmt.bufPrint(&disassemble_sro_temp, "{s},RRX", .{disassemble_register(sro.rm)}),
        }
    else
        std.fmt.bufPrint(&disassemble_sro_temp, "{s},{s}#{d}", .{ disassemble_register(sro.rm), @tagName(sro.shift_type), sro.shift_amount.imm })) catch unreachable;
}

fn disassemble_addr_mode_2(inst: arm7.SingleDataTransferInstruction) []const u8 {
    const sign = if (inst.u == 1) "" else "-";
    if (inst.i == 0) {
        if (inst.offset == 0) {
            return std.fmt.bufPrint(&disassemble_addr_mode_temp, "[{s}]", .{disassemble_register(inst.rn)}) catch unreachable;
        } else {
            return std.fmt.bufPrint(&disassemble_addr_mode_temp, "[{s}, #{s}0x{X}]", .{ disassemble_register(inst.rn), sign, inst.offset }) catch unreachable;
        }
    } else {
        const sro = dissasemble_sro(@bitCast(inst.offset));
        return std.fmt.bufPrint(&disassemble_addr_mode_temp, "[{s}, {s}{s}]", .{ disassemble_register(inst.rn), sign, sro }) catch unreachable;
    }
}

fn disassemble_branch_and_exchange(instruction: u32) []const u8 {
    _ = instruction;

    return "BranchAndExchange";
}

fn disassemble_block_data_transfer(instruction: u32) []const u8 {
    const inst: arm7.BlockDataTransferInstruction = @bitCast(instruction);
    const cond = disassemble_condition(inst.cond);
    return std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s} {s}{s}{s}", .{
        if (inst.l == 0) "stm" else "ldm",
        cond,
        switch ((@as(u2, inst.u) << 1) | inst.p) {
            0b00 => "da",
            0b01 => "db",
            0b10 => "ia",
            0b11 => "ib",
        },
        disassemble_register(inst.rn),
        if (inst.w == 1) "!" else "",
        disassemble_reg_list(inst.reg_list),
    }) catch unreachable;
}

fn disassemble_branch(instruction: u32) []const u8 {
    const inst: arm7.BranchInstruction = @bitCast(instruction);

    const offset = (arm7.sign_extend(@TypeOf(inst.offset), inst.offset) << 2) +% 8; // FIXME: This is PC relative, and signed.
    const cond = disassemble_condition(inst.cond);

    return std.fmt.bufPrint(&disassemble_temp, "b{s}{s} 0x{x}", .{ if (inst.l == 0) "" else "l", cond, offset }) catch unreachable;
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
    const expr = disassemble_addr_mode_2(instr);

    return std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s}{s} {s},{s}{s}", .{
        if (instr.l == 1) "ltr" else "str",
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
    const instr: arm7.MultiplyInstruction = @bitCast(instruction);
    const cond = disassemble_condition(instr.cond);
    return std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s} {s}, {s}, {s}, {s}", .{
        if (instr.a == 1) "mla" else "mul",
        cond,
        if (instr.s == 1) "s" else "",
        disassemble_register(instr.rd),
        disassemble_register(instr.rm),
        disassemble_register(instr.rs),
        if (instr.a == 1) disassemble_register(instr.rn) else "mul",
    }) catch unreachable;
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

fn disassemble_mrs(instruction: u32) []const u8 {
    const inst: arm7.MRSInstruction = @bitCast(instruction);
    const cond = disassemble_condition(inst.cond);
    return std.fmt.bufPrint(&disassemble_temp, "mrs{s} {s}, {s}", .{ cond, disassemble_register(inst.rd), if (inst.r == 1) "SPSR" else "CPSR" }) catch unreachable;
}

fn disassemble_msr(instruction: u32) []const u8 {
    const inst: arm7.MSRInstruction = @bitCast(instruction);
    const cond = disassemble_condition(inst.cond);
    const immediate = std.fmt.bufPrint(&disassemble_addr_mode_temp, "#{x}", .{arm7_interpreter.immediate_shifter_operand(inst.source_operand)}) catch unreachable;
    return std.fmt.bufPrint(&disassemble_temp, "msr{s} {s}_{s}{s}{s}{s}, {s}", .{
        cond,
        if (inst.r == 1) "SPSR" else "CPSR",
        if (inst.field_mask.c == 1) "c" else "",
        if (inst.field_mask.x == 1) "x" else "",
        if (inst.field_mask.s == 1) "s" else "",
        if (inst.field_mask.f == 1) "f" else "",
        if (inst.i == 1) immediate else disassemble_register(@truncate(inst.source_operand)),
    }) catch unreachable;
}

fn disassemble_data_processing(instruction: u32) []const u8 {
    const instr: arm7.DataProcessingInstruction = @bitCast(instruction);

    const op2 = if (instr.i == 1)
        std.fmt.bufPrint(&disassemble_sro_temp, "#{X:0>3}", .{arm7_interpreter.immediate_shifter_operand(@bitCast(instr.operand2))}) catch unreachable
    else
        dissasemble_sro(@bitCast(instr.operand2));

    return switch (instr.opcode) {
        .MOV, .MVN => std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s} {s},{s}", .{
            disassemble_opcode(instr.opcode),
            disassemble_condition(instr.cond),
            if (instr.s == 1) "s" else "",
            disassemble_register(instr.rd),
            op2,
        }),
        .CMP, .CMN, .TEQ, .TST => std.fmt.bufPrint(&disassemble_temp, "{s}{s} {s},{s}", .{
            disassemble_opcode(instr.opcode),
            disassemble_condition(instr.cond),
            disassemble_register(instr.rn),
            op2,
        }),
        else => std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s} {s},{s},{s}", .{
            disassemble_opcode(instr.opcode),
            disassemble_condition(instr.cond),
            if (instr.s == 1) "s" else "",
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
    disassemble_mrs,
    disassemble_msr,
    disassemble_data_processing,

    disassemble_invalid,
};
