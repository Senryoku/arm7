const std = @import("std");
const testing = std.testing;

const RegisterMode = enum(u5) {
    User = 0b10000,
    FastInterrupt = 0b10001,
    Interrupt = 0b10010,
    Supervisor = 0b10011,
    Abort = 0b10111,
    Undefined = 0b11011,
    System = 0b11111,
};

const CPSR = packed struct(u32) {
    m: RegisterMode = .User,
    t: bool = false, // State bit
    f: bool = false, // FIQ Disable
    i: bool = false, // IRQ Disable
    _: u20 = 0, // Reserved
    v: bool = false, // Overflow
    c: bool = false, // Carry or borrow or extend
    z: bool = false, // Zero
    n: bool = false, // Negative or less than
};

const DataProcessingInstructionTags: u32 = 0b0000_0000_0000_0000_0000_0000_0000_0000;
const DataProcessingInstructionMask: u32 = 0b0000_1100_0000_0000_0000_0000_0000_0000;

const DataProcessingInstruction = packed struct(u32) {
    operand2: u12,
    rd: u4,
    rn: u4,
    s: u1,
    opcode: u4,
    i: u1,
    tag: u2,
    cond: u4,
};

const MultiplyInstructionTags: u32 = 0b0000_0000_0000_0000_0000_0000_1001_0000;
const MultiplyInstructionMask: u32 = 0b0000_1111_1100_0000_0000_0000_1111_0000;

const MultiplyInstruction = packed struct(u32) {
    rm: u4,
    _tag2: u4,
    rs: u4,
    rn: u4,
    rd: u4,
    s: u1,
    a: u1,
    _tag: u6,
    cond: u4,
};

const MultiplyLongInstructionTags: u32 = 0b0000_0000_1000_0000_0000_0000_1001_0000;
const MultiplyLongInstructionMask: u32 = 0b0000_1111_1000_0000_0000_0000_1111_0000;

const MultiplyLongInstruction = packed struct(u32) {
    rm: u4,
    _tag2: u4,
    rn: u4,
    rdlo: u4,
    rdhi: u4,
    s: u1,
    a: u1,
    u: u1,
    _tag: u5,
    cond: u4,
};

const SingleDataSwapInstructionTags: u32 = 0b0000_0001_0000_0000_0000_0000_1001_0000;
const SingleDataSwapInstructionMask: u32 = 0b0000_1111_1011_0000_0000_1111_1111_0000;

const SingleDataSwapInstruction = packed struct(u32) {
    rm: u4,
    _tag3: u8,
    rd: u4,
    rn: u4,
    _tag2: u2,
    u: u1,
    _tag: u5,
    cond: u4,
};

const BranchAndExchangeInstructionTags: u32 = 0b0000_0001_0010_1111_1111_1111_0001_0000;
const BranchAndExchangeInstructionMask: u32 = 0b0000_1111_1111_1111_1111_1111_1111_0000;

const BranchAndExchangeInstruction = packed struct(u32) {
    rm: u4,
    _tag: u24,
    cond: u4,
};

const HalfwordDataTransferRegisterOffsetInstructionTags: u32 = 0b0000_0000_0000_0000_0000_0000_1001_0000;
const HalfwordDataTransferRegisterOffsetInstructionMask: u32 = 0b0000_1110_0100_0000_0000_1111_1001_0000;

const HalfwordDataTransferRegisterOffsetInstruction = packed struct(u32) {
    rm: u4,
    _tag4: u1,
    h: u1,
    s: u1,
    _tag3: u5,
    rd: u4,
    rn: u4,
    l: u1,
    w: u1,
    _tag2: u1,
    u: u1,
    p: u1,
    _tag: u3,
    cond: u4,
};

const HalfwordDataTransferImmediateOffsetInstructionTags: u32 = 0b0000_0000_0100_0000_0000_0000_1001_0000;
const HalfwordDataTransferImmediateOffsetInstructionMask: u32 = 0b0000_1110_0100_0000_0000_0000_1001_0000;

const HalfwordDataTransferImmediateOffsetInstruction = packed struct(u32) {
    offset2: u4,
    _tag4: u1,
    h: u1,
    s: u1,
    _tag3: u1,
    offset: u4,
    rd: u4,
    rn: u4,
    l: u1,
    w: u1,
    _tag2: u1,
    u: u1,
    p: u1,
    _tag: u3,
    cond: u4,
};

const SingleDataTransferInstructionTags: u32 = 0b0000_0100_0000_0000_0000_0000_0000_0000;
const SingleDataTransferInstructionMask: u32 = 0b0000_1100_0000_0000_0000_0000_0000_0000;

const SingleDataTransferInstruction = packed struct(u32) {
    offset: u12,
    rd: u4,
    rn: u4,
    l: u1,
    w: u1,
    b: u1,
    u: u1,
    p: u1,
    i: u1,
    _tag: u2,
    cond: u4,
};

const UndefinedInstructionTags: u32 = 0b0000_0110_0000_0000_0000_0000_0001_0000;
const UndefinedInstructionMask: u32 = 0b0000_1110_0000_0000_0000_0000_0001_0000;

const UndefinedInstruction = packed struct(u32) {
    _: u28,
    cond: u4,
};

const BlockDataTransferInstructionTags: u32 = 0b0000_1000_0000_0000_0000_0000_0000_0000;
const BlockDataTransferInstructionMask: u32 = 0b0000_1110_0000_0000_0000_0000_0000_0000;

const BlockDataTransferInstruction = packed struct(u32) {
    reg_list: u16,
    rn: u4,
    l: u1,
    w: u1,
    s: u1,
    u: u1,
    p: u1,
    _tag: u3,
    cond: u4,
};

const BranchInstructionTags: u32 = 0b0000_1010_0000_0000_0000_0000_0000_0000;
const BranchInstructionMask: u32 = 0b0000_1110_0000_0000_0000_0000_0000_0000;

const BranchInstruction = packed struct(u32) {
    offset: u24,
    l: u1,
    _tag: u3,
    cond: u4,
};

const CoprocessorDataTransferInstructionTags: u32 = 0b0000_1100_0000_0000_0000_0000_0000_0000;
const CoprocessorDataTransferInstructionMask: u32 = 0b0000_1110_0000_0000_0000_0000_0000_0000;

const CoprocessorDataTransferInstruction = packed struct(u32) {
    crm: u4,
    _tag2: u1,
    cp: u3,
    cpn: u4,
    crd: u4,
    rn: u4,
    l: u1,
    w: u1,
    n: u1,
    u: u1,
    p: u1,
    _tag: u3,
    cond: u4,
};

const CoprocessorDataOperationInstructionTags: u32 = 0b0000_1110_0000_0000_0000_0000_0000_0000;
const CoprocessorDataOperationInstructionMask: u32 = 0b0000_1111_0000_0000_0000_0000_0001_0000;

const CoprocessorDataOperationInstruction = packed struct(u32) {
    crm: u4,
    _tag2: u1,
    cp: u3,
    cpn: u4,
    crd: u4,
    crn: u4,
    cp_opc: u4,
    _tag: u4,
    cond: u4,
};

const CoprocessorRegisterTransferInstructionTags: u32 = 0b0000_1110_0000_0000_0000_0000_0001_0000;
const CoprocessorRegisterTransferInstructionMask: u32 = 0b0000_1111_0000_0000_0000_0000_0001_0000;

const CoprocessorRegisterTransferInstruction = packed struct(u32) {
    crm: u4,
    _tag2: u1,
    cp: u3,
    cpn: u4,
    crd: u4,
    crn: u4,
    l: u1,
    cp_opc: u3,
    _tag: u4,
    cond: u4,
};

const SoftwareInterruptInstructionTags: u32 = 0b0000_1111_0000_0000_0000_0000_0000_0000;
const SoftwareInterruptInstructionMask: u32 = 0b0000_1111_0000_0000_0000_0000_0000_0000;

const SoftwareInterruptInstruction = packed struct(u32) {
    _: u28,
    cond: u4,
};

const Instruction = packed union {
    DataProcessing: DataProcessingInstruction,
    Multiply: MultiplyInstruction,
    MultiplyLong: MultiplyLongInstruction,
    SingleDataSwap: SingleDataSwapInstruction,
    BranchAndExchange: BranchAndExchangeInstruction,
    HalfwordDataTransferRegisterOffset: HalfwordDataTransferRegisterOffsetInstruction,
    HalfwordDataTransferImmediateOffset: HalfwordDataTransferImmediateOffsetInstruction,
    SingleDataTransfer: SingleDataTransferInstruction,
    Undefined: UndefinedInstruction,
    Branch: BranchInstruction,
    BlockDataTransfer: BlockDataTransferInstruction,
    CoprocessorDataTransfer: CoprocessorDataTransferInstruction,
    CoprocessorDataOperation: CoprocessorDataOperationInstruction,
    CoprocessorRegisterTransfer: CoprocessorRegisterTransferInstruction,
    SoftwareInterrupt: SoftwareInterruptInstruction,
};

const InstructionMasks = [_]u32{
    BranchAndExchangeInstructionMask,
    BlockDataTransferInstructionMask,
    BranchInstructionMask,
    SoftwareInterruptInstructionMask,
    UndefinedInstructionMask,
    SingleDataTransferInstructionMask,
    SingleDataSwapInstructionMask,
    MultiplyInstructionMask,
    MultiplyLongInstructionMask,
    HalfwordDataTransferRegisterOffsetInstructionMask,
    HalfwordDataTransferImmediateOffsetInstructionMask,
    CoprocessorDataTransferInstructionMask,
    CoprocessorDataOperationInstructionMask,
    CoprocessorRegisterTransferInstructionMask,
    DataProcessingInstructionMask,
};

const InstructionTags = [_]u32{
    BranchAndExchangeInstructionTags,
    BlockDataTransferInstructionTags,
    BranchInstructionTags,
    SoftwareInterruptInstructionTags,
    UndefinedInstructionTags,
    SingleDataTransferInstructionTags,
    SingleDataSwapInstructionTags,
    MultiplyInstructionTags,
    MultiplyLongInstructionTags,
    HalfwordDataTransferRegisterOffsetInstructionTags,
    HalfwordDataTransferImmediateOffsetInstructionTags,
    CoprocessorDataTransferInstructionTags,
    CoprocessorDataOperationInstructionTags,
    CoprocessorRegisterTransferInstructionTags,
    DataProcessingInstructionTags,
};

var disassemble_temp: [256]u8 = undefined;

fn sign_extend_u26(val: u26) u32 {
    if (val & 0x2000000 != 0) {
        return @as(u32, val) | 0xFC000000;
    } else {
        return val;
    }
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
    const inst: BranchInstruction = @bitCast(instruction);

    const offset = sign_extend_u26(@as(u26, inst.offset) << 2) + 8; // NOTE: This is PC relative.
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
    const instr: SingleDataTransferInstruction = @bitCast(instruction);

    const cond = disassemble_condition(instr.cond);
    const expr = "[TODO]"; // TODO

    return std.fmt.bufPrint(&disassemble_temp, "{s}{s}{s}{s} r{d},{s}{s}", .{
        if (instr.l == 0) "ltr" else "str",
        cond,
        if (instr.b == 1) "b" else "",
        if (instr.w == 1 and instr.p == 1) "t" else "", // FIXME: I don't how it works yet.
        instr.rd,
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
    _ = instruction;

    return "DataProcessing";
}

fn disassemble_invalid(_: u32) []const u8 {
    return "Invalid";
}

const disassemble_table = [_]*const fn (u32) []const u8{
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

var JumpTable: [0x1000]u8 = undefined;

// Generic ARM7 Core
// T: 16bit Thumb
// D: JTAG Debug
// M: Fast Multiplier
// I: Enhanced ICE
//
// AFAIK:
//   The GBA cpu is a sort of ARM7TDMI
//   The arm code of the AICA, the Dreamcast sound chip, is a ARM7DI
pub const ARM7 = struct {
    memory: []u8,

    r: [16]u32 = undefined,
    r_irq: [7]u32 = undefined,
    r_svc: [2]u32 = undefined,
    r_fiq: [2]u32 = undefined,
    r_abt: [2]u32 = undefined,
    r_und: [2]u32 = undefined,
    spsr: u32 = undefined,

    fn init_jump_table() void {
        for (0..0x1000) |i| {
            const tag: u12 = @truncate(i);
            JumpTable[tag] = 255;
            for (0..InstructionTags.len) |t| {
                if (tag & get_instr_tag(InstructionMasks[t]) == get_instr_tag(InstructionTags[t])) {
                    JumpTable[tag] = @intCast(t);
                    // Collision are expected, decoding order matters.
                    break;
                }
            }
            if (JumpTable[tag] == 255) {
                std.debug.print("  {d: >6} {b:0>12}\n", .{ i, tag });
                // @panic("Instruction tag not found");
                JumpTable[tag] = disassemble_table.len - 1;
            }
        }
    }

    pub fn init(memory: []u8) @This() {
        init_jump_table();
        return .{ .memory = memory };
    }

    pub inline fn pc(self: *@This()) *u32 {
        return &self.r[15];
    }

    fn get_instr_tag(instruction: u32) u12 {
        return @truncate(((instruction >> 16) & 0xFF0) | ((instruction >> 4) & 0xF));
    }

    pub fn fetch(self: *@This()) Instruction {
        const r = @as(*const u32, @alignCast(@ptrCast(&self.memory[self.pc().*]))).*;
        self.pc().* += 4;
        return @bitCast(r);
    }

    fn decode() void {}

    pub fn disassemble(instr: Instruction) []const u8 {
        return disassemble_table[JumpTable[@This().get_instr_tag(@bitCast(instr))]](@bitCast(instr));
    }
};
