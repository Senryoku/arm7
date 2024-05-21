const std = @import("std");

const arm7 = @import("arm7.zig");

pub fn execute(cpu: *arm7.ARM7, instruction: u16) void {
    _ = cpu;
    std.debug.print("Thumb Instruction 0x{X:0>4}\n", .{instruction});
    switch (instruction >> 8) {
        0b0000_0000...0b0000_0111 => {
            // LSL
        },
        0b0000_1000...0b0000_1111 => {
            // LSR
        },
        0b0001_0000...0b0001_0111 => {
            // ASR
        },
        0b0001_1000...0b0001_1111 => {
            // Add/Substract
        },
        0b0010_0000...0b0011_1111 => {
            // Move/compare/add/subtract immediate
        },
        0b0100_0000...0b0100_0011 => {
            // ALU operations
        },
        0b0100_0100...0b0100_0111 => {
            // Hi register operations/branch exchange
        },
        0b0100_1000...0b0100_1111 => {
            // PC-relative load
        },
        else => {
            std.debug.print("Unknown Thumb Instruction 0x{X:0>4}\n", .{instruction});
            @panic("Unknown Thumb Instruction");
        },
    }
}

const MoveShiftedRegisterInstruction = packed struct(u16) {
    rd: u3 = 0,
    rs: u3 = 0,
    offset5: u5,
    op: u2,
    opcode: u3 = 0b000,
};

const AddSubstractInstruction = packed struct(u16) {
    rd: u3,
    rs: u3,
    rn: u3,
    op: u2,
    i: u1,
    opcode: u5 = 0b00011,
};

const OpImmediateInstruction = packed struct(u16) {
    offset8: u8,
    rd: u3,
    op: u2,
    opcode: u3 = 0b001,
};

const ALUInstruction = packed struct(u16) {
    rd: u3,
    rs: u3,
    op: u3,
    opcode: u6 = 0b010000,
};

const HighRegisterInstruction = packed struct(u16) {
    rd: u3,
    rs: u3,
    h2: u1,
    h1: u1,
    op: u2,
    opcode: u6 = 0b010001,
};

const PCRelativeLoadInstruction = packed struct(u16) {
    word8: u8,
    rd: u3,
    opcode: u5 = 0b01001,
};
