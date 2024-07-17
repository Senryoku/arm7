const std = @import("std");
const testing = std.testing;

pub const interpreter = @import("arm7_interpreter.zig");
pub const dissasemble = @import("arm7_disassemble.zig");

const arm7_log = std.log.scoped(.arm7);

pub const RegisterMode = enum(u5) {
    User = 0b10000,
    FastInterrupt = 0b10001,
    Interrupt = 0b10010,
    Supervisor = 0b10011,
    Abort = 0b10111,
    Undefined = 0b11011,
    System = 0b11111,
};

pub const CPSR = packed struct(u32) {
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

pub const Condition = enum(u4) {
    EQ = 0b0000,
    NE = 0b0001,
    CS = 0b0010,
    CC = 0b0011,
    MI = 0b0100,
    PL = 0b0101,
    VS = 0b0110,
    VC = 0b0111,
    HI = 0b1000,
    LS = 0b1001,
    GE = 0b1010,
    LT = 0b1011,
    GT = 0b1100,
    LE = 0b1101,
    AL = 0b1110,
    Invalid = 0b1111,
};

pub const Opcode = enum(u4) {
    AND = 0b0000, // operand1 AND operand2
    EOR = 0b0001, // operand1 EOR operand2
    SUB = 0b0010, // operand1 - operand2
    RSB = 0b0011, // operand2 - operand1
    ADD = 0b0100, // operand1 + operand2
    ADC = 0b0101, // operand1 + operand2 + carry
    SBC = 0b0110, // operand1 - operand2 + carry - 1
    RSC = 0b0111, // operand2 - operand1 + carry - 1
    TST = 0b1000, // as AND, but result is not written
    TEQ = 0b1001, // as EOR, but result is not written
    CMP = 0b1010, // as SUB, but result is not written
    CMN = 0b1011, // as ADD, but result is not written
    ORR = 0b1100, // operand1 OR operand2
    MOV = 0b1101, // operand2(operand1 is ignored)
    BIC = 0b1110, // operand1 AND NOT operand2(Bit clear)
    MVN = 0b1111, // NOT operand2(operand1 is ignored
};

pub const DataProcessingInstruction = packed struct(u32) {
    operand2: u12,
    rd: u4,
    rn: u4,
    s: u1,
    opcode: Opcode,
    i: u1,
    tag: u2,
    cond: Condition,
};

pub const MultiplyInstructionTags: u32 = 0b0000_0000_0000_0000_0000_0000_1001_0000;
pub const MultiplyInstructionMask: u32 = 0b0000_1111_1100_0000_0000_0000_1111_0000;

pub const MultiplyInstruction = packed struct(u32) {
    rm: u4,
    _tag2: u4,
    rs: u4,
    rn: u4,
    rd: u4,
    s: u1,
    a: u1,
    _tag: u6,
    cond: Condition,
};

pub const MultiplyLongInstructionTags: u32 = 0b0000_0000_1000_0000_0000_0000_1001_0000;
pub const MultiplyLongInstructionMask: u32 = 0b0000_1111_1000_0000_0000_0000_1111_0000;

pub const MultiplyLongInstruction = packed struct(u32) {
    rm: u4,
    _tag2: u4,
    rn: u4,
    rdlo: u4,
    rdhi: u4,
    s: u1,
    a: u1,
    u: u1,
    _tag: u5,
    cond: Condition,
};

pub const SingleDataSwapInstructionTags: u32 = 0b0000_0001_0000_0000_0000_0000_1001_0000;
pub const SingleDataSwapInstructionMask: u32 = 0b0000_1111_1011_0000_0000_1111_1111_0000;

pub const SingleDataSwapInstruction = packed struct(u32) {
    rm: u4,
    _tag3: u8,
    rd: u4,
    rn: u4,
    _tag2: u2,
    b: u1,
    _tag: u5,
    cond: Condition,
};

pub const BranchAndExchangeInstructionTags: u32 = 0b0000_0001_0010_1111_1111_1111_0001_0000;
pub const BranchAndExchangeInstructionMask: u32 = 0b0000_1111_1111_1111_1111_1111_1111_0000;

pub const BranchAndExchangeInstruction = packed struct(u32) {
    rm: u4,
    _tag: u24,
    cond: Condition,
};

pub const HalfwordDataTransferRegisterOffsetInstructionTags: u32 = 0b0000_0000_0000_0000_0000_0000_1001_0000;
pub const HalfwordDataTransferRegisterOffsetInstructionMask: u32 = 0b0000_1110_0100_0000_0000_1111_1001_0000;

pub const HalfwordDataTransferRegisterOffsetInstruction = packed struct(u32) {
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
    cond: Condition,
};

pub const HalfwordDataTransferImmediateOffsetInstructionTags: u32 = 0b0000_0000_0100_0000_0000_0000_1001_0000;
pub const HalfwordDataTransferImmediateOffsetInstructionMask: u32 = 0b0000_1110_0100_0000_0000_0000_1001_0000;

pub const HalfwordDataTransferImmediateOffsetInstruction = packed struct(u32) {
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
    cond: Condition,
};

pub const SingleDataTransferInstructionTags: u32 = 0b0000_0100_0000_0000_0000_0000_0000_0000;
pub const SingleDataTransferInstructionMask: u32 = 0b0000_1100_0000_0000_0000_0000_0000_0000;

pub const SingleDataTransferInstruction = packed struct(u32) {
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
    cond: Condition,
};

pub const UndefinedInstructionTags: u32 = 0b0000_0110_0000_0000_0000_0000_0001_0000;
pub const UndefinedInstructionMask: u32 = 0b0000_1110_0000_0000_0000_0000_0001_0000;

pub const UndefinedInstruction = packed struct(u32) {
    _: u28,
    cond: Condition,
};

pub const BlockDataTransferInstructionTags: u32 = 0b0000_1000_0000_0000_0000_0000_0000_0000;
pub const BlockDataTransferInstructionMask: u32 = 0b0000_1110_0000_0000_0000_0000_0000_0000;

pub const BlockDataTransferInstruction = packed struct(u32) {
    reg_list: u16,
    rn: u4,
    l: u1,
    w: u1,
    s: u1,
    u: u1,
    p: u1,
    _tag: u3,
    cond: Condition,

    pub inline fn reg(self: @This(), idx: anytype) bool {
        return self.reg_list & (@as(u16, 1) << @intCast(idx)) != 0;
    }
};

pub const BranchInstructionTags: u32 = 0b0000_1010_0000_0000_0000_0000_0000_0000;
pub const BranchInstructionMask: u32 = 0b0000_1110_0000_0000_0000_0000_0000_0000;

pub const BranchInstruction = packed struct(u32) {
    offset: u24,
    l: u1,
    _tag: u3,
    cond: Condition,
};

pub const CoprocessorDataTransferInstructionTags: u32 = 0b0000_1100_0000_0000_0000_0000_0000_0000;
pub const CoprocessorDataTransferInstructionMask: u32 = 0b0000_1110_0000_0000_0000_0000_0000_0000;

pub const CoprocessorDataTransferInstruction = packed struct(u32) {
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
    cond: Condition,
};

pub const CoprocessorDataOperationInstructionTags: u32 = 0b0000_1110_0000_0000_0000_0000_0000_0000;
pub const CoprocessorDataOperationInstructionMask: u32 = 0b0000_1111_0000_0000_0000_0000_0001_0000;

pub const CoprocessorDataOperationInstruction = packed struct(u32) {
    crm: u4,
    _tag2: u1,
    cp: u3,
    cpn: u4,
    crd: u4,
    crn: u4,
    cp_opc: u4,
    _tag: u4,
    cond: Condition,
};

pub const CoprocessorRegisterTransferInstructionTags: u32 = 0b0000_1110_0000_0000_0000_0000_0001_0000;
pub const CoprocessorRegisterTransferInstructionMask: u32 = 0b0000_1111_0000_0000_0000_0000_0001_0000;

pub const CoprocessorRegisterTransferInstruction = packed struct(u32) {
    crm: u4,
    _tag1: u1,
    cp: u3,
    cpn: u4,
    crd: u4,
    crn: u4,
    l: u1,
    cp_opc: u3,
    _tag2: u4,
    cond: Condition,
};

pub const SoftwareInterruptInstructionTags: u32 = 0b0000_1111_0000_0000_0000_0000_0000_0000;
pub const SoftwareInterruptInstructionMask: u32 = 0b0000_1111_0000_0000_0000_0000_0000_0000;

pub const SoftwareInterruptInstruction = packed struct(u32) {
    _: u28,
    cond: Condition,
};

pub const MRSInstructionTags: u32 = 0b0000_0001_0000_0000_0000_0000_0000_0000;
pub const MRSInstructionMask: u32 = 0b0000_1111_1011_0000_0000_0000_0000_0000;

pub const MRSInstruction = packed struct(u32) {
    sbz: u12,
    rd: u4,
    sbo: u4,
    _tag1: u2,
    r: u1,
    _tag2: u5,
    cond: Condition,
};

pub const MSRInstructionTags: u32 = 0b0000_0001_0010_0000_0000_0000_0000_0000;
pub const MSRInstructionMask: u32 = 0b0000_1101_1011_0000_0000_0000_0000_0000;

pub const FieldMask = packed struct(u4) {
    c: u1,
    x: u1,
    s: u1,
    f: u1,
};

pub const MSRInstruction = packed struct(u32) {
    source_operand: u12,
    sbo: u4,
    field_mask: FieldMask,
    _tag1: u2,
    r: u1,
    _tag2: u2,
    i: u1,
    _tag3: u2,
    cond: Condition,
};

pub const Instruction = packed union {
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
    MRS: MRSInstruction,
    MSR: MSRInstruction,
    SoftwareInterrupt: SoftwareInterruptInstruction,
};

pub const InstructionMasks = [_]u32{
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
    MRSInstructionMask,
    MSRInstructionMask,
    DataProcessingInstructionMask,
};

pub const InstructionTags = [_]u32{
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
    MRSInstructionTags,
    MSRInstructionTags,
    DataProcessingInstructionTags,
};

pub fn zero_extend(val: anytype) u32 {
    return @as(u32, val);
}

pub fn sign_extend(comptime T: type, val: T) u32 {
    const sign_bit_mask = comptime @as(T, 1 << (@bitSizeOf(T) - 1));
    if (val & sign_bit_mask != 0) {
        const high_bits: u32 = comptime @as(u32, 0xFFFFFFFF) << @intCast(@bitSizeOf(T));
        return @as(u32, val) | high_bits;
    } else {
        return val;
    }
}

pub var JumpTable: [0x1000]u8 = undefined;

fn init_jump_table() void {
    for (0..0x1000) |i| {
        const tag: u12 = @truncate(i);
        JumpTable[tag] = 255;
        for (0..InstructionTags.len) |t| {
            if (tag & ARM7.get_instr_tag(InstructionMasks[t]) == ARM7.get_instr_tag(InstructionTags[t])) {
                JumpTable[tag] = @intCast(t);
                // Collision are expected, decoding order matters.
                break;
            }
        }
        if (JumpTable[tag] == 255) {
            std.debug.print("  {d: >6} {b:0>12}\n", .{ i, tag });
            // @panic("Instruction tag not found");
            JumpTable[tag] = dissasemble.DisassembleTable.len - 1;
        }
    }
}

const Exception = enum {
    Reset,
    DataAbort,
    FIQ,
    IRQ,
    PrefetchAbort,
    SoftwareInterrupt,
    Undefined,
};

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

    memory_address_mask: u32, // Bits used for internal memory addressing (should mask anything outisde of memory.len)
    external_memory_address_mask: u32, // Bit(s) used to signal external memory access. If (addr & external_memory_address_mask) != 0, we'll use the external callbacks.

    cpsr: CPSR = .{},
    r: [16]u32 = undefined,
    r_fiq_8_12: [5]u32 = undefined, //  8-12
    r_usr: [2]u32 = undefined,
    r_fiq: [2]u32 = undefined, // 13-14
    r_svc: [2]u32 = undefined, // 13-14
    r_irq: [2]u32 = undefined, // 13-14
    r_abt: [2]u32 = undefined, // 13-14
    r_und: [2]u32 = undefined, // 13-14
    spsr_irq: CPSR = .{},
    spsr_svc: CPSR = .{},
    spsr_fiq: CPSR = .{},
    spsr_abt: CPSR = .{},
    spsr_und: CPSR = .{},

    instruction_pipeline: [1]u32 = undefined,

    fiq_signaled: bool = false,

    running: bool = false,

    on_external_read8: struct {
        callback: *const fn (data: *const anyopaque, address: u32) u8 = undefined,
        data: ?*const anyopaque = null,
    } = .{},
    on_external_read32: struct {
        callback: *const fn (data: *const anyopaque, address: u32) u32 = undefined,
        data: ?*const anyopaque = null,
    } = .{},
    on_external_write8: struct {
        callback: *const fn (data: *anyopaque, address: u32, value: u8) void = undefined,
        data: ?*anyopaque = null,
    } = .{},
    on_external_write32: struct {
        callback: *const fn (data: *anyopaque, address: u32, value: u32) void = undefined,
        data: ?*anyopaque = null,
    } = .{},

    pub fn init(memory: []u8, memory_address_mask: u32, external_memory_address_mask: u32) @This() {
        init_jump_table();
        return .{
            .memory = memory,
            .memory_address_mask = memory_address_mask,
            .external_memory_address_mask = external_memory_address_mask,
        };
    }

    pub fn reset(self: *@This(), enable: bool) void {
        // When the nRESET signal goes LOW a reset occurs, and the ARM7TDMI core
        // abandons the executing instruction and continues to increment the address bus as if still
        // fetching word or halfword instructions. nMREQ and SEQ indicates internal cycles
        // during this time.

        // When nRESET goes HIGH again, the ARM7TDMI processor:
        // 1. Overwrites R14_svc and SPSR_svc by copying the current values of the PC and
        // CPSR into them. The values of the PC and CPSR are indeterminate.
        // 2. Forces M[4:0] to b10011, Supervisor mode, sets the I and F bits, and clears the
        // T-bit in the CPSR.
        // 3. Forces the PC to fetch the next instruction from address 0x00.
        // 4. Reverts to ARM state if necessary and resumes execution.
        // After reset, all register values except the PC and CPSR are indeterminate.
        if (!self.running and enable) {
            self.r_svc[1] = self.pc();
            self.save_cpsr(.Supervisor);
            self.change_mode(.Supervisor);
            self.cpsr.i = true;
            self.cpsr.f = true;
            self.cpsr.t = false;

            self.set_pc(0x00);
        }
        self.running = enable;
    }

    pub fn set_cpsr(self: *@This(), cpsr: CPSR) void {
        self.change_mode(cpsr.m);
        self.cpsr = cpsr;
    }

    pub fn restore_cpsr(self: *@This()) void {
        self.set_cpsr(self.spsr().*);
    }

    pub fn banked_regs(self: *@This(), mode: RegisterMode) []u32 {
        return switch (mode) {
            .User, .System => &self.r_usr,
            .FastInterrupt => &self.r_fiq,
            .Interrupt => &self.r_irq,
            .Supervisor => &self.r_svc,
            .Abort => &self.r_abt,
            .Undefined => &self.r_und,
        };
    }

    // Saves CPSR to SPSR of the supplied mode (used by interrupt before changing to the new mode)
    fn save_cpsr(self: *@This(), mode: RegisterMode) void {
        if (mode != .User and mode != .System) {
            self.spsr_for(mode).* = self.cpsr;
        }
    }

    pub fn change_mode(self: *@This(), mode: RegisterMode) void {
        const prev_mode = self.cpsr.m;

        if (prev_mode == mode)
            return;

        // Save registers of the previous mode.
        const save_to = self.banked_regs(prev_mode);

        // Restore registers for the new one.
        const restore_from = self.banked_regs(mode);

        save_to[0] = self.r[13];
        save_to[1] = self.r[14];

        self.r[13] = restore_from[0];
        self.r[14] = restore_from[1];

        if (prev_mode == .FastInterrupt or mode == .FastInterrupt) {
            for (0..5) |i| {
                std.mem.swap(u32, &self.r[8 + i], &self.r_fiq_8_12[i]);
            }
        }

        self.cpsr.m = mode;
    }

    pub fn check_fiq(self: *@This()) void {
        //                         FIQ disabled.
        if (self.fiq_signaled and !self.cpsr.f) {

            // When a FIQ is detected, ARM7DI:
            // (1) Saves the address of the next instruction to be executed plus 4 in R14_fiq; saves CPSR in SPSR_fiq
            // (2) Forces M[4:0]=10001 (FIQ mode) and sets the F and I bits in the CPSR
            // (3) Forces the PC to fetch the next instruction from address 0x1C

            // NOTE: The order here is slightly different because of how change_mode works, but the result should be the same.

            self.save_cpsr(.FastInterrupt);
            self.change_mode(.FastInterrupt);
            self.cpsr.f = true;
            self.cpsr.i = true;
            self.set_lr(self.pc());
            self.set_pc(0x1C);
        }
    }

    // Stack Pointer
    pub inline fn sp(self: *@This()) u32 {
        return self.r[13];
    }

    // Link Register
    pub inline fn lr(self: *@This()) u32 {
        return self.r[14];
    }

    pub inline fn set_lr(self: *@This(), new_value: u32) void {
        self.r[14] = new_value;
    }

    // Program Counter
    pub inline fn pc(self: *@This()) u32 {
        return self.r[15];
    }

    pub inline fn set_pc(self: *@This(), new_value: u32) void {
        self.r[15] = new_value;
        self.reset_pipeline();
    }

    pub inline fn spsr_for(self: *@This(), mode: RegisterMode) *CPSR {
        return switch (mode) {
            .User, .System => @panic("Attempt to access SPSR of User/System mode"),
            .FastInterrupt => &self.spsr_fiq,
            .Interrupt => &self.spsr_irq,
            .Supervisor => &self.spsr_svc,
            .Abort => &self.spsr_abt,
            .Undefined => &self.spsr_und,
        };
    }

    pub inline fn spsr(self: *@This()) *CPSR {
        return self.spsr_for(self.cpsr.m);
    }

    pub inline fn get_instr_condition(instruction: u32) Condition {
        return @enumFromInt(@as(u4, @truncate(instruction >> 28)));
    }

    pub inline fn get_instr_tag(instruction: u32) u12 {
        return @truncate(((instruction >> 16) & 0xFF0) | ((instruction >> 4) & 0xF));
    }

    pub fn fetch(self: *@This()) u32 {
        const instr = @as(*const u32, @alignCast(@ptrCast(&self.memory[self.pc() & self.memory_address_mask]))).*;
        self.r[15] +%= 4;
        return instr;
    }

    pub fn reset_pipeline(self: *@This()) void {
        self.instruction_pipeline[0] = self.fetch();
    }

    pub fn read(self: *const @This(), comptime T: type, address: u32) T {
        if (address & self.external_memory_address_mask != 0) switch (T) {
            u8 => return self.on_external_read8.callback(self.on_external_read8.data.?, address),
            u32 => return self.on_external_read32.callback(self.on_external_read32.data.?, address),
            else => @compileError("Unsupported type: " ++ @typeName(T)),
        };
        return @as(*const T, @alignCast(@ptrCast(&self.memory[address & self.memory_address_mask]))).*;
    }

    pub fn write(self: *@This(), comptime T: type, address: u32, value: T) void {
        if (address & self.external_memory_address_mask != 0) switch (T) {
            u8 => return self.on_external_write8.callback(self.on_external_write8.data.?, address, value),
            u32 => return self.on_external_write32.callback(self.on_external_write32.data.?, address, value),
            else => @compileError("Unsupported type: " ++ @typeName(T)),
        };
        @as(*T, @alignCast(@ptrCast(&self.memory[address & self.memory_address_mask]))).* = value;
    }

    pub inline fn in_a_privileged_mode(self: *const @This()) bool {
        return self.cpsr.m != .User;
    }

    pub fn disassemble(instr: u32) []const u8 {
        return dissasemble.DisassembleTable[JumpTable[@This().get_instr_tag(instr)]](instr);
    }
};
