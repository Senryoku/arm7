const std = @import("std");

const arm7 = @import("arm7");
const arm7_interpreter = arm7.interpreter;
const arm7_disassemble = arm7.disassemble;

const test_program = @embedFile("bin/basic.bin");

fn disassemble_current_instruction(cpu: *arm7.ARM7) void {
    std.debug.print("[{X:0>8}] {X:0>8} {s}\n", .{ cpu.pc().* - 4, cpu.instruction_pipeline[0], arm7.ARM7.disassemble(cpu.instruction_pipeline[0]) });
}

fn expect_mem(mem: []const u8, address: u32, value: u32) !void {
    try std.testing.expect(@as(*const u32, @alignCast(@ptrCast(&mem[address]))).* == value);
}

test "ARM7 Tests" {
    const mem = try std.testing.allocator.alignedAlloc(u8, 32, 0x40000);
    defer std.testing.allocator.free(mem);

    @memcpy(mem[0..test_program.len], test_program);

    var cpu = arm7.ARM7.init(mem, 0x3FFFF, 0x40000);

    cpu.set_pc(0);

    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 1);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[1] == 1);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[1] == 0);
    arm7_interpreter.tick(&cpu);

    try std.testing.expect(cpu.r[0] == 0x20000);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[1] == 0xC0);
    arm7_interpreter.tick(&cpu);
    try expect_mem(mem, 0x20000, 0x000000C0);

    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[1] == 0xC1);
    arm7_interpreter.tick(&cpu);
    try expect_mem(mem, 0x20000, 0x000000C1);

    arm7_interpreter.tick(&cpu);
    try expect_mem(mem, 0x20000, 0x0000C1C1);
    arm7_interpreter.tick(&cpu);
    try expect_mem(mem, 0x20000, 0x00C1C1C1);
    arm7_interpreter.tick(&cpu);
    try expect_mem(mem, 0x20000, 0xC1C1C1C1);
}

const ldm_stm = @embedFile("bin/ldm_stm.bin");

test "ldm/stm" {
    const mem = try std.testing.allocator.alignedAlloc(u8, 32, 0x40000);
    defer std.testing.allocator.free(mem);

    @memset(mem, 0);
    @memcpy(mem[0..ldm_stm.len], ldm_stm);

    var cpu = arm7.ARM7.init(mem, 0x3FFFF, 0x40000);

    cpu.set_pc(0);

    for (0..15) |_| {
        arm7_interpreter.tick(&cpu);
    }
    try std.testing.expect(cpu.r[0] == 0x2000);
    for (1..15) |i| {
        try std.testing.expect(cpu.r[@intCast(i)] == i);
    }

    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x2000 - 14 * 4);
    for (1..15) |i| {
        try expect_mem(mem, @intCast(0x2000 - 14 * 4 + 4 * (i - 1)), @intCast(i));
    }
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x4000);
    arm7_interpreter.tick(&cpu);
    for (1..15) |i| {
        try std.testing.expect(cpu.r[@intCast(i)] == 0);
    }
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x2000);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x2000 - 14 * 4);
    arm7_interpreter.tick(&cpu);
    for (1..15) |i| {
        try std.testing.expect(cpu.r[@intCast(i)] == i);
    }
}

test "and" {
    const mem = try std.testing.allocator.alignedAlloc(u8, 32, 0x40000);
    defer std.testing.allocator.free(mem);

    const and_test = @embedFile("bin/and.bin");

    @memset(mem, 0);
    @memcpy(mem[0..and_test.len], and_test);

    var cpu = arm7.ARM7.init(mem, 0x3FFFF, 0x40000);

    cpu.set_pc(0);

    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0xFFFFFFFF);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x1);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x2);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x4);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x8);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x10);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x20);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x40);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x80);

    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0xFFFFFFFF);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x1);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x2);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x4);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x8);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x10);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x20);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x40);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x80);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x80000000);
    try std.testing.expect(cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);

    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x1);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x2);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(!cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x0);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(cpu.cpsr.z);
    arm7_interpreter.tick(&cpu);
    arm7_interpreter.tick(&cpu);
    try std.testing.expect(cpu.r[0] == 0x0);
    try std.testing.expect(!cpu.cpsr.n);
    try std.testing.expect(cpu.cpsr.z);
}
