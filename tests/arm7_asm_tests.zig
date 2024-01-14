const std = @import("std");

const arm7 = @import("arm7");
const arm7_interpreter = @import("../arm7_interpreter.zig");
const arm7_disassemble = @import("../arm7_disassemble.zig");

const test_program = @embedFile("bin/basic.bin");

fn disassemble_current_instruction(cpu: *arm7.ARM7) void {
    std.debug.print("[{X:0>8}] {X:0>8} {s}\n", .{ cpu.pc().* - 8, cpu.instruction_pipeline[0], arm7.ARM7.disassemble(cpu.instruction_pipeline[0]) });
}

fn expect_mem(mem: []const u8, address: u32, value: u32) !void {
    try std.testing.expect(@as(*const u32, @alignCast(@ptrCast(&mem[address]))).* == value);
}

test "ARM7 Tests" {
    const mem = try std.testing.allocator.alignedAlloc(u8, 32, 0x40000);
    defer std.testing.allocator.free(mem);

    @memcpy(mem[0..test_program.len], test_program);

    var cpu = arm7.ARM7.init(mem);

    cpu.pc().* = 0;
    cpu.reset_pipeline();

    cpu.tick();
    try std.testing.expect(cpu.r(0).* == 1);
    cpu.tick();
    try std.testing.expect(cpu.r(0).* == 0);
    cpu.tick();
    try std.testing.expect(cpu.r(1).* == 1);
    cpu.tick();
    try std.testing.expect(cpu.r(1).* == 0);
    cpu.tick();

    try std.testing.expect(cpu.r(0).* == 0x20000);
    cpu.tick();
    try std.testing.expect(cpu.r(1).* == 0xC0);
    cpu.tick();
    try expect_mem(mem, 0x20000, 0x000000C0);

    cpu.tick();
    try std.testing.expect(cpu.r(1).* == 0xC1);
    cpu.tick();
    try expect_mem(mem, 0x20000, 0x000000C1);

    cpu.tick();
    try expect_mem(mem, 0x20000, 0x0000C1C1);
    cpu.tick();
    try expect_mem(mem, 0x20000, 0x00C1C1C1);
    cpu.tick();
    try expect_mem(mem, 0x20000, 0xC1C1C1C1);
}
