const std = @import("std");

const arm7 = @import("arm7.zig");
const arm7_interpreter = @import("arm7_interpreter.zig");
const arm7_disassemble = @import("arm7_disassemble.zig");

const test_program = [_]u8{
    // mov r0, #1
    0x01, 0x00, 0xa0, 0xe3,
    // mov r0, #0
    0x00, 0x00, 0xa0, 0xe3,
    // mov r1, #1
    0x01, 0x10, 0xa0, 0xe3,
    // mov r1, #0
    0x00, 0x10, 0xa0, 0xe3,
    // mov r0, #20000
    0x02, 0x08, 0xa0, 0xe3,
    // mov r1, #0xC0
    0xc0, 0x10, 0xa0, 0xe3,
    // str r1, [r0]
    0x00, 0x10, 0x80, 0xe5,
};

test "ARM7 Tests" {
    const mem = try std.testing.allocator.alignedAlloc(u8, 32, 0x40000);
    defer std.testing.allocator.free(mem);
    @memcpy(mem[0..test_program.len], &test_program);

    var cpu = arm7.ARM7.init(mem);

    cpu.pc().* = 0;
    cpu.reset_pipeline();

    // std.debug.print("[{X:0>8}] {X:0>8} {s}\n", .{ cpu.pc().* - 8, cpu.instruction_pipeline[0], arm7.ARM7.disassemble(cpu.instruction_pipeline[0]) });

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

    std.debug.print("[{X:0>8}] {X:0>8} {s}\n", .{ cpu.pc().* - 8, cpu.instruction_pipeline[0], arm7.ARM7.disassemble(cpu.instruction_pipeline[0]) });
    cpu.tick();
    std.debug.print("{X:0>2} {X:0>2} {X:0>2} {X:0>2} \n", .{ mem[0x20000], mem[0x20001], mem[0x20002], mem[0x20003] });
    try std.testing.expect(mem[0x20000] == 0xC0);
}
