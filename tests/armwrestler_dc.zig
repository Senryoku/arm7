const std = @import("std");

const arm7 = @import("arm7");

const test_program = @embedFile("bin/armwrestler_dc_test_0.bin");

test "armwrestler-dc-test-0" {
    const mem = try std.testing.allocator.alloc(u8, 0x200000);
    defer std.testing.allocator.free(mem);
    @memset(mem, 0);

    var cpu = arm7.ARM7.init(mem);

    @memcpy(mem[0..test_program.len], test_program);

    cpu.pc().* = 0;
    cpu.reset_pipeline();

    for (0..2048) |_| {
        const instr = cpu.instruction_pipeline[0];
        std.debug.print("[{X:0>8}] {X:0>8}: {b:0>32} {s}\n", .{ cpu.pc().* - 4, @as(u32, @bitCast(instr)), @as(u32, @bitCast(instr)), arm7.ARM7.disassemble(instr) });
        cpu.tick();
    }
}
