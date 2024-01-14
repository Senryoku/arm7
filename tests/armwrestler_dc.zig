const std = @import("std");

const arm7 = @import("arm7");

const test_program = @embedFile("bin/armwrestler-dc.bin");

test "armwrestler-dc" {
    const mem = try std.testing.allocator.alloc(u8, 0x40000);
    defer std.testing.allocator.free(mem);
    @memset(mem, 0);

    var core = arm7.ARM7.init(mem);

    @memcpy(mem[0..test_program.len], test_program);

    core.pc().* = 0;
    core.reset_pipeline();
    for (0..128) |_| {
        const instr = core.instruction_pipeline[0];
        std.debug.print("[{X:0>8}] {X:0>8}: {b:0>32} {s}\n", .{ core.pc().* - 8, @as(u32, @bitCast(instr)), @as(u32, @bitCast(instr)), arm7.ARM7.disassemble(instr) });
        core.tick();
    }
}
