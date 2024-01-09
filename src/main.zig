const std = @import("std");

const arm7 = @import("arm7.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    var allocator = gpa.allocator();
    defer {
        const deinit_status = gpa.deinit();
        if (deinit_status == .leak) {
            std.debug.print("Memory leaked!\n", .{});
        }
    }

    const mem = try allocator.alloc(u8, 0x40000);
    defer allocator.free(mem);
    @memset(mem, 0);

    var core = arm7.ARM7.init(mem);

    const gba_test_file = try std.fs.cwd().openFile("bin/armwrestler-dc.bin", .{});
    defer gba_test_file.close();
    const read_bytes = try gba_test_file.readAll(mem);
    _ = read_bytes;

    core.r[15] = 0;

    for (0..32) |_| {
        const instr = core.fetch();
        //std.debug.print("{X:0>8}: {b:0>32}\n", .{ @as(u32, @bitCast(instr)), @as(u32, @bitCast(instr)) });
        //std.debug.print("{X:0>8}: {s}\n", .{ @as(u32, @bitCast(instr)), arm7.ARM7.disassemble(instr) });
        std.debug.print("{X:0>8}: {b:0>32} {s}\n", .{ @as(u32, @bitCast(instr)), @as(u32, @bitCast(instr)), arm7.ARM7.disassemble(instr) });
    }
}
