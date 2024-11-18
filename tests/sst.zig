// https://github.com/SingleStepTests/ARM7TDMI

const std = @import("std");
const arm7 = @import("arm7");

pub fn green(comptime str: []const u8) []const u8 {
    return "\u{001b}[32m" ++ str ++ "\u{001b}[0m";
}

pub fn yellow(comptime str: []const u8) []const u8 {
    return "\u{001b}[33m" ++ str ++ "\u{001b}[0m";
}

pub fn red(comptime str: []const u8) []const u8 {
    return "\u{001b}[31m" ++ str ++ "\u{001b}[0m";
}

const CPUState = struct {
    R: []u32,
    R_fiq: []u32,
    R_svc: []u32,
    R_abt: []u32,
    R_irq: []u32,
    R_und: []u32,
    CPSR: u32,
    SPSR: []u32,
    pipeline: []u32,

    fn log(self: *const @This()) void {
        for (0..8) |i| {
            std.debug.print("  R{d}: {X:0>8}   R{d: <2}: {X:0>8} |\n", .{
                i,
                self.R[i],
                i + 8,
                self.R[i + 8],
            });
        }
    }
};

fn cpu_log(cpu: *const arm7.ARM7) void {
    std.debug.print("  PC:    {X:0>8}   PR:   {X:0>8}   SPC: {X:0>8}\n", .{ cpu.pc, cpu.pr, cpu.spc });
}

fn compare_state(cpu: *const arm7.ARM7, expected_state: *const CPUState) void {
    for (0..16) |i| {
        if (cpu.r[i] != expected_state.R[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i, cpu.r[i], expected_state.R[i] });
        if (i % 4 == 3) std.debug.print("\n", .{});
    }
    std.debug.print("\n", .{});
    for (0..5) |i| {
        if (cpu.r_fiq_8_12[i] != expected_state.R_fiq[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R_fiq{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i, cpu.r_fiq_8_12[i], expected_state.R_fiq[i] });
    }
    std.debug.print("\n", .{});
    for (0..2) |i| {
        if (cpu.r_fiq[i] != expected_state.R_fiq[5 + i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R_fiq{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ 5 + i, cpu.r_fiq[i], expected_state.R_fiq[5 + i] });
    }
    std.debug.print("\n", .{});
    for (0..2) |i| {
        if (cpu.r_svc[i] != expected_state.R_svc[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R_svc{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i, cpu.r_svc[i], expected_state.R_svc[i] });
    }
    std.debug.print("\n", .{});
    for (0..2) |i| {
        if (cpu.r_abt[i] != expected_state.R_abt[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R_abt{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i, cpu.r_abt[i], expected_state.R_abt[i] });
    }
    std.debug.print("\n", .{});
    for (0..2) |i| {
        if (cpu.r_irq[i] != expected_state.R_irq[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R_irq{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i, cpu.r_irq[i], expected_state.R_irq[i] });
    }
    std.debug.print("\n", .{});
    for (0..2) |i| {
        if (cpu.r_und[i] != expected_state.R_und[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R_und{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i, cpu.r_und[i], expected_state.R_und[i] });
    }
    std.debug.print("\n", .{});

    if (@as(u32, @bitCast(cpu.cpsr)) != expected_state.CPSR) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  CPSR: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.cpsr)), expected_state.CPSR });

    std.debug.print("\n", .{});

    if (@as(u32, @bitCast(cpu.spsr_fiq)) != expected_state.SPSR[1]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_fiq: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_fiq)), expected_state.SPSR[1] });
    if (@as(u32, @bitCast(cpu.spsr_svc)) != expected_state.SPSR[2]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_svc: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_svc)), expected_state.SPSR[2] });

    if (@as(u32, @bitCast(cpu.spsr_abt)) != expected_state.SPSR[3]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_abt: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_abt)), expected_state.SPSR[3] });
    if (@as(u32, @bitCast(cpu.spsr_irq)) != expected_state.SPSR[4]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_irq: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_irq)), expected_state.SPSR[4] });
    if (@as(u32, @bitCast(cpu.spsr_und)) != expected_state.SPSR[5]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_und: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_und)), expected_state.SPSR[5] });
}

const Test = struct {
    initial: CPUState,
    final: CPUState,
    transactions: []struct {
        kind: enum(u32) { InstructionRead = 0, Read = 1, Write = 2 },
        size: u32,
        addr: u32,
        data: u32,
        cycle: u32,
    },
    opcodes: []u32,
    base_addr: []u32,

    fn log(self: *const @This()) void {
        self.initial.log();

        for (self.opcodes) |opcode| {
            std.debug.print("   > {X}\n", .{opcode});
        }
    }
};

pub const std_options: std.Options = .{
    .log_level = .info,
    .log_scope_levels = &[_]std.log.ScopeLevel{
        .{ .scope = .dc, .level = .err },
        .{ .scope = .sh4, .level = .err },
        .{ .scope = .sh4_jit, .level = .err },
        .{ .scope = .arm_jit, .level = .err },
        .{ .scope = .x86_64_emitter, .level = .err },
        .{ .scope = .syscall_log, .level = .err },
        .{ .scope = .aica, .level = .err },
        .{ .scope = .holly, .level = .err },
        .{ .scope = .gdrom, .level = .err },
        .{ .scope = .maple, .level = .err },
        .{ .scope = .renderer, .level = .err },
    },
};

const TestState = struct {
    cpu: *arm7.ARM7 = undefined,
    cycle: u32 = 0,
    test_data: Test = undefined,
};

fn read8(state: *TestState, addr: u32) u8 {
    for (state.test_data.transactions) |t| {
        if (t.kind == .Read and t.addr == addr) {
            return @truncate(t.data);
        }
    }
    std.debug.print("Unexpected read at {X:>8}\n", .{addr});
    return 0;
}

fn read32(state: *TestState, addr: u32) u32 {
    for (state.test_data.transactions) |t| {
        if (t.kind == .Read and t.addr == addr) {
            return t.data;
        }
    }
    std.debug.print("Unexpected read at {X:>8}\n", .{addr});
    return 0;
}

fn write8(state: *TestState, addr: u32, val: u8) void {
    for (state.test_data.transactions) |t| {
        if (t.kind == .Write and t.addr == addr) {
            if (t.data != val) {
                std.debug.print(red(" Unexpected write at {X:>8}: {X:>2} != {X:>2}\n"), .{ addr, val, t.data });
            }
        }
    }
    std.debug.print(red(" Unexpected write at {X:>8} (={X:>2})\n"), .{ addr, val });
}

fn write32(state: *TestState, addr: u32, val: u32) void {
    for (state.test_data.transactions) |t| {
        if (t.kind == .Write and t.addr == addr) {
            if (t.data != val) {
                std.debug.print(red(" Unexpected write at {X:>8}: {X:>8} != {X:>8}\n"), .{ addr, val, t.data });
            }
        }
    }
    std.debug.print(red(" Unexpected write at {X:>8} (={X:>8})\n"), .{ addr, val });
}

fn run_test(t: Test, cpu: *arm7.ARM7, comptime log: bool) !void {
    var ts: TestState = .{ .cpu = cpu, .cycle = 0, .test_data = t };
    cpu.on_external_read8 = .{ .callback = @ptrCast(&read8), .data = &ts };
    cpu.on_external_read32 = .{ .callback = @ptrCast(&read32), .data = &ts };
    cpu.on_external_write8 = .{ .callback = @ptrCast(&write8), .data = &ts };
    cpu.on_external_write32 = .{ .callback = @ptrCast(&write32), .data = &ts };

    for (0..16) |i| {
        cpu.r[i] = t.initial.R[i];
    }

    for (0..5) |i| {
        cpu.r_fiq_8_12[i] = t.initial.R_fiq[i];
    }

    for (0..2) |i| {
        cpu.r_fiq[i] = t.initial.R_fiq[5 + i];
        cpu.r_svc[i] = t.initial.R_svc[i];
        cpu.r_abt[i] = t.initial.R_abt[i];
        cpu.r_irq[i] = t.initial.R_irq[i];
        cpu.r_und[i] = t.initial.R_und[i];
    }

    cpu.cpsr = @bitCast(t.initial.CPSR);
    cpu.spsr_fiq = @bitCast(t.initial.SPSR[1]);
    cpu.spsr_svc = @bitCast(t.initial.SPSR[2]);
    cpu.spsr_abt = @bitCast(t.initial.SPSR[3]);
    cpu.spsr_irq = @bitCast(t.initial.SPSR[4]);
    cpu.spsr_und = @bitCast(t.initial.SPSR[5]);

    cpu.instruction_pipeline[0] = t.initial.pipeline[1];

    if (log) {
        t.log();
    }

    arm7.interpreter.execute(cpu, t.opcodes[0]);
    ts.cpu.r[15] +%= 4;
    arm7.interpreter.execute(cpu, t.opcodes[
        if (ts.cpu.r[15] == t.initial.R[15] +% 4) 1 else 3
    ]);
    ts.cpu.r[15] +%= 4;

    try std.testing.expectEqualSlices(u32, t.final.R, &cpu.r);
    try std.testing.expectEqualSlices(u32, t.final.R_fiq[0..5], &cpu.r_fiq_8_12);
    try std.testing.expectEqualSlices(u32, t.final.R_fiq[5..], &cpu.r_fiq);
    try std.testing.expectEqualSlices(u32, t.final.R_svc, &cpu.r_svc);
    try std.testing.expectEqualSlices(u32, t.final.R_abt, &cpu.r_abt);
    try std.testing.expectEqualSlices(u32, t.final.R_irq, &cpu.r_irq);
    try std.testing.expectEqualSlices(u32, t.final.R_und, &cpu.r_und);
    try std.testing.expectEqual(t.final.CPSR, @as(u32, @bitCast(cpu.cpsr)));
    try std.testing.expectEqual(t.final.SPSR[0], @as(u32, @bitCast(cpu.spsr_for(.FastInterrupt).*)));
    try std.testing.expectEqual(t.final.SPSR[1], @as(u32, @bitCast(cpu.spsr_for(.Supervisor).*)));
    try std.testing.expectEqual(t.final.SPSR[2], @as(u32, @bitCast(cpu.spsr_for(.Abort).*)));
    try std.testing.expectEqual(t.final.SPSR[3], @as(u32, @bitCast(cpu.spsr_for(.Interrupt).*)));
    try std.testing.expectEqual(t.final.SPSR[4], @as(u32, @bitCast(cpu.spsr_for(.Undefined).*)));
}

test {
    const TestDir = "../ARM7TDMI/v1";
    var test_dir = try std.fs.cwd().openDir(TestDir, .{ .iterate = true });
    defer test_dir.close();

    var walker = try test_dir.walk(std.testing.allocator);
    defer walker.deinit();

    const mem = try std.testing.allocator.alignedAlloc(u8, 32, 0x40000);
    defer std.testing.allocator.free(mem);

    var cpu = arm7.ARM7.init(mem, 0x00000000, 0xFFFFFFFF);

    var skipped_tests: u32 = 0;
    var failed_tests = std.ArrayList(struct {
        instruction: []const u8,
        failed_cases: u32,
    }).init(std.testing.allocator);
    defer failed_tests.deinit();
    var file_num: u32 = 0;
    tests_loop: while (try walker.next()) |entry| {
        if (entry.kind == .file and std.mem.endsWith(u8, entry.basename, ".json")) {
            for ([_][]const u8{
                // Skipped tests
            }) |filename| {
                if (std.mem.eql(u8, entry.basename, filename)) {
                    std.debug.print(yellow("! Skipping {s}\n"), .{entry.basename});
                    skipped_tests += 1;
                    continue :tests_loop;
                }
            }
            file_num += 1;

            const fullpath = try std.fs.path.join(std.testing.allocator, &[_][]const u8{ TestDir, entry.basename });
            defer std.testing.allocator.free(fullpath);
            std.debug.print(green("[{d: >3}/{d: >3}]") ++ " Opening {s}\n", .{ file_num, 233, fullpath });
            const data = try std.fs.cwd().readFileAlloc(std.testing.allocator, fullpath, 512 * 1024 * 1024);
            defer std.testing.allocator.free(data);

            const test_data = try std.json.parseFromSlice([]Test, std.testing.allocator, data, .{});
            defer test_data.deinit();

            var failed_test_cases: u32 = 0;
            for (test_data.value) |t| {
                run_test(t, &cpu, false) catch |err| {
                    if (failed_test_cases == 0) {
                        std.debug.print(red("Failed to run test {s}: {s}\n"), .{ entry.basename, @errorName(err) });
                        run_test(t, &cpu, true) catch {};
                        compare_state(&cpu, &t.final);
                    }
                    failed_test_cases += 1;
                };
            }
            if (failed_test_cases > 0) {
                std.debug.print(red("  [{s}] {d}/{d} test cases failed.\n"), .{ entry.basename, failed_test_cases, test_data.value.len });
                try failed_tests.append(.{
                    .instruction = entry.basename,
                    .failed_cases = failed_test_cases,
                });
            }
        }
    }
    if (skipped_tests > 0) {
        std.debug.print(yellow("Skipped {d} tests.\n"), .{skipped_tests});
    }
    if (failed_tests.items.len > 0) {
        std.debug.print(red("{d}/{d} tests failed.\n"), .{ failed_tests.items.len, file_num });
        for (failed_tests.items) |f| {
            std.debug.print(red(" {s: <20} {d: >3}/{d} test cases failed.\n"), .{ f.instruction, f.failed_cases, 500 });
        }
        return error.TestFailed;
    }
}
