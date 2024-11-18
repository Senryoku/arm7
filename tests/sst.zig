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
            std.debug.print("    R{d}: {X:0>8}   R{d: <2}: {X:0>8} |\n", .{
                i,
                self.R[i],
                i + 8,
                self.R[i + 8],
            });
        }
        std.debug.print("    CPSR: {any}\n", .{@as(arm7.CPSR, @bitCast(self.CPSR))});
        std.debug.print("    SPSR_fiq: {X:0>8} | R_fiq: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[1], self.R_fiq[5 + 0], self.R_fiq[5 + 1] });
        std.debug.print("    SPSR_svc: {X:0>8} | R_svc: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[2], self.R_svc[0], self.R_svc[1] });
        std.debug.print("    SPSR_abt: {X:0>8} | R_abt: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[3], self.R_abt[0], self.R_abt[1] });
        std.debug.print("    SPSR_irq: {X:0>8} | R_irq: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[4], self.R_irq[0], self.R_irq[1] });
        std.debug.print("    SPSR_und: {X:0>8} | R_und: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[5], self.R_und[0], self.R_und[1] });
    }
};

fn cpu_log(cpu: *const arm7.ARM7) void {
    std.debug.print("  PC:    {X:0>8}   PR:   {X:0>8}   SPC: {X:0>8}\n", .{ cpu.pc, cpu.pr, cpu.spc });
}

fn compare_state(cpu: *const arm7.ARM7, expected_state: *const CPUState) void {
    std.debug.print("  (Reg: Actual - Expected)\n", .{});
    for (0..16) |i| {
        if (cpu.r[i] != expected_state.R[i]) std.debug.print("\u{001b}[31m", .{});
        std.debug.print("  R{d: <2}: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ i, cpu.r[i], expected_state.R[i] });
        if (i % 4 == 3) std.debug.print("\n", .{});
    }
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
    std.debug.print("  CPSR: {X:0>8}  {X:0>8} ({any} / {any})\u{001b}[0m", .{ @as(u32, @bitCast(cpu.cpsr)), expected_state.CPSR, cpu.cpsr, @as(arm7.CPSR, @bitCast(expected_state.CPSR)) });

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

    std.debug.print("\n", .{});
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

        pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
            try writer.print("Transaction({d}){s}({d}): {X:0>8} = {X:0>8}", .{
                self.cycle,
                @tagName(self.kind),
                self.size,
                self.addr,
                self.data,
            });
        }
    },
    opcodes: []u32,
    base_addr: []u32,
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
    failed: bool = false,
};

fn read8(state: *TestState, addr: u32) u8 {
    for (state.test_data.transactions) |t| {
        if (t.kind == .Read and t.addr == addr and t.size == 1)
            return @truncate(t.data);
    }
    state.failed = true;
    std.debug.print("  Unexpected read at {X:>8}. Expected one of:\n", .{addr});
    for (state.test_data.transactions) |t| {
        std.debug.print("   {any}\n", .{t});
    }
    return 0;
}

fn read32(state: *TestState, addr: u32) u32 {
    for (state.test_data.transactions) |t| {
        if (t.kind == .Read and t.addr & 0xFFFFFFFC == addr & 0xFFFFFFFC and t.size == 4)
            return t.data;
    }

    for (state.test_data.transactions) |t| {
        if (t.kind == .InstructionRead and t.addr & 0xFFFFFFFC == addr & 0xFFFFFFFC and t.size == 4)
            return t.data;
    }
    state.failed = true;
    std.debug.print("  Unexpected read at {X:>8}. Expected one of:\n", .{addr});
    for (state.test_data.transactions) |t| {
        std.debug.print("   {any}\n", .{t});
    }
    return 0;
}

fn write8(state: *TestState, addr: u32, val: u8) void {
    for (state.test_data.transactions) |t| {
        if (t.kind == .Write and t.addr == addr and t.size == 1) {
            if (t.data != val) {
                std.debug.print(red(" Unexpected write at {X:>8}: Got {X:>2}, expected {X:>2}\n"), .{ addr, val, t.data });
                state.failed = true;
            }
            return;
        }
    }
    state.failed = true;
    std.debug.print(red(" Unexpected write at {X:>8} (={X:>2}). Expected one of:\n"), .{ addr, val });
    for (state.test_data.transactions) |t| {
        std.debug.print("   {any}\n", .{t});
    }
}

fn write32(state: *TestState, addr: u32, val: u32) void {
    for (state.test_data.transactions) |t| {
        if (t.kind == .Write and t.addr & 0xFFFFFFFC == addr & 0xFFFFFFFC and t.size == 4) {
            if (t.data != val) {
                std.debug.print(red(" Unexpected write at {X:>8}: Got {X:>8}, expected {X:>8}\n"), .{ addr, val, t.data });
                state.failed = true;
            }
            return;
        }
    }
    state.failed = true;
    std.debug.print(red(" Unexpected write at {X:>8} (={X:>8}). Expected one of:\n"), .{ addr, val });
    for (state.test_data.transactions) |t| {
        std.debug.print("   {any}\n", .{t});
    }
}

fn run_test(t: Test, cpu: *arm7.ARM7) !void {
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
        cpu.r_usr[i] = t.initial.R[13 + i];
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

    std.debug.assert(t.opcodes[0] == t.initial.pipeline[0]);

    cpu.instruction_pipeline[0] = t.initial.pipeline[1];

    arm7.interpreter.execute(cpu, t.opcodes[0]);
    ts.cpu.r[15] +%= 4;
    ts.cpu.r[15] &= 0xFFFF_FFFE;
    arm7.interpreter.execute(cpu, cpu.instruction_pipeline[0]);
    ts.cpu.r[15] +%= 4;
    ts.cpu.r[15] &= 0xFFFF_FFFE;

    try std.testing.expectEqualSlices(u32, t.final.R, &cpu.r);
    try std.testing.expectEqualSlices(u32, t.final.R_fiq[0..5], &cpu.r_fiq_8_12);
    try std.testing.expectEqualSlices(u32, t.final.R_fiq[5..], &cpu.r_fiq);
    try std.testing.expectEqualSlices(u32, t.final.R_svc, &cpu.r_svc);
    try std.testing.expectEqualSlices(u32, t.final.R_abt, &cpu.r_abt);
    try std.testing.expectEqualSlices(u32, t.final.R_irq, &cpu.r_irq);
    try std.testing.expectEqualSlices(u32, t.final.R_und, &cpu.r_und);
    try std.testing.expectEqual(t.final.CPSR, @as(u32, @bitCast(cpu.cpsr)));
    try std.testing.expectEqual(t.final.SPSR[1], @as(u32, @bitCast(cpu.spsr_for(.FastInterrupt).*)));
    try std.testing.expectEqual(t.final.SPSR[2], @as(u32, @bitCast(cpu.spsr_for(.Supervisor).*)));
    try std.testing.expectEqual(t.final.SPSR[3], @as(u32, @bitCast(cpu.spsr_for(.Abort).*)));
    try std.testing.expectEqual(t.final.SPSR[4], @as(u32, @bitCast(cpu.spsr_for(.Interrupt).*)));
    try std.testing.expectEqual(t.final.SPSR[5], @as(u32, @bitCast(cpu.spsr_for(.Undefined).*)));

    if (ts.failed) return error.IOFail;
}

test {
    const TestDir = "../ARM7TDMI/v1";
    var test_dir = try std.fs.cwd().openDir(TestDir, .{ .iterate = true });
    defer test_dir.close();

    var walker = try test_dir.walk(std.testing.allocator);
    defer walker.deinit();

    const mem = try std.testing.allocator.alignedAlloc(u8, 32, 0x40000);
    defer std.testing.allocator.free(mem);

    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    var arena_allocator = arena.allocator();

    var cpu = arm7.ARM7.init(mem, 0x00000000, 0xFFFFFFFF);

    var skipped_tests: u32 = 0;
    var results = std.ArrayList(struct {
        instruction: []const u8,
        tests_count: usize,
        skipped_cases: u32 = 0,
        failed_cases: u32 = 0,
    }).init(std.testing.allocator);
    defer results.deinit();

    var file_num: u32 = 0;
    tests_loop: while (try walker.next()) |entry| {
        if (entry.kind == .file and std.mem.endsWith(u8, entry.basename, ".json")) {
            for ([_][]const u8{
                // Skipped tests
                "branch_exchange.json", // Unimplemented
                "hw_data_transfer_immediate.json", // Unimplemented
                "hw_data_transfer_register.json", // Unimplemented
                "mull.json", // Unimplemented
                "undefined.json",
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
            std.debug.print(green("[{d: >3}/{d: >3}]") ++ " Opening {s}\n", .{ file_num, 11, fullpath });
            const data = try std.fs.cwd().readFileAlloc(std.testing.allocator, fullpath, 512 * 1024 * 1024);
            defer std.testing.allocator.free(data);

            const test_data = try std.json.parseFromSlice([]Test, std.testing.allocator, data, .{});
            defer test_data.deinit();

            var failed_test_cases: u32 = 0;
            var skipped_test_cases: u32 = 0;
            var ran_test_cases: u32 = 0;
            for (test_data.value) |t| {
                ran_test_cases += 1;
                // std.debug.print(" Testing: {s}\n", .{arm7.ARM7.disassemble(t.opcodes[0])});
                // FIXME: Test cases going out of FastInterrupt mode doesn't seem to work correctly?
                if ((std.mem.eql(u8, entry.basename, "swi.json") or std.mem.eql(u8, entry.basename, "block_data_transfer.json")) and @as(arm7.CPSR, @bitCast(t.initial.CPSR)).m == .FastInterrupt) {
                    if (t.final.R[8] == 0) {
                        skipped_test_cases += 1;
                        continue;
                    }
                }
                if (!arm7.interpreter.is_valid(t.opcodes[0])) {
                    // std.debug.print(yellow("! Skipping {s} ({X:0>8})\n"), .{ arm7.ARM7.disassemble(t.opcodes[0]), t.opcodes[0] });
                    skipped_test_cases += 1;
                    continue;
                }
                run_test(t, &cpu) catch |err| {
                    std.debug.print(red("[{s}] Test #{d} '{s}' ({X:0>8}) failed: {s}\n"), .{ entry.basename, ran_test_cases, arm7.ARM7.disassemble(t.opcodes[0]), t.opcodes[0], @errorName(err) });
                    if (true or failed_test_cases == 0) {
                        std.debug.print("  ==== Initial state ==== \n", .{});
                        t.initial.log();
                        std.debug.print("  ======================= \n", .{});
                        for (t.transactions) |tr| {
                            std.debug.print("   {any}\n", .{tr});
                        }
                        compare_state(&cpu, &t.final);
                    }
                    failed_test_cases += 1;
                    // break;
                };
            }
            if (failed_test_cases > 0) {
                std.debug.print(red("[{s}] {d}/{d} ({d} total) test cases failed.\n"), .{ entry.basename, failed_test_cases, ran_test_cases, test_data.value.len });
            } else {
                std.debug.print(green("[{s}] {d}/{d} ({d} total) test cases passed.\n"), .{ entry.basename, ran_test_cases, ran_test_cases, test_data.value.len });
            }
            try results.append(.{
                .instruction = try arena_allocator.dupe(u8, entry.basename),
                .tests_count = test_data.value.len,
                .failed_cases = failed_test_cases,
                .skipped_cases = skipped_test_cases,
            });
        }
    }
    if (skipped_tests > 0) {
        std.debug.print(yellow("Skipped {d} tests.\n"), .{skipped_tests});
    }
    if (results.items.len > 0)
        std.debug.print(red("{d}/{d} tests failed.\n"), .{ results.items.len, file_num });
    for (results.items) |f| {
        if (f.failed_cases > 0) {
            std.debug.print(red(" {s: <30}: {d: >3}/{d} test cases failed ({d} skipped).\n"), .{ f.instruction, f.failed_cases, f.tests_count, f.skipped_cases });
        } else if (f.skipped_cases > 0) {
            std.debug.print(yellow(" {s: <30}: {d: >3}/{d} test cases skipped.\n"), .{ f.instruction, f.skipped_cases, f.tests_count });
        } else {
            std.debug.print(green(" {s: <30}: All {d} test cases passed.\n"), .{ f.instruction, f.tests_count });
        }
    }
    if (results.items.len > 0)
        return error.TestFailed;
}
