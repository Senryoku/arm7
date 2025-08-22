// https://github.com/SingleStepTests/ARM7TDMI

const std = @import("std");
const arm7 = @import("arm7.zig");

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
    access: u32,

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
        std.debug.print("    SPSR_fiq: {X:0>8} | R_fiq: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[0], self.R_fiq[5 + 0], self.R_fiq[5 + 1] });
        std.debug.print("    SPSR_svc: {X:0>8} | R_svc: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[1], self.R_svc[0], self.R_svc[1] });
        std.debug.print("    SPSR_abt: {X:0>8} | R_abt: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[2], self.R_abt[0], self.R_abt[1] });
        std.debug.print("    SPSR_irq: {X:0>8} | R_irq: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[3], self.R_irq[0], self.R_irq[1] });
        std.debug.print("    SPSR_und: {X:0>8} | R_und: [{X:0<8}, {X:0<8}]\n", .{ self.SPSR[4], self.R_und[0], self.R_und[1] });
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
    std.debug.print("  CPSR: {X:0>8}  {X:0>8}\n        {any}\n        {any}\u{001b}[0m", .{ @as(u32, @bitCast(cpu.cpsr)), expected_state.CPSR, cpu.cpsr, @as(arm7.CPSR, @bitCast(expected_state.CPSR)) });

    std.debug.print("\n", .{});

    if (@as(u32, @bitCast(cpu.spsr_fiq)) != expected_state.SPSR[0]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_fiq: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_fiq)), expected_state.SPSR[0] });
    if (@as(u32, @bitCast(cpu.spsr_svc)) != expected_state.SPSR[1]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_svc: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_svc)), expected_state.SPSR[1] });

    if (@as(u32, @bitCast(cpu.spsr_abt)) != expected_state.SPSR[2]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_abt: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_abt)), expected_state.SPSR[2] });
    if (@as(u32, @bitCast(cpu.spsr_irq)) != expected_state.SPSR[3]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_irq: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_irq)), expected_state.SPSR[3] });
    if (@as(u32, @bitCast(cpu.spsr_und)) != expected_state.SPSR[4]) std.debug.print("\u{001b}[31m", .{});
    std.debug.print("  SPSR_und: {X:0>8}  {X:0>8}\u{001b}[0m  |", .{ @as(u32, @bitCast(cpu.spsr_und)), expected_state.SPSR[4] });

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
        access: u32,

        pub fn format(self: @This(), comptime _: []const u8, _: std.fmt.FormatOptions, writer: anytype) !void {
            try writer.print("Transaction({d}) {s}({d}): {X:0>8} = {X:0>8}", .{
                self.cycle,
                @tagName(self.kind),
                self.size,
                self.addr,
                self.data,
            });
        }
    },
    opcode: u32,
    base_addr: u32,
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
    if (addr == state.test_data.base_addr) return state.test_data.opcode;

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

    cpu.change_mode(.User);
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

    cpu.spsr_fiq = @bitCast(t.initial.SPSR[0]);
    cpu.spsr_svc = @bitCast(t.initial.SPSR[1]);
    cpu.spsr_abt = @bitCast(t.initial.SPSR[2]);
    cpu.spsr_irq = @bitCast(t.initial.SPSR[3]);
    cpu.spsr_und = @bitCast(t.initial.SPSR[4]);

    cpu.set_cpsr(@bitCast(t.initial.CPSR));

    cpu.instruction_pipeline[0] = t.initial.pipeline[0];

    arm7.interpreter.execute(cpu, t.initial.pipeline[0]);
    cpu.r[15] +%= 4;

    // mul - Here the C flag should be set to a "meaningless value" per the doc.
    //       But NBA is very accurate and emulate its full behaviour. I don't want to do that.
    //       See https://bmchtech.github.io/post/multiply/
    if (arm7.JumpTable[arm7.ARM7.get_instr_tag(t.initial.pipeline[0])] == 7)
        cpu.cpsr.c = ((t.final.CPSR >> 29) & 1) == 1;

    try std.testing.expectEqual(t.final.CPSR, @as(u32, @bitCast(cpu.cpsr)));

    const cpsr_backup = cpu.cpsr;
    cpu.change_mode(.User); // Simplifies checking the results
    defer cpu.cpsr = cpsr_backup;

    try std.testing.expectEqualSlices(u32, t.final.R, &cpu.r);

    try std.testing.expectEqualSlices(u32, t.final.R_fiq[0..5], &cpu.r_fiq_8_12);
    try std.testing.expectEqualSlices(u32, t.final.R_fiq[5..], &cpu.r_fiq);
    try std.testing.expectEqualSlices(u32, t.final.R_svc, &cpu.r_svc);
    try std.testing.expectEqualSlices(u32, t.final.R_abt, &cpu.r_abt);
    try std.testing.expectEqualSlices(u32, t.final.R_irq, &cpu.r_irq);
    try std.testing.expectEqualSlices(u32, t.final.R_und, &cpu.r_und);

    try std.testing.expectEqual(t.final.SPSR[0], @as(u32, @bitCast(cpu.spsr_for(.FastInterrupt).*)));
    try std.testing.expectEqual(t.final.SPSR[1], @as(u32, @bitCast(cpu.spsr_for(.Supervisor).*)));
    try std.testing.expectEqual(t.final.SPSR[2], @as(u32, @bitCast(cpu.spsr_for(.Abort).*)));
    try std.testing.expectEqual(t.final.SPSR[3], @as(u32, @bitCast(cpu.spsr_for(.Interrupt).*)));
    try std.testing.expectEqual(t.final.SPSR[4], @as(u32, @bitCast(cpu.spsr_for(.Undefined).*)));

    if (ts.failed) return error.IOFail;
}

fn test_file(filepath: []const u8) !struct {
    tests_count: usize,
    evaluated_cases: usize,
    failed_cases: usize,
    skipped_cases: usize,
} {
    const BailOnFailure = false;
    const PrintAll = false;
    const basename = std.fs.path.basename(filepath);

    const mem = try std.testing.allocator.alignedAlloc(u8, .@"32", 0x40000);
    defer std.testing.allocator.free(mem);

    var cpu = arm7.ARM7.init(mem, 0x00000000, 0xFFFFFFFF);

    const data = try std.fs.cwd().readFileAlloc(std.testing.allocator, filepath, 512 * 1024 * 1024);
    defer std.testing.allocator.free(data);

    const test_data = try std.json.parseFromSlice([]Test, std.testing.allocator, data, .{});
    defer test_data.deinit();

    var failed_test_cases: usize = 0;
    var skipped_test_cases: usize = 0;
    var evaluated_test_cases: usize = 0;
    for (test_data.value) |t| {
        evaluated_test_cases += 1;
        // std.debug.print(" Testing: {s} ({X:0>8})\n", .{ arm7.ARM7.disassemble(t.initial.pipeline[0]), t.initial.pipeline[0] });

        if (!arm7.interpreter.is_valid(t.opcode)) {
            // std.debug.print(yellow("! Skipping {s} ({X:0>8})\n"), .{ arm7.ARM7.disassemble(t.opcode), t.opcode });
            skipped_test_cases += 1;
            continue;
        }

        run_test(t, &cpu) catch |err| {
            std.debug.print(red("[{s}] Test #{d} '{s}' ({X:0>8}) failed: {s}\n"), .{ basename, evaluated_test_cases, arm7.ARM7.disassemble(t.initial.pipeline[0]), t.initial.pipeline[0], @errorName(err) });
            if (PrintAll or failed_test_cases == 0) {
                std.debug.print("  ==== Initial state ==== \n", .{});
                t.initial.log();
                std.debug.print("  ======================= \n", .{});
                for (t.transactions) |tr| {
                    std.debug.print("   {any}\n", .{tr});
                }
                compare_state(&cpu, &t.final);
            }
            failed_test_cases += 1;
            if (BailOnFailure) return error.Failure;
            // break;
        };
    }
    if (failed_test_cases > 0) {
        std.debug.print(red("[{s}] {d}/{d} ({d} total) test cases failed.\n"), .{ basename, failed_test_cases, evaluated_test_cases, test_data.value.len });
    } else {
        std.debug.print(green("[{s}] {d}/{d} ({d} total) test cases passed.\n"), .{ basename, evaluated_test_cases, evaluated_test_cases, test_data.value.len });
    }
    return .{
        .tests_count = test_data.value.len,
        .evaluated_cases = evaluated_test_cases,
        .failed_cases = failed_test_cases,
        .skipped_cases = skipped_test_cases,
    };
}

const TestDir = "../ARM7TDMI/v1/";

test "All Files" {
    if (true) return error.SkipZigTest;

    var test_dir = try std.fs.cwd().openDir(TestDir, .{ .iterate = true });
    defer test_dir.close();

    var walker = try test_dir.walk(std.testing.allocator);
    defer walker.deinit();

    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();
    var arena_allocator = arena.allocator();

    var skipped_tests: usize = 0;
    var failed_tests: usize = 0;
    var results = std.ArrayList(struct {
        instruction: []const u8,
        tests_count: usize,
        evaluated_cases: usize = 0,
        skipped_cases: usize = 0,
        failed_cases: usize = 0,
    }).init(std.testing.allocator);
    defer results.deinit();

    var file_num: u32 = 0;
    tests_loop: while (try walker.next()) |entry| {
        if (entry.kind == .file and std.mem.startsWith(u8, entry.basename, "arm_") and std.mem.endsWith(u8, entry.basename, ".json")) {
            for ([_][]const u8{
                // Skipped tests
                "arm_bx.json", // Unimplemented
                "arm_cdp.json", // Unimplemented
                "arm_ldrh_strh.json", // Unimplemented
                "arm_ldrsb_ldrsh.json", // Unimplemented
                "arm_mull_mlal.json", // Unimplemented
                "arm_mcr_mrc.json", // Unimplemented
                "arm_stc_ldc.json", // Unimplemented CoprocessorDataTransfer
                "arm_undefined.json",
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

            const r = try test_file(fullpath);
            if (r.failed_cases > 0) failed_tests += 1;

            try results.append(.{
                .instruction = try arena_allocator.dupe(u8, entry.basename),
                .tests_count = r.tests_count,
                .evaluated_cases = r.evaluated_cases,
                .failed_cases = r.failed_cases,
                .skipped_cases = r.skipped_cases,
            });
        }
    }

    if (skipped_tests > 0)
        std.debug.print(yellow("Skipped {d} tests.\n"), .{skipped_tests});
    if (failed_tests > 0)
        std.debug.print(red("{d}/{d} tests failed.\n"), .{ failed_tests, file_num });

    for (results.items) |f| {
        if (f.failed_cases > 0) {
            std.debug.print(red(" {s: <30}: {d: >3}/{d} test cases failed ({d} skipped).\n"), .{ f.instruction, f.failed_cases, f.evaluated_cases, f.skipped_cases });
        } else if (f.skipped_cases > 0) {
            std.debug.print(yellow(" {s: <30}: {d: >3}/{d} test cases skipped.\n"), .{ f.instruction, f.skipped_cases, f.evaluated_cases });
        } else {
            std.debug.print(green(" {s: <30}: All {d} test cases passed.\n"), .{ f.instruction, f.evaluated_cases });
        }
    }
    if (results.items.len > 0)
        return error.TestFailed;
}

test "arm_b_bl" {
    const r = try test_file(TestDir ++ "arm_b_bl.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_data_proc_immediate" {
    const r = try test_file(TestDir ++ "arm_data_proc_immediate.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_data_proc_immediate_shift" {
    const r = try test_file(TestDir ++ "arm_data_proc_immediate_shift.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_data_proc_register_shift" {
    const r = try test_file(TestDir ++ "arm_data_proc_register_shift.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_ldm_stm" {
    const r = try test_file(TestDir ++ "arm_ldm_stm.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_ldr_str_immediate_offset" {
    const r = try test_file(TestDir ++ "arm_ldr_str_immediate_offset.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_ldr_str_register_offset" {
    const r = try test_file(TestDir ++ "arm_ldr_str_register_offset.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_mul_mla" {
    const r = try test_file(TestDir ++ "arm_mul_mla.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_mrs" {
    const r = try test_file(TestDir ++ "arm_mrs.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_msr_imm" {
    const r = try test_file(TestDir ++ "arm_msr_imm.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_msr_reg" {
    if (true) return error.SkipZigTest;
    const r = try test_file(TestDir ++ "arm_msr_reg.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_swi" {
    const r = try test_file(TestDir ++ "arm_swi.json");
    if (r.failed_cases > 0) return error.TestFailed;
}

test "arm_swp" {
    const r = try test_file(TestDir ++ "arm_swp.json");
    if (r.failed_cases > 0) return error.TestFailed;
}
