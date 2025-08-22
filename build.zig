const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const arm7_module = b.addModule("arm7", .{
        .root_source_file = b.path("src/arm7.zig"),
        .target = target,
        .optimize = optimize,
    });

    const lib = b.addLibrary(.{
        .name = "arm7",
        .root_module = arm7_module,
        .linkage = .static,
    });
    b.installArtifact(lib);

    const lib_unit_tests_module = b.addModule("arm7_tests", .{
        .root_source_file = b.path("src/arm7.zig"),
        .target = target,
        .optimize = optimize,
    });
    const lib_unit_tests = b.addTest(.{
        .name = "arm7_tests",
        .root_module = lib_unit_tests_module,
    });
    b.installArtifact(lib_unit_tests);
    const run_lib_unit_tests = b.addRunArtifact(lib_unit_tests);

    const run_asm_tests = add_test(b, "arm7_asm_tests", "tests/arm7_asm_tests.zig", &lib.step, arm7_module, target, optimize);
    //const run_armwrestler_dc_tests = add_test(b, "armwrestler_dc", "tests/armwrestler_dc.zig", &lib.step, arm7_module);

    // Exposes a `test` step to the `zig build --help` menu.
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_lib_unit_tests.step);
    test_step.dependOn(&run_asm_tests.step);
    //test_step.dependOn(&run_armwrestler_dc_tests.step);

    const sst_mod = b.addModule("arm7_tests", .{
        .root_source_file = b.path("src/arm7_tests.zig"),
        .target = target,
        .optimize = optimize,
    });
    const sst = b.addTest(.{ .root_module = sst_mod });
    const run_sst = b.addRunArtifact(sst);
    const sst_step = b.step("sst", "Run Single Step Tests");
    sst_step.dependOn(&run_sst.step);
}

fn add_test(b: *std.Build, comptime name: []const u8, comptime entry: []const u8, arm7_lib: *std.Build.Step, arm7_module: *std.Build.Module, target: std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode) *std.Build.Step.Run {
    const mod = b.addModule(name, .{
        .root_source_file = b.path(entry),
        .target = target,
        .optimize = optimize,
    });
    const exe = b.addTest(.{
        .name = name,
        .root_module = mod,
    });
    exe.root_module.addImport("arm7", arm7_module);

    const install_step = b.step(name, "Install " ++ name ++ " for debugging");
    install_step.dependOn(&b.addInstallArtifact(exe, .{}).step);
    install_step.dependOn(arm7_lib);
    return b.addRunArtifact(exe);
}
