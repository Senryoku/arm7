const std = @import("std");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    const lib = b.addStaticLibrary(.{
        .name = "arm7",
        // In this case the main source file is merely a path, however, in more
        // complicated build scripts, this could be a generated file.
        .root_source_file = .{ .path = "src/arm7.zig" },
        .target = target,
        .optimize = optimize,
    });
    // This declares intent for the library to be installed into the standard
    // location when the user invokes the "install" step (the default step when
    // running `zig build`).
    b.installArtifact(lib);

    const arm7_module = b.createModule(.{ .root_source_file = .{ .path = "src/arm7.zig" } });

    // Creates a step for unit testing. This only builds the test executable
    // but does not run it.
    const lib_unit_tests = b.addTest(.{
        .name = "arm7_tests",
        .root_source_file = .{ .path = "src/arm7.zig" },
        .target = target,
        .optimize = optimize,
    });
    b.installArtifact(lib_unit_tests);
    const run_lib_unit_tests = b.addRunArtifact(lib_unit_tests);

    const run_asm_tests = add_test(b, "arm7_asm_tests", "tests/arm7_asm_tests.zig", &lib.step, arm7_module, target, optimize);
    //const run_armwrestler_dc_tests = add_test(b, "armwrestler_dc", "tests/armwrestler_dc.zig", &lib.step, arm7_module);

    // Similar to creating the run step earlier, this exposes a `test` step to
    // the `zig build --help` menu, providing a way for the user to request
    // running the unit tests.
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_lib_unit_tests.step);
    test_step.dependOn(&run_asm_tests.step);
    //test_step.dependOn(&run_armwrestler_dc_tests.step);
}

fn add_test(b: *std.Build, comptime name: []const u8, comptime entry: []const u8, arm7_lib: *std.Build.Step, arm7_module: *std.Build.Module, target: std.Build.ResolvedTarget, optimize: std.builtin.OptimizeMode) *std.Build.Step.Run {
    const exe = b.addTest(.{
        .name = name,
        .root_source_file = .{ .path = entry },
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.addImport("arm7", arm7_module);

    const install_step = b.step(name, "Install " ++ name ++ " for debugging");
    install_step.dependOn(&b.addInstallArtifact(exe, .{}).step);
    install_step.dependOn(arm7_lib);
    return b.addRunArtifact(exe);
}
