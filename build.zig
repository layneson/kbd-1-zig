const std = @import("std");
const Builder = std.build.Builder;

const the_cpu = &std.Target.arm.cpu.cortex_m0plus;

fn link(thing: *std.build.LibExeObjStep) void {
    const target = std.zig.CrossTarget{
        .cpu_arch = .thumb,
        .os_tag = .freestanding,
        .cpu_model = .{ .explicit = the_cpu },
    };

    thing.setTarget(target);
    thing.setBuildMode(.ReleaseFast);
    //thing.strip = true;
    thing.setLinkerScriptPath(.{ .path = "stm32l052.ld" });
}

fn buildKbdspec2zig(b: *Builder) *std.build.LibExeObjStep {
    const exe = b.addExecutable("kbdspec2zig", "tools/kbdspec2zig.zig");
    exe.setTarget(.{});
    exe.setBuildMode(.Debug);
    return exe;
}

/// Build examples.
pub fn build(b: *Builder) void {
    const enable_semihosting = b.option(bool, "semihosting", "enable semihosting") orelse false;

    var options = b.addOptions();
    options.addOption(bool, "enable_semihosting", enable_semihosting);

    const kbdspec2zig = buildKbdspec2zig(b);
    var gen_matrix = GenMatrixStep.create(b, kbdspec2zig, "kb-1.kbdspec");

    const exe = b.addExecutable("kbd-1-fw", "src/kbd1.zig");
    link(exe);
    exe.step.dependOn(&options.step);
    exe.step.dependOn(&gen_matrix.step);
    exe.addPackage(options.getPackage("options"));
    exe.addPackage(.{
        .name = "kbdspec",
        .source = .{ .generated = &gen_matrix.out_file },
    });
    exe.install();
}

const GenMatrixStep = struct {
    step: std.build.Step,
    builder: *Builder,
    kbdspec2zig_artifact: *std.build.LibExeObjStep,
    in_file: []const u8,
    out_file: std.build.GeneratedFile,

    pub fn create(b: *Builder, kbdspec2zig_artifact: *std.build.LibExeObjStep, in_file: []const u8) *GenMatrixStep {
        const self = b.allocator.create(GenMatrixStep) catch unreachable;

        self.* = .{
            .step = std.build.Step.init(.custom, "gen_matrix", b.allocator, make),
            .builder = b,
            .kbdspec2zig_artifact = kbdspec2zig_artifact,
            .in_file = in_file,
            .out_file = undefined,
        };

        self.out_file = .{
            .step = &self.step,
            .path = null,
        };

        self.step.dependOn(&kbdspec2zig_artifact.step);

        return self;
    }

    // Hashing scheme inspired by lib/std/build/WriteFileStep.zig.
    pub fn make(step: *std.build.Step) !void {
        const self = @fieldParentPtr(GenMatrixStep, "step", step);

        const in_file_path = self.builder.pathFromRoot(self.in_file);

        var hash = std.crypto.hash.blake2.Blake2b384.init(.{});
        {
            // Random bytes to make GenMatrixStep unique. Refresh this with
            // new random bytes when GenMatrixStep implementation is modified
            // in a non-backwards-compatible way.
            hash.update("cEghV0qwk5XdHfFt");

            var input_file = try std.fs.cwd().openFile(in_file_path, .{ .mode = .read_only });
            defer input_file.close();

            const reader = input_file.reader();

            var read_buffer: [4096]u8 = undefined;

            while (true) {
                const bread = try reader.read(&read_buffer);
                if (bread == 0) break;

                hash.update(read_buffer[0..bread]);
            }
        }
        
        var digest: [48]u8 = undefined;
        hash.final(&digest);
        var hash_basename: [64]u8 = undefined;
        _ = std.fs.base64_encoder.encode(&hash_basename, &digest);
        const out_file_path = try std.fs.path.join(self.builder.allocator, &[_][]const u8{
            self.builder.cache_root,
            "o",
            &hash_basename,
        });
        self.out_file = std.build.GeneratedFile{
            .step = step,
            .path = out_file_path,
        };

        var run_step = self.kbdspec2zig_artifact.run();
        run_step.addArg(in_file_path);
        run_step.addArg(out_file_path);

        try run_step.step.make();
    }
};