const std = @import("std");
const Builder = std.build.Builder;

const the_cpu = &std.Target.arm.cpu.cortex_m0plus;

pub fn link(thing: *std.build.LibExeObjStep) void {
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

/// Build examples.
pub fn build(b: *Builder) void {
    // Point to the main file.
    const exe = b.addExecutable("kbd-1-fw", "src/kbd1.zig");
    link(exe);
    exe.install();
}
