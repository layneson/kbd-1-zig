const std = @import("std");

const startup = @import("startup.zig");
const mcu = @import("peripherals.zig");

// Force Zig to evaluate startup so that we get our exports.
comptime {
    _ = startup;
}

pub fn main() noreturn {
    rcc.hsi.enableAndWaitUntilReady();
    rcc.sys.setSourceAndWaitUntilReady(.hsi);

    rcc.gpio.enablePortC();

    gpio.setMode(.c, 15, .output);

    while (true) {
        gpio.set(.c, 15);
        delay(16_000_000, 500_000);
        gpio.clear(.c, 15);
        delay(16_000_000, 500_000);
    }
}

const rcc = struct {
    pub const hsi = struct {
        pub fn enable() void {
            setBits(&mcu.rcc.cr, 0, 0, 1);
        }

        pub fn ready() bool {
            return getBits(mcu.rcc.cr, 2, 2) == 1;
        }

        pub fn enableAndWaitUntilReady() void {
            enable();
            while (!ready()) {}
        }
    };

    pub const sys = struct {
        pub const Source = enum {
            msi,
            hsi,
            hse,
            pll,
        };

        pub fn setSource(source: Source) void {
            switch (source) {
                .msi => setBits(&mcu.rcc.cfgr, 0, 1, 0b00),
                .hsi => setBits(&mcu.rcc.cfgr, 0, 1, 0b01),
                .hse => setBits(&mcu.rcc.cfgr, 0, 1, 0b10),
                .pll => setBits(&mcu.rcc.cfgr, 0, 1, 0b11),
            }
        }

        pub fn getSource() Source {
            return switch (getBits(mcu.rcc.cfgr, 2, 3)) {
                0b00 => .msi,
                0b01 => .hsi,
                0b10 => .hse,
                0b11 => .pll,
            };
        }

        pub fn setSourceAndWaitUntilReady(source: Source) void {
            setSource(source);
            while (getSource() != source) {}
        }
    };

    const gpio = struct {
        pub fn enablePortC() void {
            setBits(&mcu.rcc.iopenr, 2, 2, 1);
        }
    };
};

const gpio = struct {
    pub const Mode = enum {
        input,
        output,
    };
    
    pub const Port = enum {
        c,
    };

    pub fn setMode(port: Port, pin: u4, mode: Mode) void {
        const moder = switch (port) {
            .c => &mcu.gpio_c.moder,
        };

        setBitsRuntime(moder, @intCast(u5, pin) * 2, @intCast(u5, pin) * 2 + 1, switch (mode) {
            .input => 0b00,
            .output => 0b01,
        });
    }

    pub fn set(port: Port, pin: u4) void {
        const bsrr = switch (port) {
            .c => &mcu.gpio_c.bsrr,
        };

        setBitsRuntime(bsrr, pin, pin, 1);
    }

    pub fn clear(port: Port, pin: u4) void {
        const bsrr = switch (port) {
            .c => &mcu.gpio_c.bsrr,
        };

        setBitsRuntime(bsrr, @intCast(u5, pin) + 16, @intCast(u5, pin) + 16, 1);
    }
};

fn delay(comptime f_cpu_hz: comptime_int, comptime delay_us: comptime_int) void {
    // On an m0+, nop takes 1 cycle, and then b.n can take 1 or 2 (the documentation link was broken).
    // Let's assume 2. Then each loop iteration is three cycles, or f_cpu / 3. Then we have f_cpu/3 iter/sec.
    // Thus, num_iterations = (f_cpu/3) * delay_us * (1/1_000_000). (iter/sec * usec * sec/usec = iter)

    const num_iterations = ((f_cpu_hz / 3) * delay_us) / 1_000_000;

    var i: u32 = 0;
    while (i < num_iterations) : (i += 1) {
        asm volatile (
            "nop"
        );
    }
}

pub fn InclusiveBitsType(comptime start: u5, comptime end: u5) type {
    return std.meta.Int(.unsigned, end-start+1);
}

// Inclusive on both sides (since that's how the datasheet does it).
pub fn setBits(
    reg: *align(1) volatile u32,
    comptime start: u5,
    comptime end: u5,
    value: InclusiveBitsType(start, end),
) void {
    const mask = ~@as(InclusiveBitsType(start, end), 0);
    reg.* &= ~(@as(u32, mask) << start);
    reg.* |= (@as(u32, value) << start);
}

// Inclusive on both sides.
pub fn getBits(
    reg_value: u32,
    comptime start: u5,
    comptime end: u5,
) InclusiveBitsType(start, end) {
    const mask = ~@as(InclusiveBitsType(start, end), 0);
    return @intCast(
        InclusiveBitsType(start, end),
        (reg_value & (@as(u32, mask) << start)) >> start,
    );
}

pub fn setBitsRuntime(
    reg: *align(1) volatile u32,
    start: u5,
    end: u5,
    value: u32,
) void {
    const mask_bits = end - start + 1;
    const mask = (@as(u32, 1) << mask_bits) - 1;
    reg.* &= ~(mask << start);
    reg.* |= (value << start);
}

pub fn getBitsRuntime(
    reg_value: u32,
    start: u5,
    end: u5,
) u32 {
    const mask_bits = end - start + 1;
    const mask = (@as(u32, 1) << mask_bits) - 1;
    return (reg_value & (mask << start)) >> start;
}
