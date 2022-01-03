const std = @import("std");

const mcu = @import("peripherals.zig");

pub const rcc = struct {
    pub const hsi16 = struct {
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

    pub const hsi48 = struct {
        pub const Source = enum {
            rc48,
        };

        pub fn setSource(source: Source) void {
            setBits(&mcu.rcc.ccipr, 26, 26, switch (source) {
                .rc48 => 1,
            });
        }

        pub fn enable() void {
            setBits(&mcu.rcc.crrcr, 0, 0, 1);
        }

        pub fn ready() bool {
            return getBits(mcu.rcc.crrcr, 1, 1) == 1;
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

    pub const Peripheral = enum {
        gpiob,
        gpioc,
        usart1,
        syscfg,
        crs,
    };

    pub fn enablePeripheralClock(peripheral: Peripheral) void {
        switch (peripheral) {
            .gpiob => setBits(&mcu.rcc.iopenr, 1, 1, 1),
            .gpioc => setBits(&mcu.rcc.iopenr, 2, 2, 1),
            .usart1 => setBits(&mcu.rcc.apb2enr, 14, 14, 1),
            .syscfg => setBits(&mcu.rcc.apb2enr, 0, 0, 1),
            .crs => setBits(&mcu.rcc.apb1enr, 27, 27, 1),
        }
    }

    pub const apb = struct {
        pub const Prescaler = enum {
            none,

            pub fn toBits(self: Prescaler) u3 {
                return switch (self) {
                    .none => 0b000,
                };
            }
        };

        pub fn setApb1Prescaler(prescaler: Prescaler) void {
            setBits(&mcu.rcc.cfgr, 8, 10, prescaler.toBits());
        }

        pub fn setApb2Prescaler(prescaler: Prescaler) void {
            setBits(&mcu.rcc.cfgr, 11, 13, prescaler.toBits());
        }
    };

    pub const ahb = struct {
        pub const Prescaler = enum {
            none,
        };

        pub fn setPrescaler(prescaler: Prescaler) void {
            setBits(&mcu.rcc.cfgr, 4, 7, switch(prescaler) {
                .none => 0b0000,
            });
        }
    };
};

pub const gpio = struct {
    pub const Mode = enum {
        input,
        output,
        alternate,
    };
    
    pub const Port = enum {
        b,
        c,
    };

    pub fn setMode(port: Port, pin: u4, mode: Mode) void {
        const moder = switch (port) {
            .b => &mcu.gpio_b.moder,
            .c => &mcu.gpio_c.moder,
        };

        setBitsRuntime(moder, @intCast(u5, pin) * 2, @intCast(u5, pin) * 2 + 1, switch (mode) {
            .input => 0b00,
            .output => 0b01,
            .alternate => 0b10,
        });
    }

    pub fn setAlternateFunction(port: Port, pin: u4, alternate_function: u3) void {
        const reg = switch (port) {
            .b => if (pin < 0) &mcu.gpio_b.afrl else &mcu.gpio_b.afrh,
            .c => if (pin < 8) &mcu.gpio_c.afrl else &mcu.gpio_c.afrh,
        };

        const offset = @as(u5, pin % 8) * 4;

        setBitsRuntime(reg, offset, offset + 3, alternate_function);
    }

    pub fn set(port: Port, pin: u4) void {
        const bsrr = switch (port) {
            .b => &mcu.gpio_b.bsrr,
            .c => &mcu.gpio_c.bsrr,
        };

        setBitsRuntime(bsrr, pin, pin, 1);
    }

    pub fn clear(port: Port, pin: u4) void {
        const bsrr = switch (port) {
            .b => &mcu.gpio_b.bsrr,
            .c => &mcu.gpio_c.bsrr,
        };

        setBitsRuntime(bsrr, @intCast(u5, pin) + 16, @intCast(u5, pin) + 16, 1);
    }
};

pub const usart = struct {
    pub const Instance = enum {
        usart1,
    };
    
    pub fn init(
        comptime instance: Instance, 
        comptime apb_clock_rate: comptime_int,
        comptime baud_rate: comptime_int,
    ) void {
        const u = switch (instance) {
            .usart1 => mcu.usart1,
        };

        // Oversampling of 16.
        // BRR = USARTDIV.
        const usartdiv: u16 = apb_clock_rate / baud_rate;

        u.brr = usartdiv;

        // Set stop bits.
        setBits(&u.cr2, 12, 13, 0b00);

        // Enable.
        setBits(&u.cr1, 0, 0, 1);
    }

    pub const StartOptions = struct {
        transmit: bool = false,
        receive: bool = false,
    };

    pub fn start(instance: Instance, options: StartOptions) void {
        const u = switch (instance) {
            .usart1 => mcu.usart1,
        };

        if (options.transmit) setBits(&u.cr1, 3, 3, 1);
        if (options.receive) setBits(&u.cr1, 2, 2, 1);
    }

    pub fn write(instance: Instance, buffer: []const u8) void {
        const u = switch (instance) {
            .usart1 => mcu.usart1,
        };

        for (buffer) |b| {
            while (getBits(u.isr, 7, 7) != 1) {}
            u.tdr = b;
            while (getBits(u.isr, 6, 6) != 1) {}
        }
    }
};

pub const syscfg = struct {
    pub fn enableHsi48Vref() void {
        setBits(&mcu.syscfg.cfgr3, 13, 13, 1);
    }
};

pub const crs = struct {
    pub fn enableUsbAutoTrim() void {
        setBits(&mcu.crs.cfgr, 28, 29, 0b10);
        setBits(&mcu.crs.cr, 6, 6, 1);
        setBits(&mcu.crs.cr, 5, 5, 1);
    }
};

pub fn delay(comptime f_cpu_hz: comptime_int, comptime delay_us: comptime_int) void {
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
