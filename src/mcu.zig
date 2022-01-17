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
        usb,
    };

    pub fn enablePeripheralClock(peripheral: Peripheral) void {
        switch (peripheral) {
            .gpiob => setBits(&mcu.rcc.iopenr, 1, 1, 1),
            .gpioc => setBits(&mcu.rcc.iopenr, 2, 2, 1),
            .usart1 => setBits(&mcu.rcc.apb2enr, 14, 14, 1),
            .syscfg => setBits(&mcu.rcc.apb2enr, 0, 0, 1),
            .crs => setBits(&mcu.rcc.apb1enr, 27, 27, 1),
            .usb => setBits(&mcu.rcc.apb1enr, 23, 23, 1),
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
            setBits(&mcu.rcc.cfgr, 4, 7, switch (prescaler) {
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
            .b => if (pin < 8) &mcu.gpio_b.afrl else &mcu.gpio_b.afrh,
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

    pub fn writer(instance: Instance) Writer {
        return .{ .context = .{ .instance = instance } };
    }

    pub const Writer = std.io.Writer(WriterContext, WriterContext.WriteError, WriterContext.writeFn);

    pub const WriterContext = struct {
        instance: Instance,

        pub const WriteError = error{};

        pub fn writeFn(self: WriterContext, bytes: []const u8) WriteError!usize {
            write(self.instance, bytes);
            return bytes.len;
        }
    };
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

pub const UsbDeviceDescription = struct {
    endpoints: []const Endpoint,

    pub const Endpoint = struct {
        ep_type: EndpointType,
        direction: struct { in: bool, out: bool },
        max_packet_size: u16,
    };

    pub const EndpointType = enum {
        control,
        isochronous,
        bulk,
        interrupt,
    };
};

pub fn usb(comptime desc: UsbDeviceDescription) type {
    return struct {
        comptime {
            // Just want to check desc.

            if (desc.endpoints.len == 0) @compileError("endpoint 0 is required");
            if (!desc.endpoints[0].direction.in or !desc.endpoints[0].direction.out) @compileError("endpoint 0 must be bi-directional");
        }

        pub fn init() void {
            mcu.usb.cntr = 0;
            mcu.usb.btable = 0;
            mcu.usb.istr = 0;

            // Enable USB and disable all interrupts (since we use a polling model).
            mcu.usb.cntr = 0;

            // Enable the pullup on the DP line.
            // We can signal disconnect to the host by setting this back to 0.
            setBits16(&mcu.usb.bcdr, 15, 15, 1);
        }

        pub const PollResult = enum {
            none,
            reset,
            setup,
        };

        pub fn poll() PollResult {
            // We need to:
            //   (a) Sample ISTR once at the beginning;
            //   (b) Clear the bits we handle.

            const istr = mcu.usb.istr;

            if (getBits16(istr, 10, 10) == 1) {
                // Reset.

                setBits16(&mcu.usb.istr, 10, 10, 0);

                // Reset our global state.
                global_state = .{};

                // Set device address so that the USB peripheral responds to incoming data.
                // Also enable.
                setBits16(&mcu.usb.daddr, 0, 6, 0);
                setBits16(&mcu.usb.daddr, 7, 7, 1);

                setupEndpoints();

                return .reset;
            }

            if (getBits16(istr, 15, 15) == 1) {
                // CTR. A transaction was completed.
                // If SETUP, we need to respond automatically with config info.
                // If OUT, we need to hand the packet over to the user.
                // If IN, we need to tell the user so they can send another if they so please.

                // The EP_ID field is actually 4 bits ([0, 3]), but we constrain
                // to the 8 lowest ids since we only have 8 endpoints and we don't need any more.
                const ep = getBits16(istr, 0, 2);
                const ep_reg = endpointRegister(ep);

                var process_in: bool = false;
                var process_out_or_setup: bool = false;

                const direction = getBits16(istr, 4, 4);
                switch (direction) {
                    0 => {
                        // IN
                        process_in = true;
                    },
                    1 => {
                        // OUT or SETUP and possibly IN

                        process_out_or_setup = true;

                        if (getBits16(ep_reg.*, 7, 7) == 1) {
                            // Also IN
                            process_in = true;
                        }
                    },
                }

                if (process_out_or_setup) {
                    // Set status back to VALID. Do this at the end.
                    defer setBits16(ep_reg, 12, 13, 0b11);

                    // Clear CTR_RX.
                    // Datasheet wants us to do this before reading the packet.
                    setBits16(ep_reg, 15, 15, 0);

                    if (getBits16(ep_reg.*, 11, 11) == 1) {
                        // It is SETUP, not OUT!

                        const table_entry = getBdtBidirectionalEntry(ep);
                        const pbm_offset = table_entry.rx_addr;

                        // Count is only 10 bits of the rx_count register.
                        const packet_len = @intCast(u16, getBits16(table_entry.rx_count, 0, 9));

                        copyFromPbm(global_state.ep0_packet_buffer[0..packet_len], pbm_offset);

                        return .setup;
                    } else {
                        // TODO: Handle OUT.
                    }
                }

                if (process_in) {
                    // TODO: Handle IN. Read the datasheet section about how to properly handle.

                    // Clear CTR_TX.
                    setBits16(ep_reg, 7, 7, 0);
                }

            }

            return .none;
        }

        /// This must be called upon reset.
        fn setupEndpoints() void {
            // Set up endpoints.

            // Buffer descriptor table is 4 entries, 2 bytes each, for 8 total endpoints.
            comptime var packet_mem_top: u16 = 8 * 4 * 2;

            inline for (desc.endpoints) |ep, ep_idx| {
                // Packet buffer accesses must be 2-byte aligned.
                // Thus, the max packet size must be even. If it were odd, we might start a packet buffer on an odd offset, which is not allowed.
                if (ep.max_packet_size & 0b1 != 0) @compileError("endpoints must have even max_packet_size");

                const ep_addr = @intCast(u3, ep_idx);

                const ep_reg = endpointRegister(ep_addr);

                // Set the address for the endpoint.
                setBits16(ep_reg, 0, 3, @intCast(u4, ep_addr));

                // Set endpoint type.
                setBits16(ep_reg, 9, 10, switch (ep.ep_type) {
                    .control => 0b01,
                    .isochronous => 0b10,
                    .bulk => 0b00,
                    .interrupt => 0b11,
                });

                // Need to set up either send or receive.

                const table_entry = getBdtBidirectionalEntry(ep_addr);

                if (ep.direction.in or ep_addr == 0) {
                    table_entry.tx_addr = packet_mem_top;

                    // Set data toggle so that first packet is DATA0.
                    setBits16(ep_reg, 6, 6, 0);

                    // Set status to NAK since no data is buffered to be sent.
                    setBits16(ep_reg, 4, 5, 0b10);

                    packet_mem_top += ep.max_packet_size;
                }

                if (ep.direction.out or ep_addr == 0) {
                    table_entry.rx_addr = packet_mem_top;

                    // Set data toggle so that first packet is DATA0.
                    setBits16(ep_reg, 14, 14, 0);

                    // Need to set the table_entry.rx_count so that the USB peripheral knows how much space we've allocated for this packet.
                    // Complication: this is not as straightforward as just setting it to the max packet size.
                    //
                    // Following equations are from libopencm3 docs: (division rounds up!!!)
                    //   if size <= 62: BL_SIZE = 0 and NUM_BLOCK = (size / 2). Real size = 2 * NUM_BLOCK.
                    //   if ize > 62: BL_SIZE = 1 and NUM_BLOCK = ((size / 32) - 1). Real size = 32 * NUM_BLOCK.

                    comptime var bl_size: u1 = 0;
                    comptime var num_blocks: u5 = 0;
                    comptime var real_size: u16 = 0;

                    comptime {
                        if (ep.max_packet_size <= 62) {
                            bl_size = 0;
                            num_blocks = (ep.max_packet_size + 1) / 2;
                            real_size = @intCast(u16, num_blocks) * 2;
                        } else {
                            bl_size = 1;
                            num_blocks = ((ep.max_packet_size + 31) / 32) - 1;
                            real_size = @intCast(u16, num_blocks) * 32;
                        }

                        if (bl_size == 0 and num_blocks == 0) @compileError("usb: invalid receive buffer max size");
                        if (real_size > 1024) @compileError("usb: receive buffer max size too large");
                    }
                    
                    setBits16(&table_entry.rx_count, 10, 14, num_blocks);
                    setBits16(&table_entry.rx_count, 15, 15, bl_size);

                    // Set status to VALID so that we can receive packets.
                    setBits16(ep_reg, 12, 13, 0b11);

                    packet_mem_top += real_size;
                }
            }
        }

        /// Gets the register for the given endpoint address.
        fn endpointRegister(ep_addr: u3) *align(1) volatile u16 {
            return switch (ep_addr) {
                0 => &mcu.usb.ep0r,
                1 => &mcu.usb.ep1r,
                2 => &mcu.usb.ep2r,
                3 => &mcu.usb.ep3r,
                4 => &mcu.usb.ep4r,
                5 => &mcu.usb.ep5r,
                6 => &mcu.usb.ep6r,
                7 => &mcu.usb.ep7r,
            };
        }

        /// BDT == Buffer Descriptor Table
        const BdtBidirectionalEntry = packed struct {
            tx_addr: u16,
            tx_count: u16,

            rx_addr: u16,
            rx_count: u16,
        };

        fn getBdtBidirectionalEntry(ep: u3) *volatile BdtBidirectionalEntry {
            return @intToPtr(
                *volatile BdtBidirectionalEntry,
                mcu.usb_packet_memory_offset + @intCast(usize, ep) * @sizeOf(BdtBidirectionalEntry),
            );
        }

        /// Copies provided bytes to PBM.
        /// PBM only supports 8 and 16-bit access, so we need to make sure the compiler doesn't
        /// try to get fancy and produce word copies.
        /// 
        /// This is dest-source order since that's what Zig's mem.copy does.
        fn copyToPbm(dest_offset: u16, source: []const u8) void {
            const dest = @intToPtr([*]volatile u8, mcu.usb_packet_memory_offset + @intCast(usize, dest_offset));

            for (source) |source_byte, i| {
                dest[i] = source_byte;
            }
        }

        fn copyFromPbm(dest: []u8, source_offset: u16) void {
            const source = @intToPtr([*]volatile u8, mcu.usb_packet_memory_offset + @intCast(usize, source_offset));

            for (dest) |*dest_byte, i| {
                dest_byte.* = source[i];
            }
        }

        const State = struct {
            address: u8 = 0,
            setup_state: SetupState = .uninitialized,
            ep0_packet_buffer: [desc.endpoints[0].max_packet_size]u8 = undefined,
        };

        /// States for the device state setup state machine.
        const SetupState = enum {
            /// The host has not sent any SETUP packets yet.
            uninitialized,
        };

        var global_state: State = .{};
    };
}

pub fn delay(comptime f_cpu_hz: comptime_int, comptime delay_us: comptime_int) void {
    // On an m0+, nop takes 1 cycle, and then b.n can take 1 or 2 (the documentation link was broken).
    // Let's assume 2. Then each loop iteration is three cycles, or f_cpu / 3. Then we have f_cpu/3 iter/sec.
    // Thus, num_iterations = (f_cpu/3) * delay_us * (1/1_000_000). (iter/sec * usec * sec/usec = iter)

    const num_iterations = ((f_cpu_hz / 3) * delay_us) / 1_000_000;

    var i: u32 = 0;
    while (i < num_iterations) : (i += 1) {
        asm volatile ("nop");
    }
}

pub fn InclusiveBitsType(comptime start: comptime_int, comptime end: comptime_int) type {
    return std.meta.Int(.unsigned, end - start + 1);
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

// Copypasta from above for 16-bit registers (really just the USB peripheral)
// Inclusive on both sides (since that's how the datasheet does it).

pub fn setBits16(
    reg: *align(1) volatile u16,
    comptime start: u4,
    comptime end: u4,
    value: InclusiveBitsType(start, end),
) void {
    const mask = ~@as(InclusiveBitsType(start, end), 0);
    reg.* &= ~(@as(u16, mask) << start);
    reg.* |= (@as(u16, value) << start);
}

// Inclusive on both sides.
pub fn getBits16(
    reg_value: u16,
    comptime start: u4,
    comptime end: u4,
) InclusiveBitsType(start, end) {
    const mask = ~@as(InclusiveBitsType(start, end), 0);
    return @intCast(
        InclusiveBitsType(start, end),
        (reg_value & (@as(u16, mask) << start)) >> start,
    );
}

pub fn setBitsRuntime16(
    reg: *align(1) volatile u16,
    start: u4,
    end: u4,
    value: u16,
) void {
    const mask_bits = end - start + 1;
    const mask = (@as(u16, 1) << mask_bits) - 1;
    reg.* &= ~(mask << start);
    reg.* |= (value << start);
}

pub fn getBitsRuntime16(
    reg_value: u16,
    start: u4,
    end: u4,
) u32 {
    const mask_bits = end - start + 1;
    const mask = (@as(u16, 1) << mask_bits) - 1;
    return (reg_value & (mask << start)) >> start;
}
