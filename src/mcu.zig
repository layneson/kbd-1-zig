const std = @import("std");

const mcu = @import("peripherals.zig");
const usb_std = @import("usb_std.zig");

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

pub const UsbEndpointInfo = struct {
    address: u3,
    ep_type: enum {
        control,
        isochronous,
        bulk,
        interrupt,
    },
    direction: enum {
        in,
        out,
        control,
    },
    max_packet_size: u16,
};

/// - configuration_descriptor: A packed struct of the entire
///   configuration descriptor (including all other descriptors).
///
///   The first member must be of type usb_std.ConfigurationDescriptor.
///   Its wTotalLength should be set appropriately.
pub fn usb(
    comptime device_descriptor: usb_std.DeviceDescriptor,
    comptime configuration_descriptor: anytype,
    comptime string_descriptors: []const []const u8,
    comptime endpoint_info: []const UsbEndpointInfo,
) type {
    return struct {
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

        pub const PollResult = union(enum) {
            none,
            reset,
            setup: u3,
            received: u3,
            sent: u3,
        };

        pub fn poll() PollResult {
            const result = pollInternal();

            switch (result) {
                .setup => |ep| {
                    if (ep != 0) return result;

                    return handleSetup();
                },
                .received => |ep| {
                    if (ep != 0) return result;

                    if (global_state.device_state != .configured) {
                        handleInitOut();
                        return .{ .none = {} };
                    }

                    if (global_state.ep0_in_control_transfer) {
                        // Waiting for status stage of control transfer that the user is not involved in.

                        global_state.ep0_in_control_transfer = false;

                        // Set status back to VALID so that another packet can be received.
                        const ep_reg = endpointRegister(0);
                        ep_reg.setStatRx(0b11);

                        return .{ .none = {} };
                    }

                    return result;
                },
                .sent => |ep| if (ep == 0 and global_state.device_state != .configured) {
                    handleInitIn();
                    return .{ .none = {} };
                } else return result,
                else => return result,
            }
        }

        // This is separated from poll() so that we can intercept STATUS transfers on endpoint 0.
        fn pollInternal() PollResult {
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

                return .{ .reset = {} };
            }

            if (getBits16(istr, 15, 15) == 1) {
                // CTR. A transaction was completed.

                // The EP_ID field is actually 4 bits ([0, 3]), but we constrain
                // to the 8 lowest ids since we only have 8 endpoints and we don't need any more.
                const ep = getBits16(istr, 0, 2);
                const ep_reg = endpointRegister(ep);

                const direction = getBits16(istr, 4, 4);
                switch (direction) {
                    0 => {
                        // IN

                        // Clear CTR_TX.
                        ep_reg.clearCtrTx();

                        return .{ .sent = ep };
                    },
                    1 => {
                        // OUT or SETUP and possibly IN (although we won't handle that here).

                        // Clear CTR_RX.
                        // Datasheet wants us to do this before reading the packet.
                        ep_reg.clearCtrRx();

                        if (ep_reg.getSetup() == 1) {
                            // SETUP.
                            return .{ .setup = ep };
                        }

                        return .{ .received = ep };
                    },
                }
            }

            return .{ .none = {} };
        }

        /// Called internally when a SETUP packet has been received to endpoint 0.
        /// If it is not recognized as one which needs to be handled by this driver,
        /// it is returned to the user.
        fn handleSetup() PollResult {
            const ep_reg = endpointRegister(0);

            defer {
                // We are finished processing.
                // Set status back to VALID so that another packet can be received.
                ep_reg.setStatRx(0b11);
            }

            const table_entry = getBdtBidirectionalEntry(0);
            const pbm_offset = table_entry.rx_addr;

            // Count is only 10 bits of the rx_count register.
            const packet_len = @intCast(u16, getBits16(table_entry.rx_count, 0, 9));

            copyFromPbm(global_state.ep0_packet_buffer[0..packet_len], pbm_offset);

            const setup_packet = @ptrCast(*const usb_std.Setup, global_state.ep0_packet_buffer[0..packet_len]).*;

            // We may have disabled responding to IN requests in another life due to an unexpected packet.
            // Let's go back to the default (NAK).
            ep_reg.setStatTx(0b10);

            if (setup_packet.bmRequestType == 0b1000_0000 and
                setup_packet.bRequest == 6 and
                (setup_packet.wValue >> 8) == 1)
            {
                // GET_DESCRIPTOR (device).
                // Reply with device descriptor.

                const device_descriptor_bytes = std.mem.toBytes(device_descriptor);

                sendPacket(0, device_descriptor_bytes[0..std.math.min(device_descriptor_bytes.len, setup_packet.wLength)]);
            } else if (setup_packet.bmRequestType == 0 and
                setup_packet.bRequest == 5)
            {
                // SET_ADDRESS.

                global_state.address = @intCast(u8, setup_packet.wValue);

                // Status stage.
                sendPacket(0, &.{});

                global_state.should_set_address_on_next_status_stage = true;
            } else if (setup_packet.bmRequestType == 0b1000_0000 and
                setup_packet.bRequest == 6 and
                (setup_packet.wValue >> 8) == 2)
            {
                // GET_DESCRIPTOR (configuration).

                var configuration_descriptor_bytes = std.mem.toBytes(configuration_descriptor);

                sendPacket(0, configuration_descriptor_bytes[0..std.math.min(
                    @intCast(u16, configuration_descriptor_bytes.len),
                    setup_packet.wLength,
                )]);
            } else if (setup_packet.bmRequestType == 0b1000_0000 and
                setup_packet.bRequest == 6 and
                (setup_packet.wValue >> 8) == 3)
            {
                // GET_DESCRIPTOR (string).

                const descriptor_idx = @intCast(u8, setup_packet.wValue);

                if (descriptor_idx == 0) {
                    // Wants a language descriptor thingy.

                    const language_descriptor_bytes = [_]u8{ 4, 3, 0x09, 0x04 };
                    sendPacket(0, &language_descriptor_bytes);
                } else {
                    const string_idx = descriptor_idx - 1;

                    var descriptor_writer = std.io.fixedBufferStream(&global_state.ep0_packet_buffer).writer();

                    const string_utf16_len = @intCast(u8, string_descriptors[string_idx].len) * 2;

                    const string_descriptor: usb_std.StringDescriptor = .{
                        .bLength = @sizeOf(usb_std.StringDescriptor) + string_utf16_len,
                        .bDescriptorType = 3, // STRING
                    };

                    descriptor_writer.writeAll(&std.mem.toBytes(string_descriptor)) catch {};

                    for (string_descriptors[string_idx]) |char| {
                        descriptor_writer.writeByte(char) catch {};
                        descriptor_writer.writeByte(0) catch {};
                    }

                    sendPacket(0, global_state.ep0_packet_buffer[0..std.math.min(
                        @sizeOf(usb_std.StringDescriptor) + string_utf16_len,
                        setup_packet.wLength,
                    )]);
                }
            } else if (setup_packet.bmRequestType == 0b1000_0000 and
                setup_packet.bRequest == 0)
            {
                // GET_STATUS.

                const status = [_]u8{ 0, 0 };

                sendPacket(0, &status);
            } else if (setup_packet.bmRequestType == 0b0000_0000 and
                setup_packet.bRequest == 9)
            {
                // SET_CONFIGURATION.

                // Status stage.
                sendPacket(0, &.{});

                global_state.device_state = .configured;
            } else {
                // TODO: Do we need to do this now?
                // Disable responding to IN requests because we don't know what this is...
                // ep_reg.setStatTx(0b01);

                return .{ .setup = 0 };
            }

            return .{ .none = {} };
        }

        fn handleInitOut() void {
            const ep_reg = endpointRegister(0);

            // AFAIK, during our init dance, we don't ever have an OUT data stage (relevant host-to-device
            // data is contained within the STATUS packet).

            // Set status back to VALID so that another packet can be received.
            ep_reg.setStatRx(0b11);
        }

        fn handleInitIn() void {
            //const ep_reg = endpointRegister(0);

            // Success! If we got confirmation of IN, we just sent some data
            // and were waiting on confirmation. Let's finish that wait.

            // However, if we were waiting on confirmation of sending the status
            // stage of a control transfer that just set our address, we need to
            // finish the job.
            if (global_state.should_set_address_on_next_status_stage) {
                global_state.should_set_address_on_next_status_stage = false;

                // SET_ADDRESS.
                setBits16(&mcu.usb.daddr, 0, 6, @intCast(u7, global_state.address));

                global_state.device_state = .address;
            }
        }

        /// This must be called upon reset.
        fn setupEndpoints() void {
            // Set up endpoints.

            // Buffer descriptor table is 4 entries, 2 bytes each, for 8 total endpoints.
            comptime var packet_mem_top: u16 = 8 * 4 * 2;

            const endpoint_info_including_ep0 = &[1]UsbEndpointInfo{
                .{
                    .address = 0,
                    .ep_type = .control,
                    .direction = .control,
                    .max_packet_size = device_descriptor.bMaxPacketSize0,
                },
            } ++ endpoint_info;

            inline for (endpoint_info_including_ep0) |ep| {
                // Packet buffer accesses must be 2-byte aligned.
                // Thus, the max packet size must be even. If it were odd, we might start a packet buffer on an odd offset, which is not allowed.
                if (ep.max_packet_size & 0b1 != 0) @compileError("endpoints must have even max_packet_size");

                const ep_addr = ep.address;

                const ep_reg = endpointRegister(ep_addr);

                // Set the address for the endpoint.
                ep_reg.setEa(ep_addr);

                // Set endpoint type.
                ep_reg.setEpType(switch (ep.ep_type) {
                    .control => 0b01,
                    .isochronous => 0b10,
                    .bulk => 0b00,
                    .interrupt => 0b11,
                });

                // Need to set up either send or receive.

                const table_entry = getBdtBidirectionalEntry(ep_addr);
                table_entry.tx_count = 0;
                table_entry.rx_count = 0;

                if (ep.direction == .in or ep.direction == .control) {
                    table_entry.tx_addr = packet_mem_top;

                    // Set data toggle so that first packet is DATA0.
                    // TODO: Make sure to set this properly for non-control endpoints.
                    ep_reg.setDtogTx(0);

                    // Set status to NAK since no data is buffered to be sent.
                    ep_reg.setStatTx(0b10);

                    packet_mem_top += ep.max_packet_size;
                }

                if (ep.direction == .out or ep.direction == .control) {
                    table_entry.rx_addr = packet_mem_top;

                    // Set data toggle so that first packet is DATA0.
                    // TODO: Make sure to set this properly for non-control endpoints.
                    ep_reg.setDtogRx(0);

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
                    ep_reg.setStatRx(0b11);

                    packet_mem_top += real_size;
                }
            }
        }

        /// Only call this if poll() notifies of reception!
        /// Provides the length of the currently buffered incoming packet.
        /// Use this to know how big the buffer must be for readPacket().
        /// This can be called any number of times before readPacket().
        pub fn getPacketLength(ep_addr: u3) u16 {
            const table_entry = getBdtBidirectionalEntry(ep_addr);
            return @intCast(u16, getBits16(table_entry.rx_count, 0, 9));
        }

        /// Only call this if poll() notifies of reception!
        /// This signals to the peripheral that the packet has been read and can
        /// be internally overwritten by the next incoming packet. Thus, this
        /// can only be called once per incoming packet!
        ///
        /// To discard the packet, pass a zero-length buffer.
        pub fn readPacket(ep_addr: u3, buffer: []u8) void {
            const ep_reg = endpointRegister(ep_addr);
            const table_entry = getBdtBidirectionalEntry(ep_addr);

            // Count is only 10 bits of the rx_count register.
            const packet_len = @intCast(u16, getBits16(table_entry.rx_count, 0, 9));

            if (buffer.len > 0) {
                copyFromPbm(buffer[0..std.math.min(buffer.len, packet_len)], table_entry.rx_addr);
            }

            // Set status back to VALID so that another packet can be received.
            ep_reg.setStatRx(0b11);
        }

        /// Places this packet in the packet memory for this endpoint and signals
        /// that a packet is ready to send if the host provides an IN packet.
        ///
        /// Only call this once until poll() returns sent for this ep!
        pub fn sendPacket(ep_addr: u3, packet: []const u8) void {
            const ep_reg = endpointRegister(ep_addr);

            const table_entry = getBdtBidirectionalEntry(ep_addr);

            @call(.{}, copyToPbm, .{ table_entry.tx_addr, packet });
            table_entry.tx_count = @intCast(u16, packet.len);

            // Set status to VALID so that we can send the packet.
            ep_reg.setStatTx(0b11);
        }

        /// We abstract this because writing to this register is weird.
        /// There are a number of bits that are toggle-only, which is not how we want
        /// to use them (we want to just the the bits).
        const EndpointRegister = struct {
            reg: *align(1) volatile u16,

            /// Where should we write ones if we don't want to change value?
            const write_ones_mask: u16 = 0b1000_0000_1000_0000;
            /// Where should be write the read value if we don't want to change value?
            const write_original_mask: u16 = 0b0000_1111_0000_1111;

            /// Adapted from setBits16, but writes the correct value to perserve toggle & set-on-0 registers.
            /// Note that the correct toggle bits need to be set here!
            fn setEpnrBits(
                reg: *align(1) volatile u16,
                comptime start: u4,
                comptime end: u4,
                value: InclusiveBitsType(start, end),
            ) void {
                // Contains the bits that we write in everywhere we don't care about.
                const backing_value = (reg.* & write_original_mask) | (@as(u16, 0xFFFF) & write_ones_mask);

                const mask = ~@as(InclusiveBitsType(start, end), 0);

                reg.* = (backing_value & ~(@as(u16, mask) << start)) | (@as(u16, value) << start);
            }

            pub fn getCtrRx(self: EndpointRegister) u1 {
                return getBits16(self.reg.*, 15, 15);
            }

            pub fn clearCtrRx(self: EndpointRegister) void {
                setEpnrBits(self.reg, 15, 15, 0);
            }

            pub fn getDtogRx(self: EndpointRegister) u1 {
                return getBits16(self.reg.*, 14, 14);
            }

            pub fn setDtogRx(self: EndpointRegister, value: u1) void {
                setEpnrBits(self.reg, 15, 15, self.getDtogRx() ^ value);
            }

            pub fn getStatRx(self: EndpointRegister) u2 {
                return getBits16(self.reg.*, 12, 13);
            }

            pub fn setStatRx(self: EndpointRegister, value: u2) void {
                setEpnrBits(self.reg, 12, 13, self.getStatRx() ^ value);
            }

            pub fn getSetup(self: EndpointRegister) u1 {
                return getBits16(self.reg.*, 11, 11);
            }

            pub fn getEpType(self: EndpointRegister) u2 {
                return getBits16(self.reg.*, 9, 10);
            }

            pub fn setEpType(self: EndpointRegister, value: u2) void {
                setEpnrBits(self.reg, 9, 10, value);
            }

            pub fn getEpKind(self: EndpointRegister) u1 {
                return getBits16(self.reg.*, 8, 8);
            }

            pub fn setEpKind(self: EndpointRegister, value: u1) void {
                setEpnrBits(self.reg, 8, 8, value);
            }

            pub fn getCtrTx(self: EndpointRegister) u1 {
                return getBits16(self.reg.*, 7, 7);
            }

            pub fn clearCtrTx(self: EndpointRegister) void {
                setEpnrBits(self.reg, 7, 7, 0);
            }

            pub fn getDtogTx(self: EndpointRegister) u1 {
                return getBits16(self.reg.*, 6, 6);
            }

            pub fn setDtogTx(self: EndpointRegister, value: u1) void {
                setEpnrBits(self.reg, 6, 6, self.getDtogTx() ^ value);
            }

            pub fn getStatTx(self: EndpointRegister) u2 {
                return getBits16(self.reg.*, 4, 5);
            }

            pub fn setStatTx(self: EndpointRegister, value: u2) void {
                setEpnrBits(self.reg, 4, 5, self.getStatTx() ^ value);
            }

            pub fn getEa(self: EndpointRegister) u4 {
                return getBits16(self.reg.*, 0, 3);
            }

            pub fn setEa(self: EndpointRegister, value: u4) void {
                setEpnrBits(self.reg, 0, 3, value);
            }
        };

        /// Gets the register for the given endpoint address.
        fn endpointRegister(ep_addr: u3) EndpointRegister {
            return .{ .reg = switch (ep_addr) {
                0 => &mcu.usb.ep0r,
                1 => &mcu.usb.ep1r,
                2 => &mcu.usb.ep2r,
                3 => &mcu.usb.ep3r,
                4 => &mcu.usb.ep4r,
                5 => &mcu.usb.ep5r,
                6 => &mcu.usb.ep6r,
                7 => &mcu.usb.ep7r,
            } };
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
            const dest = @intToPtr([*]volatile u16, mcu.usb_packet_memory_offset + @intCast(usize, dest_offset));

            for (source) |source_byte, i| {
                if (i % 2 == 1) {
                    dest[i / 2] = @intCast(u16, source[i - 1]) | (@intCast(u16, source_byte) << 8);
                }
            }

            if (source.len % 2 == 1) {
                dest[source.len / 2] = (dest[source.len / 2] & 0xFF00) | source[source.len - 1];
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
            device_state: DeviceState = .default,
            should_set_address_on_next_status_stage: bool = false,
            ep0_in_control_transfer: bool = false,
            ep0_packet_buffer: [device_descriptor.bMaxPacketSize0]u8 = undefined,
        };

        /// States for the device state setup state machine.
        /// These are taken from the usb 2.0 spec.
        /// We don't include powered here because there is no difference between
        /// the powered and default states from our perspective.
        pub const DeviceState = enum {
            /// Device has been reset (or not, see note above) but not assigned an address.
            default,
            /// Device has an address but has not been configured.
            address,
            /// Device has been configured. Normal operation.
            configured,
        };

        pub fn getDeviceState() DeviceState {
            return global_state.device_state;
        }

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

pub const semihosting = struct {
    pub fn print(comptime fmt: []const u8, args: anytype) void {
        var writer = std.io.Writer(void, PrintRawError, printRaw){ .context = {} };
        writer.print(fmt, args) catch {};
    }

    const PrintRawError = error{};

    fn printRaw(_: void, str: []const u8) PrintRawError!usize {
        const sys_write_args = [3]u32{
            @as(u32, 2), // Stderr.
            @ptrToInt(str.ptr),
            str.len,
        };

        asm volatile ("BKPT 0xAB"
            :
            : [sh_op_type] "{R0}" (@as(u32, 0x05)),
              [sys_write_args] "{R1}" (@ptrToInt(&sys_write_args)),
        );

        return str.len;
    }
};
