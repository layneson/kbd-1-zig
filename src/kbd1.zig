const std = @import("std");

const startup = @import("startup.zig");
const mcu = @import("mcu.zig");
const usb_std = @import("usb_std.zig");

// Force Zig to evaluate startup so that we get our exports.
comptime {
    _ = startup;
}

pub fn main() noreturn {
    mcu.rcc.hsi16.enableAndWaitUntilReady();
    mcu.rcc.sys.setSourceAndWaitUntilReady(.hsi);

    mcu.rcc.ahb.setPrescaler(.none);
    mcu.rcc.apb.setApb1Prescaler(.none);
    mcu.rcc.apb.setApb2Prescaler(.none);

    mcu.rcc.enablePeripheralClock(.gpiob);
    mcu.rcc.enablePeripheralClock(.gpioc);
    mcu.rcc.enablePeripheralClock(.usart1);
    mcu.rcc.enablePeripheralClock(.syscfg);
    mcu.rcc.enablePeripheralClock(.crs);

    // LED
    mcu.gpio.setMode(.c, 15, .output);

    // UART
    mcu.gpio.setMode(.b, 6, .alternate); // TX
    mcu.gpio.setAlternateFunction(.b, 6, 0);
    mcu.usart.init(.usart1, 16_000_000, 576000);
    mcu.usart.start(.usart1, .{ .transmit = true });

    // USB SETUP

    // Set up HSI48 with USB SYNC trimming.
    mcu.syscfg.enableHsi48Vref();
    mcu.rcc.hsi48.setSource(.rc48);
    mcu.crs.enableUsbAutoTrim();
    mcu.rcc.hsi48.enableAndWaitUntilReady();

    mcu.rcc.enablePeripheralClock(.usb);

    usb.init();

    mcu.gpio.set(.c, 15);
    mcu.delay(16_000_000, 500_000);
    mcu.gpio.clear(.c, 15);

    var counter: u8 = 0;

    while (true) {
        switch (usb.poll()) {
            .none => {},
            .reset => {},
            .setup => {},
            .sent => {},
            .received => |ep| {
                if (ep != 0) continue;

                var recv_buffer: [64]u8 = undefined;
                const packet_len = usb.getPacketLength(ep);
                usb.readPacket(ep, recv_buffer[0..packet_len]);

                const setup_packet = @ptrCast(*usb_std.Setup, recv_buffer[0..packet_len]).*;

                if ((setup_packet.bmRequestType & 0b0110_0000) == 0b0100_0000 and setup_packet.bRequest == 2) {
                    usb.sendPacket(0, &.{});
                }

                if ((setup_packet.bmRequestType & 0b0110_0000) == 0b0100_0000 and setup_packet.bRequest == 3) {
                    usb.sendPacket(0, &[_]u8 { counter });
                    counter += 1;
                }
            },
        }
    }
}

// TODO: WE NEED TO CALCULATE TOTAL DESCRIPTOR LENGTH
// IN THE CONFIGURATION DESCRIPTOR. THIS INCLUDES
// VENDOR-DEFINED DESCRIPTORS. RIGHT NOW, THEY ARE
// NOT TAKEN INTO ACCOUNT! WE NEED TO ADJUST THIS
// WHEN DOING STUFF LIKE HID!
const usb = mcu.usb(.{
    .class = 0, // Defined by interface
    .subclass = 0,
    .protocol = 0,
    .vendor_id = 0x0483,
    .product_id = 0x1111,
    .manufacturer_descriptor_idx = 1,
    .product_descriptor_idx = 2,
    .serial_number_descriptor_idx = 3,

    .endpoints = &.{
        .{ // EP 0
            .ep_type = .control,
            .direction = .{ .in = true, .out = true },
            .max_packet_size = 64,
            .poll_interval = 0, // Ignored for this endpoint.
        },
    },

    .interfaces = &.{
        .{
            .endpoint_ids = &.{},
            .class = 0xFF, // vendor-defined.
            .subclass = 0,
            .protocol = 0,
            .interface_descriptor_idx = 4,
        },
    },

    .string_descriptors = &.{
        "Ting",
        "SuperTestThingy",
        "SN0001",
        "Vendor Thingy",
    },
});

pub fn log(
    comptime level: std.log.Level,
    comptime scope: @TypeOf(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    const level_prefix = "[" ++ switch (level) {
        .err => "E",
        .warn => "W",
        .info => "I",
        .debug => "D",
    } ++ "] ";

    const scope_name = @tagName(scope);
    const scope_prefix = if (scope == .default) "" else "(" ++ scope_name ++ ") ";

    mcu.usart.writer(.usart1).print(level_prefix ++ scope_prefix ++ format ++ "\n", args) catch {};
}

pub const log_level = .debug;
