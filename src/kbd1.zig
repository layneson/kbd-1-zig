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

    // UART
    // mcu.gpio.setMode(.b, 6, .alternate); // TX
    // mcu.gpio.setAlternateFunction(.b, 6, 0);
    // mcu.usart.init(.usart1, 16_000_000, 576000);
    // mcu.usart.start(.usart1, .{ .transmit = true });

    // USB SETUP

    // Set up HSI48 with USB SYNC trimming.
    mcu.syscfg.enableHsi48Vref();
    mcu.rcc.hsi48.setSource(.rc48);
    mcu.crs.enableUsbAutoTrim();
    mcu.rcc.hsi48.enableAndWaitUntilReady();

    mcu.rcc.enablePeripheralClock(.usb);

    usb.init();

    std.log.info("Starting loop.", .{});

    var counter: u8 = 0;

    while (true) {
        switch (usb.poll()) {
            .none => {},
            .reset => {},
            .setup => |ep| {
                if (ep != 0) continue;

                var recv_buffer: [64]u8 = undefined;
                const packet_len = usb.getPacketLength(ep);
                usb.readPacket(ep, recv_buffer[0..packet_len]);

                const setup_packet = @ptrCast(*usb_std.Setup, recv_buffer[0..packet_len]).*;

                if ((setup_packet.bmRequestType & 0b0110_0000) == 0b0100_0000 and setup_packet.bRequest == 2) {
                    usb.sendPacket(0, &.{});
                } else if ((setup_packet.bmRequestType & 0b0110_0000) == 0b0100_0000 and setup_packet.bRequest == 3) {
                    usb.sendPacket(0, &[_]u8 { counter });
                    counter += 1;
                } else {
                    std.log.info("setup: reqtype={x} request={x} value_top={x}", .{
                        setup_packet.bmRequestType,
                        setup_packet.bRequest,
                        (setup_packet.wValue >> 8),
                    });
                }
            },
            .sent => {},
            .received => {},
        }
    }
}

const OurUsbConfigurationDescriptor = packed struct {
    configuration_descriptor: usb_std.ConfigurationDescriptor,
    vendor_interface: usb_std.InterfaceDescriptor,

    hid_interface: usb_std.InterfaceDescriptor,
    hid_descriptor: usb_std.HidDescriptor,
    hid_descriptor_report_instance: usb_std.HidDescriptorReportInstance,
    // ... The actual HID report descriptor is not included in the configuration descriptor - see HID spec section 7.1...
    hid_endpoint: usb_std.EndpointDescriptor,
};

const usb = mcu.usb(
    .{
        .bcdUSB = 0x0200,
        .bDeviceClass = 0,
        .bDeviceSubClass = 0,
        .bDeviceProtocol = 0,
        .bMaxPacketSize0 = 64,
        .idVendor = 0x0483,
        .idProduct = 0x1111,
        .bcdDevice = 0x0100,
        .iManufacturer = 1,
        .iProduct = 2,
        .iSerialNumber = 3,
        .bNumConfigurations = 1,
    },
    OurUsbConfigurationDescriptor{
        .configuration_descriptor = .{
            .wTotalLength = @sizeOf(OurUsbConfigurationDescriptor),
            .bNumInterfaces = 1, // 2,
            .bConfigurationValue = 1,
            .iConfiguration = 0,
            .bmAttributes = 0x80,
            .bMaxPower = 0x50, // 120 mA
        },
        .vendor_interface = .{
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 0,
            .bInterfaceClass = 0xFF, // Vendor
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 0,
            .iInterface = 4,
        },
        
        .hid_interface = .{
            .bInterfaceNumber = 1,
            .bAlternateSetting = 0,
            .bNumEndpoints = 1,
            .bInterfaceClass = 3, // HID
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 0,
            .iInterface = 5,
        },
        .hid_descriptor = .{
            .bLength = @sizeOf(usb_std.HidDescriptor) + 1 * @sizeOf(usb_std.HidDescriptorReportInstance),
            .bcdHID = 0x0101,
            .bCountryCode = 0,
            .bNumDescriptors = 1,
        },
        .hid_descriptor_report_instance = .{
            .wDescriptorLength = todo_report_descriptor_length,
        },
        .hid_endpoint = .{
            // Endpoint 1, IN
            .bEndpointAddress = 0b1_000_0001,
            // Interrupt
            .bmAttributes = 0b000000_11,
            .wMaxPacketSize = todo_report_length,
            // This is in milliseconds:
            .bInterval = 10,
        },
    },
    &.{
        "Ting",             // 1
        "SuperTestThingy",  // 2
        "SN0001",           // 3
        "Vendor Thingy",    // 4
        "StupidKeyboard",   // 5
    },
    &.{},
);

const todo_report_descriptor_length = 10;
const todo_report_length = 8;

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

    // mcu.usart.writer(.usart1).print(level_prefix ++ scope_prefix ++ format ++ "\n", args) catch {};
    mcu.semihosting.print(level_prefix ++ scope_prefix ++ format ++ "\n", args);
}

pub const log_level = .debug;
