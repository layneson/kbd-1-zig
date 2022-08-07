const std = @import("std");
const options = @import("options");

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

    var hid_endpoint_can_send = true;
    var c_on = false;

    while (true) {
        if (usb.getDeviceState() == .configured and hid_endpoint_can_send) {
            var packet = [1]u8{ 0 } ** 8;
            if (c_on) packet[1] = 0x06;
            usb.sendPacket(hid_endpoint_addr, &packet);
            hid_endpoint_can_send = false;
            c_on = !c_on;
        }

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
                    usb.sendPacket(0, &[_]u8{counter});
                    counter += 1;
                } else if (setup_packet.bmRequestType == 0x81 and
                    setup_packet.bRequest == 0x06 and
                    (setup_packet.wValue >> 8) == 0x22)
                {
                    // HID GET_DESCRIPTOR(Report)

                    usb.sendPacket(0, &report_descriptor);
                } else if (setup_packet.bmRequestType == 0x21 and
                    setup_packet.bRequest == 0x0A)
                {
                    // HID SET_IDLE

                    usb.sendPacket(0, &.{});
                } else if (setup_packet.bmRequestType == 0x21 and
                    setup_packet.bRequest == 0x0B)
                {
                    // HID SET_PROTOCOL
                    // We are not boot protocol, so we just pretend we did something.

                    usb.sendPacket(0, &.{});
                } else {
                    std.log.info("setup: reqtype={x} request={x} value_top={x}", .{
                        setup_packet.bmRequestType,
                        setup_packet.bRequest,
                        (setup_packet.wValue >> 8),
                    });
                }
            },
            .sent => |ep| {
                std.log.info("sent {d}", .{ ep });

                if (ep != hid_endpoint_addr) continue;

                hid_endpoint_can_send = true;
            },
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
            .bNumInterfaces = 2,
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
            .wDescriptorLength = report_descriptor.len,
        },
        .hid_endpoint = .{
            // Endpoint 1, IN
            .bEndpointAddress = 0b1_000_0000 | (@as(u8, 0x0F) & @as(u8, hid_endpoint_addr)),
            // Interrupt
            .bmAttributes = 0b000000_11,
            .wMaxPacketSize = report_length,
            // This is in milliseconds:
            .bInterval = 10,
        },
    },
    &.{
        "Ting", // 1
        "SuperTestThingy", // 2
        "SN0001", // 3
        "Vendor Thingy", // 4
        "StupidKeyboard", // 5
    },
    &.{
        .{
            .address = hid_endpoint_addr,
            .ep_type = .interrupt,
            .direction = .in,
            .max_packet_size = report_length,
        },
    },
);

const hid_endpoint_addr = 1;

// The report looks like the following (least-significant byte first):
//   <modifiers - 1 byte> <key code - 1 byte> * 7
const report_length = 8;

// This website can be used to verify the descriptor: https://eleccelerator.com/usbdescreqparser/.
const report_descriptor = [_]u8{
    0x0B, 0x06, 0x00, 0x01, 0x00, // Usage(Generic Desktop, Keyboard)
    0xA1, 0x01, // Collection(Application)
    0x06, 0x07, 0x00, //   Usage Page(Keyboard Keys)
    0x19, 0xE0, //   Usage Minimum(0xE0)
    0x29, 0xE7, //   Usage Maximum(0xE7)
    0x15, 0x00, //   Logical Minimum(0x00)
    0x25, 0x01, //   Logical Maximum(0x01)
    0x75, 0x01, //   Report Size(1)
    0x95, 0x08, //   Report Count(8)
    0x82, 0x02, 0x00, //   Input(Data, Variable, Absolute)
    0x19, 0x00, //   Usage Minimum(0x00)
    0x29, 0x65, //   Usage Maximum(0x65)
    0x15, 0x00, //   Logical Minimum(0x00)
    0x25, 0x65, //   Logical Maximum(0x65)
    0x75, 0x08, //   Report Size(8)
    0x95, 0x07, //   Report Count(7)
    0x81, 0x00, //   Input(Data, Array, Absolute)
    0xC0, // End Collection
};

pub fn log(
    comptime level: std.log.Level,
    comptime scope: @TypeOf(.enum_literal),
    comptime format: []const u8,
    args: anytype,
) void {
    if (!options.enable_semihosting) return;

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
