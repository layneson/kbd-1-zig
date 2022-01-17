const std = @import("std");

pub const Setup = packed struct {
    bmRequestType: u8,
    bRequest: u8,
    wValue: u16,
    wIndex: u16,
    wLength: u16,
};

pub const DeviceDescriptor = packed struct {
    bLength: u8,
    bDescriptorType: u8,
    bcdUSB: u16,
    bDeviceClass: u8,
    bDeviceSubClass: u8,
    bDeviceProtocol: u8,
    bMaxPacketSize0: u8,
    idVendor: u16,
    idProduct: u16,
    bcdDevice: u16,
    iManufacturer: u8,
    iProduct: u8,
    iSerialNumber: u8,
    bNumConfigurations: u8,
};

// ZIG: 01 01 02 02 00 00 40 40 04 04 11 11 01 01 00 00 01 01
// CM3: 12 01 00 02 00 00 00 40 83 04 11 11 00 02 01 02 03 01