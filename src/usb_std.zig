const std = @import("std");

pub const Setup = packed struct {
    bmRequestType: u8,
    bRequest: u8,
    wValue: u16,
    wIndex: u16,
    wLength: u16,
};

pub const DeviceDescriptor = packed struct {
    pub const DescriptorType: u8 = 1;

    bLength: u8 = @sizeOf(DeviceDescriptor),
    bDescriptorType: u8 = DescriptorType,
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

pub const ConfigurationDescriptor = packed struct {
    pub const DescriptorType: u8 = 2;

    bLength: u8 = @sizeOf(ConfigurationDescriptor),
    bDescriptorType: u8 = DescriptorType,
    wTotalLength: u16,
    bNumInterfaces: u8,
    bConfigurationValue: u8,
    iConfiguration: u8,
    bmAttributes: u8,
    bMaxPower: u8,
};

pub const InterfaceDescriptor = packed struct {
    pub const DescriptorType: u8 = 4;

    bLength: u8 = @sizeOf(InterfaceDescriptor),
    bDescriptorType: u8 = DescriptorType,
    bInterfaceNumber: u8,
    bAlternateSetting: u8,
    bNumEndpoints: u8,
    bInterfaceClass: u8,
    bInterfaceSubClass: u8,
    bInterfaceProtocol: u8,
    iInterface: u8,
};

pub const EndpointDescriptor = packed struct {
    pub const DescriptorType: u8 = 5;

    bLength: u8 = @sizeOf(EndpointDescriptor),
    bDescriptorType: u8 = DescriptorType,
    bEndpointAddress: u8,
    bmAttributes: u8,
    wMaxPacketSize: u16,
    bInterval: u8,
};

pub const StringDescriptor = packed struct {
    pub const DescriptorType: u8 = 3;

    bLength: u8,
    bDescriptorType: u8 = DescriptorType,
};

pub const HidDescriptor = packed struct {
    pub const DescriptorType: u8 = 0x21;

    bLength: u8,
    bDescriptorType: u8 = DescriptorType,
    bcdHID: u16, // Should probaby be 0x0101
    bCountryCode: u8, // 33 is US, but spec says should probably be 0.
    bNumDescriptors: u8,
};

/// Indicates the existence of a report descriptor.
/// These should be included immediately following an HidDescriptor.
pub const HidDescriptorReportInstance = packed struct {
    pub const DescriptorType: u8 = 0x22;

    bDescriptorType: u8 = DescriptorType,
    wDescriptorLength: u16,
};
