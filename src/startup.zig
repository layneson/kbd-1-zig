const std = @import("std");

const peripherals = @import("peripherals.zig");

const root = @import("root");
const main = root.main;

// Defined in the linker script.
extern var __text_end: u32;
extern var __data_start: u32;
extern var __data_size: u32;
extern var __bss_start: u32;
extern var __bss_size: u32;
//extern var __stack_top: u32;

export fn Reset_Handler() void {
    // copy data from flash to RAM
    const data_size = @ptrToInt(&__data_size);
    const data = @ptrCast([*]u8, &__data_start);
    const text_end = @ptrCast([*]u8, &__text_end);
    for (text_end[0..data_size]) |b, i| data[i] = b;
    // clear the bss
    const bss_size = @ptrToInt(&__bss_size);
    const bss = @ptrCast([*]u8, &__bss_start);
    for (bss[0..bss_size]) |*b| b.* = 0;

    main();
}

export fn BusyDummy_Handler() void {
    while (true) {}
}

export fn Dummy_Handler() void {}

extern fn NMI_Handler() void;
extern fn HardFault_Handler() void;
extern fn MemManage_Handler() void;
extern fn BusFault_Handler() void;
extern fn UsageFault_Handler() void;
extern fn SVC_Handler() void;
extern fn DebugMon_Handler() void;
extern fn PendSV_Handler() void;
extern fn SysTick_Handler() void;

const Isr = fn () callconv(.C) void;

export var vector_table linksection(".isr_vector") = [_]?Isr{
    Reset_Handler,
    BusyDummy_Handler, // NMI
    BusyDummy_Handler, // HardFault
    null,
    null,
    null,
    null,
    null,
    null,
    null,
    Dummy_Handler, // SVC
    null,
    null,
    Dummy_Handler, // PendSV
    Dummy_Handler, // SusTick
};

export var irq_table linksection(".irq_vector") = blk: {
    var irqs = [1]Isr{Dummy_Handler} ** std.meta.fields(peripherals.Interrupt).len;

    if (@hasDecl(root, "irq_vectors")) {
        const root_irqs = root.irq_vectors;

        for (std.meta.fields(peripherals.Interrupt)) |irq_field| {
            if (@hasField(@TypeOf(root_irqs), irq_field.name)) {
                irqs[irq_field.value] = @field(root_irqs, irq_field.name);
            }
        }
    }

    break :blk irqs;
};
