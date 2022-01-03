const std = @import("std");

const startup = @import("startup.zig");
const mcu = @import("mcu.zig");

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

    while (true) {
        mcu.gpio.set(.c, 15);
        mcu.delay(16_000_000, 500_000);
        mcu.gpio.clear(.c, 15);
        mcu.delay(16_000_000, 500_000);

        mcu.usart.write(.usart1, "BLUBBER ");
    }
}
