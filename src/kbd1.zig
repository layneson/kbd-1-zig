const startup = @import("startup.zig");
const mcu = @import("peripherals.zig");

// Force Zig to evaluate startup so that we get our exports.
comptime {
    _ = startup;
}

pub fn main() noreturn {
    // The stm32l052 starts at 2MHz.
    // We want at least 16, which we can get from the HSI oscillator.

    // Set HSI16ON.
    mcu.rcc.cr |= 0b1;

    // Wait for HSI16RDYF.
    while (mcu.rcc.cr & 0b100 == 0) {}

    // Set system clock source to HSI.
    mcu.rcc.cfgr = (mcu.rcc.cfgr & 0xFF_FF_FF_FC) | 0x00_00_00_01;

    // Wait for HSI to become clock source.
    while ((mcu.rcc.cfgr & 0x00_00_00_0C) != 0x00_00_00_04) {}

    // Enable GPIO for the LED port (c).
    mcu.rcc.iopenr |= 0b100;

    // Set the led pin (15) to output.
    mcu.gpio_c.moder = (mcu.gpio_c.moder & 0x3F_FF_FF_FF) | 0x40_00_00_00;

    while (true) {
        // Set LED on.
        mcu.gpio_c.bsrr |= 0x00_00_80_00;

        delay(16_000_000, 500_000);

        // Set LED off.
        mcu.gpio_c.bsrr |= 0x80_00_00_00;

        delay(16_000_000, 500_000);
    }
    
   // stm32.rcc.hsi.enable();

    // while (!stm32.rcc.hsi.ready()) {}

    // stm32.rcc.system_clock.setSource(.hsi16);
    // while (stm32.rcc.system_clock.getSource() != .hsi16) {}

    // stm32.rcc.apb.setApb1Prescaler(.none);
    // stm32.rcc.apb.setApb2Prescaler(.none);

    // stm32.rcc.ahb.setPrescaler(.none);
    // while (stm32.rcc.ahb.getPrescaler() != .none) {}

    // stm32.rcc.gpio.enableClock(led_port);

    // stm32.gpio.setMode(led_port, led_pin, .output);

    // while(true) blink();
}

fn delay(comptime f_cpu_hz: comptime_int, comptime delay_us: comptime_int) void {
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
