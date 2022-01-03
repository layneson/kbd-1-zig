pub const rcc = @intToPtr(*volatile packed struct {
    cr: u32,
    _reserved0: u32,
    crrcr: u32,
    cfgr: u32,
    _reserved2: u32,
    _reserved3: u32,
    _reserved4: u32,
    _reserved5: u32,
    _reserved6: u32,
    _reserved7: u32,
    _reserved8: u32,
    iopenr: u32,
    _reserved9: u32,
    apb2enr: u32,
    apb1enr: u32,
    _reserved11: u32,
    _reserved12: u32,
    _reserved13: u32,
    _reserved14: u32,
    ccipr: u32,
}, 0x4002_1000);

pub const Gpio = packed struct {
    moder: u32,
    _reserved0: u32,
    _reserved1: u32,
    _reserved2: u32,
    _reserved3: u32,
    _reserved4: u32,
    bsrr: u32,
    _reserved5: u32,
    afrl: u32,
    afrh: u32,
};
pub const gpio_b = @intToPtr(*volatile Gpio, 0x5000_0400);
pub const gpio_c = @intToPtr(*volatile Gpio, 0x5000_0800);

pub const Usart = packed struct {
    cr1: u32,
    cr2: u32,
    _reserved1: u32,
    brr: u32,
    _reserved2: u32,
    _reserved3: u32,
    _reserved4: u32,
    isr: u32,
    _reserved5: u32,
    _reserved6: u32,
    tdr: u32,
};
pub const usart1 = @intToPtr(*volatile Usart, 0x4001_3800);

pub const syscfg = @intToPtr(*volatile packed struct{
    _reserved0: u32,
    _reserved1: u32,
    _reserved2: u32,
    _reserved3: u32,
    _reserved4: u32,
    _reserved5: u32,
    _reserved6: u32,
    _reserved7: u32,
    cfgr3: u32,
}, 0x4001_0000);

pub const crs = @intToPtr(*volatile packed struct{
    cr: u32,
    cfgr: u32,
}, 0x4000_6C00);

pub const Interrupt = enum {
    WWDG,
    PVD,
    RTC,
    _reserved0,
    RCC,
    EXTI0_1,
    EXTI2_3,
    EXTI4_15,
    TSC,
    DMA1_Channel1,
    DMA1_Channel2_3,
    DMA1_Channel4_7,
    ADC_COMP,
    LPTIM1,
    USART4_USART5,
    TIM2,
    TIM3,
    TIM6_DAC,
    TIM7,
    _reserved1,
    TIM21,
    I2C3,
    TIM22,
    I2C1,
    I2C2,
    SPI1,
    SPI2,
    USART1,
    USART2,
    AES_RNG_LPUART1,
    _reserved2,
    USB,
};
