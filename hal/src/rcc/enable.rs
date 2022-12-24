use super::*;


macro_rules! bus_enable {
    ($PER:ident => $bit:literal) => {
        impl Enable for crate::pac::$PER {
            #[inline(always)]
            fn enable(rcc: &RccRB) {
                unsafe {
                    Self::Bus::enr(rcc).modify(|r,w| w.bits(r.bits() | 1 << $bit));
                }
            }
            #[inline(always)]
            fn disable(rcc: &RccRB) {
                unsafe {
                    Self::Bus::enr(rcc).modify(|r,w| w.bits(r.bits() & (!(1 << $bit))));
                }
            }
        }
    };
}

macro_rules! bus_reset {
    ($PER:ident => $bit:literal) => {
        impl Reset for crate::pac::$PER {
            #[inline(always)]
            fn reset(rcc: &RccRB) {
                unsafe {
                    Self::Bus::rstr(rcc).modify(|r,w| w.bits(r.bits() | 1 << $bit));
                    Self::Bus::rstr(rcc).modify(|r,w| w.bits(r.bits() & (!(1 << $bit))));
                }
            }
        }
    };
}

macro_rules! bus {
    ($($PER:ident => ($busX:ty, $bit:literal),)+) => {
        $(
            impl crate::Sealed for crate::pac::$PER {}
            impl RccBus for crate::pac::$PER {
                type Bus = $busX;
            }
            bus_enable!($PER => $bit);
            bus_reset!($PER => $bit);
        )+
    }
}

bus! {
    CRC => (AHB1, 6),
    DMA1 => (AHB1, 0),
    DMA2 => (AHB1, 1),
}

bus! {
    GPIOA => (APB2, 2),
    GPIOB => (APB2, 3),
    GPIOC => (APB2, 4),
}

#[cfg(any(feature = "gpiod", feature = "gpioe"))]
bus! {
    GPIOD => (APB2, 5),
    GPIOE => (APB2, 6),
}


#[cfg(feature = "rng")]
bus! {
    RNG => (AHB1, 9),
}

#[cfg(feature = "otg-fs")]
bus! {
    OTG_FS_GLOBAL => (AHB1, 12),
}

#[cfg(feature = "fsmc")]
bus! {
    FSMC => (AHB1, 8),
}

bus! {
    PWR => (APB1, 28),
}

bus! {
    SPI1 => (APB2, 12),
    SPI2 => (APB1, 14),
}
#[cfg(feature = "spi3")]
bus! {
    SPI3 => (APB1, 15),
}


bus! {
    I2C1 => (APB1, 21),
    I2C2 => (APB1, 22),
}

bus! {
    USART1 => (APB2, 14),
    USART2 => (APB1, 17),
    UART6 => (APB1, 6),
}
#[cfg(feature = "usart3")]
bus! {
    USART3 => (APB1, 18),
}

#[cfg(any(feature = "usart4", feature = "usart5"))]
bus! {
    UART4 => (APB1, 19),
    UART5 => (APB1, 20),
}

#[cfg(any(feature = "uart7", feature = "uart8"))]
bus! {
    UART7 => (APB1, 7),
    UART8 => (APB1, 8),
}

#[cfg(any(feature = "can1", feature = "can2"))]
bus! {
    CAN1 => (APB1, 25),
    CAN2 => (APB1, 26),
}


#[cfg(feature = "dac")]
bus! {
    DAC => (APB1, 29),
}



bus! {
    ADC1 => (APB2, 9),
}

#[cfg(feature = "adc2")]
bus! {
    ADC2 => (APB2, 10),
}

#[cfg(feature = "sdio")]
bus! {
    SDIO => (AHB1, 10),
}

bus! {
    TIM1 => (APB2, 11),
    TIM5 => (APB1, 3),
    TIM9 => (APB2, 19),
}

bus! {
    TIM2 => (APB1, 0),
    TIM3 => (APB1, 1),
    TIM4 => (APB1, 2),
    TIM10 => (APB2, 20),
    TIM8 => (APB2, 13),
}

// TODO The PAC crate doesn't contain TIM6/TIM7. Fix them when the pac crate support them
// bus! {
//     TIM6 => (APB1, 4),
//     TIM7 => (APB1, 5),
// }

