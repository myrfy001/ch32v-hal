

use crate::pac::RCC;
use crate::pac::EXTEND;

use super::consts;


pub struct MainPll {
    pub use_pll: bool,
    pub use_hse: bool,
    pub mul: u32,
    pub div: u32,

}

impl MainPll {
    pub const fn calc_fast_setup(
        in_clk: u32,
        use_hse: bool,
        out_clk: u32,
    ) -> Option<MainPll> {
        if use_hse {
            if let Some((mul_cfg, div_cfg)) = consts::calc_pll_param(in_clk, out_clk, &consts::MAIN_PLL_DIV_TABLE_HSE, &consts::MAIN_PLL_MUL_TABLE) {
                return Some(MainPll { use_pll: true, use_hse, mul: mul_cfg.cfg_bit as u32, div: div_cfg.cfg_bit as u32})
            } else {
                return None
            }
        } else {
            if let Some((mul_cfg, div_cfg)) = consts::calc_pll_param(8_000_000, out_clk, &consts::MAIN_PLL_DIV_TABLE_HSI, &consts::MAIN_PLL_MUL_TABLE) {
                return Some(MainPll { use_pll: true, use_hse, mul: mul_cfg.cfg_bit as u32, div: div_cfg.cfg_bit as u32 })
            } else {
                return None
            }
        }
    }

    pub fn fast_setup(&self) {
        if self.use_pll == false {
            return;
        }

        let rcc = unsafe { &*RCC::ptr() };
        
        rcc.cfgr0.modify(|_, w|unsafe{w
            .pllsrc().bit(self.use_hse)  // use hse or hsi
            .pllmul().bits(self.mul as u8)  // set main pll mul factor
            .pllxtpre().clear_bit()  // HSE not divide by 2
        });

        if self.use_hse {
            rcc.cfgr2.modify(|_, w| unsafe{w
                .prediv1src().clear_bit()  // HSE as PREDIV1 input
                .prediv1().bits(self.div as u8)   // set main pll div factor
            })
        } else {
            let extend = unsafe{&*EXTEND::ptr()};
            // Set HSI to not divide by 2
            extend.extend_ctr.modify(|_, w|w.pll_hsi_pre().set_bit());
        }
    }
}

