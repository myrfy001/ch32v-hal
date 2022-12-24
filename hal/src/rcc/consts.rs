
pub const NO_OP_DIV_MUL_FACTOR:[DivMulFactor;1] = [
    DivMulFactor{mul:1, div: 1, cfg_bit: 0b0},
];

pub const MAIN_PLL_DIV_TABLE_HSE:[DivMulFactor;16] = [
    DivMulFactor{mul:1, div: 1, cfg_bit: 0b0000},
    DivMulFactor{mul:1, div: 2, cfg_bit: 0b0001},
    DivMulFactor{mul:1, div: 3, cfg_bit: 0b0010},
    DivMulFactor{mul:1, div: 4, cfg_bit: 0b0011},
    DivMulFactor{mul:1, div: 5, cfg_bit: 0b0100},
    DivMulFactor{mul:1, div: 6, cfg_bit: 0b0101},
    DivMulFactor{mul:1, div: 7, cfg_bit: 0b0110},
    DivMulFactor{mul:1, div: 8, cfg_bit: 0b0111},
    DivMulFactor{mul:1, div: 9, cfg_bit: 0b1000},
    DivMulFactor{mul:1, div: 10, cfg_bit: 0b1001},
    DivMulFactor{mul:1, div: 11, cfg_bit: 0b1010},
    DivMulFactor{mul:1, div: 12, cfg_bit: 0b1011},
    DivMulFactor{mul:1, div: 13, cfg_bit: 0b1100},
    DivMulFactor{mul:1, div: 14, cfg_bit: 0b1101},
    DivMulFactor{mul:1, div: 15, cfg_bit: 0b1110},
    DivMulFactor{mul:1, div: 16, cfg_bit: 0b1111},
];

pub const MAIN_PLL_DIV_TABLE_HSI:[DivMulFactor;1] = [
    DivMulFactor{mul:1, div: 1, cfg_bit: 0b0},
];

#[cfg(any(feature="ch32v305", feature="ch32v307"))]
pub const MAIN_PLL_MUL_TABLE:[DivMulFactor;16] = [
    DivMulFactor{mul: 3, div: 1, cfg_bit: 0b0001},
    DivMulFactor{mul: 4, div: 1, cfg_bit: 0b0010},
    DivMulFactor{mul: 5, div: 1, cfg_bit: 0b0011},
    DivMulFactor{mul: 6, div: 1, cfg_bit: 0b0100},
    DivMulFactor{mul: 13, div: 2, cfg_bit: 0b1101},
    DivMulFactor{mul: 7, div: 1, cfg_bit: 0b0101},
    DivMulFactor{mul: 8, div: 1, cfg_bit: 0b0110},
    DivMulFactor{mul: 9, div: 1, cfg_bit: 0b0111},
    DivMulFactor{mul: 10, div: 1, cfg_bit: 0b1000},
    DivMulFactor{mul: 11, div: 1, cfg_bit: 0b1001},
    DivMulFactor{mul: 12, div: 1, cfg_bit: 0b1010},
    DivMulFactor{mul: 13, div: 1, cfg_bit: 0b1011},
    DivMulFactor{mul: 14, div: 1, cfg_bit: 0b1100},
    DivMulFactor{mul: 15, div: 1, cfg_bit: 0b1110},
    DivMulFactor{mul: 16, div: 1, cfg_bit: 0b1111},
    DivMulFactor{mul: 18, div: 1, cfg_bit: 0b0000},
];


pub const fn calc_pll_param(
    in_freq:u32, out_freq: u32, 
    div_list:&[DivMulFactor], 
    mul_list:&[DivMulFactor]) -> Option<(DivMulFactor, DivMulFactor)>
{
    let mut div_idx = 0;
    
    loop {
        let mut mul_idx = 0;
        loop {

            if in_freq * (mul_list[mul_idx].mul as u32) / (mul_list[mul_idx].div as u32) * (div_list[div_idx].mul as u32) / (div_list[div_idx].div as u32) == out_freq {
                return Some((mul_list[mul_idx], div_list[div_idx]))
            }

            mul_idx += 1;
            if mul_idx == mul_list.len() {
                break;
            }
        }

        div_idx += 1;
        if div_idx == div_list.len() {
            break;
        }
    }
    
    None

} 



#[derive(Copy, Clone)]
pub struct DivMulFactor {
    pub mul: u16,
    pub div: u16,
    pub cfg_bit: u16,
}


#[cfg(any(feature="ch32v307"))]
pub const AHB_PREDIV_TABLE:[DivMulFactor;8] = [
    DivMulFactor{mul:1, div:2, cfg_bit:0b1000},
    DivMulFactor{mul:1, div:4, cfg_bit:0b1001},
    DivMulFactor{mul:1, div:8, cfg_bit:0b1010},
    DivMulFactor{mul:1, div:16, cfg_bit:0b1011},
    DivMulFactor{mul:1, div:64, cfg_bit:0b1100},
    DivMulFactor{mul:1, div:128, cfg_bit:0b1101},
    DivMulFactor{mul:1, div:256, cfg_bit:0b1110},
    DivMulFactor{mul:1, div:512, cfg_bit:0b1111},
];

#[cfg(any(feature="ch32v307"))]
pub const APB_PREDIV_TABLE:[DivMulFactor;4] = [
    DivMulFactor{mul:1, div:2, cfg_bit:0b100},
    DivMulFactor{mul:1, div:4, cfg_bit:0b101},
    DivMulFactor{mul:1, div:8, cfg_bit:0b110},
    DivMulFactor{mul:1, div:16, cfg_bit:0b111},
];
