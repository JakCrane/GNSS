use libm::sqrt;
use super::config::{Correlators, CHIP_RATE, F_IF, F_L1};

const DLL_BN: f64 = 1.0;    // Hz loop bandwidth
const DLL_N: u32 = 10;      // dumps summed per DLL update (10 ms)

pub struct Tracker {
    pub doppler: f64,       // carrier Doppler, Hz (fixed until the FLL/PLL exist)
    pub code_corr: f64,     // DLL correction to code rate, chips/s
    pub disc: f64,          // last (E-L)/(E+L), for printing
    e_sum: f64,
    l_sum: f64,
    n: u32,
}

impl Tracker {
    pub fn new(doppler: f64) -> Self {
        Self { doppler, code_corr: 0.0, disc: 0.0, e_sum: 0.0, l_sum: 0.0, n: 0 }
    }

    pub fn update(&mut self, c: &Correlators) {
        let mag = |i: i32, q: i32| { let (i, q) = (i as f64, q as f64); sqrt(i * i + q * q) };
        self.e_sum += mag(c.ie, c.qe);
        self.l_sum += mag(c.il, c.ql);
        self.n += 1;

        if self.n == DLL_N {
            // + : signal arrives earlier than P -> speed the code up
            self.disc = (self.e_sum - self.l_sum) / (self.e_sum + self.l_sum);
            let tau = 0.5 * self.disc;                  // chips
            self.code_corr = 4.0 * DLL_BN * tau;        // chips/s
            self.e_sum = 0.0;
            self.l_sum = 0.0;
            self.n = 0;
        }
    }

    pub fn code_hz(&self) -> f64 { CHIP_RATE * (1.0 + self.doppler / F_L1) + self.code_corr }
    pub fn carr_hz(&self) -> f64 { F_IF + self.doppler }
}