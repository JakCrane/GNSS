use libm::{atan, sqrt};
use core::f64::consts::{PI, SQRT_2};
use super::config::{Correlators, CHIP_RATE, F_IF, F_L1};

const T: f64 = 1e-3;             // one dump

const DLL_BN: f64 = 1.0;         // Hz
const DLL_N: u32 = 10;           // dumps summed per DLL update (10 ms)

const FLL_BW: f64 = 10.0;        // Hz, pull-in
const FLL_MS: u32 = 1000;        // FLL only for the first second
const PLL_BW: f64 = 40.0;        // Hz
const FLL_ASSIST_BW: f64 = 2.0;  // Hz, keeps helping once the PLL is on

pub struct Tracker {
    pub doppler: f64,            // tracked carrier Doppler, Hz (loop integrator)
    pub carr_out: f64,           // Doppler actually written to the NCO, Hz
    pub code_corr: f64,          // DLL correction to code rate, chips/s
    pub disc: f64,               // last (E-L)/(E+L), for printing
    pub lock: f64,               // smoothed cos(2*phase error): ~1 when phase-locked
    pub carrier_loop: bool,      // false = DLL only, carrier NCO fixed
    pub csign: f64,              // +1 from the mixer maths; flip if the FLL runs away
    e_sum: f64,
    l_sum: f64,
    n: u32,
    prev: Option<(f64, f64)>,    // last prompt (I, Q), for the FLL
    last_dump: Option<u8>,
    ms: u32,
}

impl Tracker {
    pub fn new(doppler: f64, carrier_loop: bool) -> Self {
        Self {
            doppler, carr_out: doppler, code_corr: 0.0, disc: 0.0, lock: 0.0,
            carrier_loop, csign: 1.0,
            e_sum: 0.0, l_sum: 0.0, n: 0, prev: None, last_dump: None, ms: 0,
        }
    }

    pub fn update(&mut self, c: &Correlators) {
        self.dll_update(c);
        if self.carrier_loop {
            self.carrier_update(c.ip as f64, c.qp as f64, c.dumps);
        }
    }

    fn dll_update(&mut self, c: &Correlators) {
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

    /// Mixer gives prompt ~ exp(j(theta_nco - theta_sig)): if the prompt rotates
    /// forwards the NCO is too fast, so corrections are subtracted (times csign).
    fn carrier_update(&mut self, i: f64, q: f64, dumps: u8) {
        // A missed dump breaks the "consecutive prompts" pair the FLL needs.
        if self.last_dump.map_or(true, |d| dumps != d.wrapping_add(1)) {
            self.prev = None;
        }
        self.last_dump = Some(dumps);

        // FLL: rotation between consecutive prompts, Hz. atan (not atan2) so a
        // data-bit flip between them doesn't matter. Pull-in range +-250 Hz.
        let mut f_err = 0.0;
        if let Some((pi, pq)) = self.prev {
            let cross = pi * q - i * pq;
            let dot = pi * i + pq * q;
            if dot != 0.0 {
                f_err = atan(cross / dot) / (2.0 * PI * T);
            }
        }
        self.prev = Some((i, q));

        let s = self.csign;
        if self.ms < FLL_MS {
            self.doppler -= s * 4.0 * FLL_BW * T * f_err;
            self.carr_out = self.doppler;
        } else {
            // Costas PLL: phase error in cycles, insensitive to the data bit sign
            let e = if i != 0.0 { atan(q / i) / (2.0 * PI) } else { 0.0 };
            let w0 = PLL_BW / 0.53;
            self.doppler -= s * (w0 * w0 * T * e + 4.0 * FLL_ASSIST_BW * T * f_err);
            self.carr_out = self.doppler - s * SQRT_2 * w0 * e;
        }

        let p = i * i + q * q;
        if p > 0.0 {
            self.lock += 0.01 * ((i * i - q * q) / p - self.lock);   // ~100 ms smoothing
        }
        self.ms += 1;
    }

    /// Code rate is carrier-aided: follows the tracked Doppler plus the DLL's nudge.
    pub fn code_hz(&self) -> f64 { CHIP_RATE * (1.0 + self.doppler / F_L1) + self.code_corr }
    pub fn carr_hz(&self) -> f64 { F_IF + self.carr_out }
}