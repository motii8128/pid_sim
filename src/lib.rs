pub struct SecondOrder {
    pub wn: f64,
    pub zeta: f64,
    pub y: f64,
    pub prev_y: f64,
}

impl SecondOrder {
    pub fn new(wn: f64, zeta: f64) -> Self {
        Self { wn, zeta, y: 0.0, prev_y: 0.0 }
    }

    pub fn step(&mut self, u: f64, dt: f64) {

        let wn2 = self.wn * self.wn;
        let t2 = dt * dt;

        let z_wn_t = self.zeta * self.wn * dt;

        let v1 = (2.0-wn2*t2)/(1.0+z_wn_t) * self.y;

        let v2 = (-1.0 + z_wn_t)/(1.0 + z_wn_t) * self.prev_y;

        let v3 = (wn2 * t2)/(1.0 + z_wn_t) * u;

        self.prev_y = self.y;
        self.y = v1 + v2 + v3;
    }
}

/// 離散PID(微分先行 + 条件付き積分によるアンチワインドアップ)
pub struct Pid {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    prev_y: Option<f64>,
}

impl Pid {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self { kp, ki, kd, integral: 0.0, prev_y: None}
    }

    pub fn update(&mut self, r: f64, y: f64, dt: f64) -> f64 {
        let e = r - y;

        let d = match self.prev_y {
            Some(p) => (e - p) / dt,
            None => 0.0,
        };
        self.prev_y = Some(e);

        self.integral += e * dt;

        let u_unsat = self.kp * e + self.ki * self.integral + self.kd * d;
        // let u = u_unsat.clamp(self.u_min, self.u_max);
        let u = u_unsat;

        u
    }
}