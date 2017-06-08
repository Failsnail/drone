use vector::Vector;

pub struct Bezier {
    p1: Vector,
    p2: Vector,
    p3: Vector,
    p4: Vector,
}

impl Bezier {
    #[allow(dead_code)]
    pub fn new (p1: Vector, p2: Vector, p3: Vector, p4: Vector) -> Bezier {
        assert!(p1.w < p2.w);
        assert!(p2.w < p3.w);
        assert!(p3.w < p4.w);

        Bezier {
            p1: p1,
            p2: p2,
            p3: p3,
            p4: p4,
        }
    }

    #[allow(dead_code)]
    pub fn sample_p (&self, tau: f32) -> Vector {
        assert!(0f32 <= tau);
        assert!(tau <= 1f32);

        self.p1 * ((1f32 - tau) * (1f32 - tau) * (1f32 - tau)) +
        self.p2 * (3f32 * (1f32 - tau) * (1f32 - tau) * tau) +
        self.p3 * (3f32 * (1f32 - tau) * tau * tau) +
        self.p4 * (tau * tau * tau)
    }

    #[allow(dead_code)]
    pub fn sample_dp_dtau (&self, tau: f32) -> Vector { //derivative with respect to tau
        assert!(0f32 <= tau);
        assert!(tau <= 1f32);

        self.p1 * (-3f32 * (1f32 - tau) * (1f32 - tau)) +
        self.p2 * (9f32 * tau * tau - 12f32 * tau + 3f32) +
        self.p3 * (-9f32 * tau * tau + 6f32 * tau) +
        self.p4 * (3f32 * tau * tau)
    }

    #[allow(dead_code)]
    pub fn sample_ddp_dtau2 (&self, tau: f32) -> Vector { //second derivative with respect to tau
        assert!(0f32 <= tau);
        assert!(tau <= 1f32);
        
        self.p1 * (-6f32 * tau + 6f32) +
        self.p2 * (18f32 * tau - 12f32) +
        self.p3 * (-18f32 * tau + 6f32) +
        self.p4 * (6f32 * tau)
    }
    
    #[allow(dead_code)]
    pub fn find_tau (&self, t: f32) -> f32 {
        assert!(self.p1.w <= t);
        assert!(t <= self.p4.w);

        //newtons method is is faster but doesn't seem to converge in every case
        //better initial conditions might fix it, checking if it doesn't converge and fall back to bisection will have a better average case performance than bisection too (probably)
        
        /*
        //newtons method;
        let delta_t = 0.001f32;

        let mut tau = 0.5f32; //initial guess

        let mut n = 0;
        
        loop {
            n += 1;
            let t_current = self.sample_p(tau).w;

            if (t - t_current).abs() < delta_t {
                break;
            }

            let dt_dtau = self.sample_dp_dtau(tau).w;
            tau += (t - t_current) / dt_dtau;

            //enforces 0f32 <= tau <= 1f32
            tau = tau.min(1f32);
            tau = tau.max(0f32);

            if n > 100 {
                panic!("newtons method didn't converge");
            }
        }

        tau
         */
        
        //bisection;
        let delta_tau = 0.001f32;
        
        let mut tau_low  = 0f32;
        let mut tau_high = 1f32;

        //uses error in tau space, might need to be switched to error in t space
        while tau_high - tau_low > delta_tau {
            let tau_mid = (tau_low + tau_high) * 0.5f32;
            if t < self.sample_p(tau_mid).w {
                // tau_low < tau_solution < tau_mid
                tau_high = tau_mid;
            } else {
                // tau_mid < tau_solution < tau_high
                tau_low = tau_mid;
            }
        }

        (tau_low + tau_high) * 0.5f32
    }

    //fix this code to work with velocities with magnitude 0 (the t2 and t3 calculations are broken)
    #[allow(dead_code)]
    pub fn merge (x1: Vector, v1: Vector, t1: f32, x4: Vector, v4: Vector, t4: f32) -> Bezier {
        let t_span = t4 - t1;
        //let d = Vector::magnitude(x4 - x1);
        
        //OPTIMIZE THIS (account for velocity?)
        let t2 = t1 + t_span / 3f32;
        let t3 = t1 + t_span * 2f32 / 3f32;

        let p1 = Vector {
            x: x1.x,
            y: x1.y,
            z: x1.z,
            w: t1,
        };

        let x2 = x1 + v1 * (t2 - t1);
        let p2 = Vector {
            x: x2.x,
            y: x2.y,
            z: x2.z,
            w: t2,
        };
        
        let x3 = x4 - v4 * (t4 - t3);
        let p3 = Vector {
            x: x3.x,
            y: x3.y,
            z: x3.z,
            w: t3,
        };

        let p4 = Vector {
            x: x4.x,
            y: x4.y,
            z: x4.z,
            w: t4,
        };
        
        let result = Bezier::new(
            p1,
            p2,
            p3,
            p4,
        );

        //assert that all boundary conditions are met;
        assert!(result.p1.x == x1.x);
        assert!(result.p1.y == x1.y);
        assert!(result.p1.z == x1.z);
        assert!(result.p1.w == t1);

        assert!(result.p4.x == x4.x);
        assert!(result.p4.y == x4.y);
        assert!(result.p4.z == x4.z);
        assert!(result.p4.w == t4);

        if Vector::magnitude(v1) != 0f32 {
            let mut vel1 = result.sample_dp_dtau(0f32);
            vel1 /= vel1.w;

            assert!((vel1.x - v1.x).abs() < 0.01f32);
            assert!((vel1.y - v1.y).abs() < 0.01f32);
            assert!((vel1.z - v1.z).abs() < 0.01f32);
            assert!(vel1.w == 1f32);
        }

        if Vector::magnitude(v4) != 0f32 {
            let mut vel4 = result.sample_dp_dtau(1f32);
            vel4 /= vel4.w;

            assert!((vel4.x - v4.x).abs() < 0.01f32);
            assert!((vel4.y - v4.y).abs() < 0.01f32);
            assert!((vel4.z - v4.z).abs() < 0.01f32);
            assert!(vel4.w == 1f32);
        }

        result
    }
    
    #[allow(dead_code)]
    pub fn print (&self) {
        print!("p1: "); self.p1.print();
        print!("p2: "); self.p2.print();
        print!("p3: "); self.p3.print();
        print!("p4: "); self.p4.print();
    }
}

pub struct Spline {
    curve_list: Vec<Bezier>,
}

impl Spline {
    #[allow(dead_code)]
    pub fn new () -> Spline {
        Spline {
            curve_list: Vec::<Bezier>::new(),
        }
    }

    #[allow(dead_code)]
    pub fn push_curve (&mut self, new_curve :Bezier) {
        if self.curve_list.len() > 0 {
            let n = self.curve_list.len() - 1;
            let last_curve = &self.curve_list[n];

            let last_state = last_curve.p4; //last state of existing spline
            let start_state = new_curve.p1; //starting state of new bezier

            assert!(last_state.x == start_state.x);
            assert!(last_state.y == start_state.y);
            assert!(last_state.z == start_state.z);
            assert!(last_state.w == start_state.w);
        }
        
        self.curve_list.push(new_curve);
    }

    #[allow(dead_code)]
    pub fn duration (&self) -> f32 {
        self.curve_list.last().unwrap().p4.w
    }

    #[allow(dead_code)]
    fn find_n_tau (&self, t :f32) -> (usize, f32) {
        let mut n = Option::None::<usize>;

        for m in 0 .. self.curve_list.len() {
            if self.curve_list[m].p1.w <= t && t <= self.curve_list[m].p4.w {
                assert!(n.is_none());
                n = Some(m);
            }
        }

        assert!(n.is_some());
        
        let tau = self.curve_list[n.unwrap()].find_tau(t);

        (n.unwrap(), tau)
    }
    
    #[allow(dead_code)]
    pub fn sample (&self, t :f32) -> Vector {
        let (n, tau) = self.find_n_tau(t);

        let p = self.curve_list[n as usize].sample_p(tau);
        
        p.to_position()
    }

    #[allow(dead_code)]
    pub fn sample_velocity (&self, t:f32) -> Vector {
        let (n, tau) = self.find_n_tau(t);

        let dp_dtau   = self.curve_list[n].sample_dp_dtau(tau); //first derivative of p with respect to tau
        let dt_dtau = dp_dtau.w; //first derivative of t with respect to tau
        let dtau_dt = 1.0f32 / dt_dtau; //first derivative of tau with respect to t

        let dp_dt = dp_dtau * dtau_dt;
        
        dp_dt.to_translation()
    }

    #[allow(dead_code)]
    pub fn sample_acceleration (&self, t:f32) -> Vector {
        let (n, tau) = self.find_n_tau(t);

        let dp_dtau   = self.curve_list[n].sample_dp_dtau(tau); //first derivative of p with respect to tau
        let ddp_dtau2 = self.curve_list[n].sample_ddp_dtau2(tau); //second derivative of p with respect to tau
        
        let dt_dtau = dp_dtau.w; //first derivative of t with respect to tau
        let dtau_dt = 1.0f32 / dt_dtau; //first derivative of tau with respect to t

        let ddt_dtau2 = ddp_dtau2.w; //second derivative of t with respect to tau
        let ddtau_dt2 = - ddt_dtau2 / (dt_dtau * dt_dtau * dt_dtau); //second derivative of tau with respect to t

        let ddp_dt2 = ddp_dtau2 * dtau_dt + dp_dtau * ddtau_dt2;
        
        ddp_dt2.to_translation()
    }

    #[allow(dead_code)]
    pub fn sample_all (&self, t: f32) -> (Vector, Vector, Vector) {
        let (n, tau) = self.find_n_tau(t);

        let p         = self.curve_list[n].sample_p(tau);
        let dp_dtau   = self.curve_list[n].sample_dp_dtau(tau); //first derivative of p with respect to tau
        let ddp_dtau2 = self.curve_list[n].sample_ddp_dtau2(tau); //second derivative of p with respect to tau
        
        let dt_dtau = dp_dtau.w; //first derivative of t with respect to tau
        let dtau_dt = 1.0f32 / dt_dtau; //first derivative of tau with respect to t

        let dp_dt = dp_dtau * dtau_dt;
        
        let ddt_dtau2 = ddp_dtau2.w; //second derivative of t with respect to tau
        let ddtau_dt2 = - ddt_dtau2 / (dt_dtau * dt_dtau * dt_dtau); //second derivative of tau with respect to t

        let ddp_dt2 = ddp_dtau2 * (dtau_dt * dtau_dt) + dp_dtau * ddtau_dt2;
        
        (p.to_position(), dp_dt.to_translation(), ddp_dt2.to_translation())
    }
}

#[allow(dead_code)]
pub fn lissajous () -> Spline {
    let mut spline = Spline::new();

    spline.push_curve(
        Bezier::new(
            Vector::new(0f32,  0f32, -3f32, 0f32),
            Vector::new(5f32,  5f32, -3f32, 3f32),
            Vector::new(5f32, -5f32, -5f32, 6f32),
            Vector::new(0f32,  0f32, -5f32, 9f32),
        )
    );
    spline.push_curve(
        Bezier::new(
            Vector::new( 0f32,  0f32, -5f32, 9f32),
            Vector::new(-5f32,  5f32, -5f32, 12f32),
            Vector::new(-5f32, -5f32, -3f32, 15f32),
            Vector::new( 0f32, -0f32, -3f32, 18f32),
        )
    );

    spline
}

#[allow(dead_code)]
pub fn test_spline () -> Spline {
    let mut spline = Spline::new();

    spline.push_curve(
        Bezier::new(
            Vector::new(-2f32, -2f32, -4f32, 0f32),
            Vector::new( 0f32, -2f32, -4f32, 2f32),
            Vector::new( 0f32, -2f32, -8f32, 3f32),
            Vector::new( 3f32, -2f32, -8f32, 5f32),
        )
    );
    spline.push_curve(
        Bezier::new(
            Vector::new(3f32, -2f32, -8f32, 5f32),
            Vector::new(6f32, -2f32, -8f32, 7f32),
            Vector::new(6f32, -1f32, -4f32, 8f32),
            Vector::new(3f32, -1f32, -4f32, 10f32),
        )
    );
    spline.push_curve(
        Bezier::new(
            Vector::new(3f32, -1f32, -4f32, 10f32),
            Vector::new(0f32, -1f32, -4f32, 12f32),
            Vector::new(0f32,  0f32, -8f32, 13f32),
            Vector::new(3f32,  0f32, -8f32, 15f32),
        )
    );
    spline.push_curve(
        Bezier::new(
            Vector::new(3f32, 0f32, -8f32, 15f32),
            Vector::new(6f32, 0f32, -8f32, 17f32),
            Vector::new(6f32, 1f32, -4f32, 18f32),
            Vector::new(3f32, 1f32, -4f32, 20f32),
        )
    );

    spline.push_curve(
        Bezier::new(
            Vector::new( 3f32,  1f32, -4f32, 20f32),
            Vector::new( 0f32,  1f32, -4f32, 22f32),
            Vector::new(-3f32,  1f32, -4f32, 24f32),
            Vector::new(-3f32,  2f32, -4f32, 26f32),
        )
    );
    spline.push_curve(
        Bezier::new(
            Vector::new(-3f32,  2f32, -4f32, 26f32),
            Vector::new(-3f32,  3f32, -4f32, 28f32),
            Vector::new(-3f32, -2f32, -4f32, 29f32),
            Vector::new(-3f32, -2f32, -4f32, 31f32),
        )
    );
    spline.push_curve(
        Bezier::new(
            Vector::new(-3f32, -2f32, -4f32, 31f32),
            Vector::new(-3f32, -2f32, -4f32, 32f32),
            Vector::new(-3f32, -2f32, -4f32, 33f32),
            Vector::new(-3f32, -2f32, -4f32, 34f32),
        )
    );
    spline.push_curve(
        Bezier::new(
            Vector::new(-3f32, -2f32, -4f32, 34f32),
            Vector::new(-3f32, -2f32, -4f32, 36f32),
            Vector::new(-3f32, -2f32, -4f32, 37f32),
            Vector::new(-2f32, -2f32, -4f32, 38f32),
        )
    );

    spline
}
