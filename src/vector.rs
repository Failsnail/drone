use std::ops::*;
use std;
use rand;
use rand::Rng;

#[derive(Clone, Copy)]
pub struct Vector {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32, //not considdered to be a part of the actual vector, only a flag to differentiate between positions and translations
}

#[allow(dead_code)]
impl Vector {
    pub fn new (x :f32, y :f32, z :f32, w :f32) -> Vector {
        Vector {
            x: x,
            y: y,
            z: z,
            w: w,
        }
    }

    pub fn ex () -> Vector {
        Vector {
            x: 1f32,
            y: 0f32,
            z: 0f32,
            w: 0f32,
        }
    }

    pub fn ey () -> Vector {
        Vector {
            x: 0f32,
            y: 1f32,
            z: 0f32,
            w: 0f32,
        }
    }

    pub fn ez () -> Vector {
        Vector {
            x: 0f32,
            y: 0f32,
            z: 1f32,
            w: 0f32,
        }
    }

    pub fn null () -> Vector {
        Vector {
            x: 0.0f32,
            y: 0.0f32,
            z: 0.0f32,
            w: 0.0f32,
        }
    }

    pub fn origin () -> Vector {
        Vector {
            x: 0.0f32,
            y: 0.0f32,
            z: 0.0f32,
            w: 1.0f32,
        }
    }

        
    pub fn print (self) {
        println!("({}, {}, {}, {})", self.x, self.y, self.z, self.w);
    }

    pub fn is_finite (&self) -> bool {
        self.x.is_finite() &&
        self.y.is_finite() &&
        self.z.is_finite() &&
        self.w.is_finite()
    }

    pub fn to_translation (self) -> Vector {
        Vector {
            x: self.x,
            y: self.y,
            z: self.z,
            w: 0.0f32,
        }
    }

    pub fn to_position (self) -> Vector {
        Vector {
            x: self.x,
            y: self.y,
            z: self.z,
            w: 1.0f32,
        }
    }
    
    pub fn dot (lhs :Vector, rhs :Vector) -> f32 {
        assert!(lhs.w == 0.0f32, "panic at Vector::dot{}", {lhs.print(); ' '});
        assert!(rhs.w == 0.0f32, "panic at Vector::dot{}", {rhs.print(); ' '});
        
        lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z
    }

    pub fn cross (lhs :Vector, rhs :Vector) -> Vector {
        assert!(lhs.w == 0.0f32, "panic at Vector::cross{}", {lhs.print(); ' '});
        assert!(rhs.w == 0.0f32, "panic at Vector::cross{}", {rhs.print(); ' '});
        
        Vector {
            x: lhs.y * rhs.z - lhs.z * rhs.y,
            y: lhs.z * rhs.x - lhs.x * rhs.z,
            z: lhs.x * rhs.y - lhs.y * rhs.x,
            w: 0.0f32,
        }
    }

    pub fn magnitude (v :Vector) -> f32 {
        assert!(v.w == 0.0f32);
        
        Vector::dot(v, v).sqrt()
    }

    pub fn normalize (v :Vector) -> Option<Vector> {
        assert!(v.w == 0.0f32);

        let len = Vector::magnitude(v);

        if len == 0.0f32 {
            return None;
        } else {
            return Some(v / len);
        }
    }

    pub fn cos_angle (v1 :Vector, v2 :Vector) -> f32 { //remove expect
        Vector::dot(
            Vector::normalize(v1).expect("cos_angle with v1 = Vector::Null()"),
            Vector::normalize(v2).expect("cos_angle with v2 = Vector::Null()")
        ).min(1f32).max(-1f32)
    }

    pub fn angle (v1 :Vector, v2 :Vector) -> f32 {
        let cos_angle = Vector::cos_angle(v1, v2);
        let angle = cos_angle.acos();

        angle
    }
    
    pub fn random_unitvector () -> Vector {
        let mut rng = rand::thread_rng();
        
        let phi :f32 = rng.gen_range::<f32>(0f32, 2f32 * std::f32::consts::PI);
        let u   :f32 = rng.gen_range::<f32>(-1f32, 1f32);
        let r   :f32 = f32::sqrt(1f32 - u*u);
        
        Vector {
            x: r * f32::sin(phi),
            y: u,
            z: r * f32::cos(phi),
            w: 0.0f32,
        }
    }
    
    //returns a unit vector which has an angle theta relative to v, where theta_low < theta < theta_high
    //assumes small values for theta_high
    pub fn offset (v :Vector, theta_low :f32, theta_high :f32) -> Vector {
        assert!(0.0f32 <= theta_low);
        assert!(theta_low < theta_high);
        
        let mut rng = rand::thread_rng();
        
        let mut v2 = Vector::cross(Vector::random_unitvector(), v);
        v2 = Vector::normalize(v2).unwrap();

        let result = Vector::normalize(v).unwrap() + v2 * rng.gen_range::<f32>(theta_low, theta_high);
        
        Vector::normalize(result).unwrap()
    }
}

impl Neg for Vector {
    type Output = Vector;

    fn neg (self) -> Vector {
        Vector {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: -self.w,
        }
    }
}

impl AddAssign<Vector> for Vector {
    fn add_assign (&mut self, rhs :Vector) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
        self.w += rhs.w;
    }
}

impl Add<Vector> for Vector {
    type Output = Vector;
    
    fn add (self, rhs :Vector) -> Vector {
        let mut result = self;
        result += rhs;
        result
    }
}

impl SubAssign<Vector> for Vector {
    fn sub_assign (&mut self, rhs :Vector) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
        self.w -= rhs.w;
    }
}

impl Sub<Vector> for Vector {
    type Output = Vector;
    
    fn sub (self, rhs :Vector) -> Vector {
        let mut result = self;
        result -= rhs;
        result
    }
}

impl MulAssign<f32> for Vector {
    fn mul_assign (&mut self, rhs :f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
        self.w *= rhs;
    }
}

impl Mul<f32> for Vector {
    type Output = Vector;
    
    fn mul (self, rhs :f32) -> Vector{
        let mut result = self;
        result *= rhs;
        result
    }
}

impl DivAssign<f32> for Vector {
    fn div_assign (&mut self, rhs :f32) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
        self.w /= rhs;
    }
}

impl Div<f32> for Vector {
    type Output = Vector;

    fn div (self, rhs :f32) -> Vector{
        let mut result = self;
        result /= rhs;
        result
    }
}
