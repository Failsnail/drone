use std::ops::Mul;

use vector::Vector;
use matrix::Matrix;

#[derive(Clone, Copy)]
pub struct Quaternion {
    r: f32,
    i: f32,
    j: f32,
    k: f32,
}

impl Quaternion {
    pub fn identity () -> Quaternion {
        Quaternion {
            r: 1.0f32,
            i: 0.0f32,
            j: 0.0f32,
            k: 0.0f32,
        }       
    }

    pub fn normalize (&mut self) {
        let len = (self.r * self.r + self.i * self.i + self.j * self.j + self.k * self.k).sqrt();

        self.r /= len;
        self.i /= len;
        self.j /= len;
        self.k /= len;
    }
    
    pub fn from_vector (mut v :Vector) -> Quaternion {
        assert!(v.w == 0.0f32);

        let theta :f32 = Vector::magnitude(v);
        
        if theta <= 0.0f32 {
            return Quaternion::identity();
        }
        
        v /= theta;

        let half_theta = theta / 2.0f32;

        Quaternion {
            r: half_theta.cos(),
            i: half_theta.sin() * v.x,
            j: half_theta.sin() * v.y,
            k: half_theta.sin() * v.z,
        }
    }

    pub fn to_matrix (self) -> Matrix {
        assert!(0.9999f32 < self.r * self.r + self.i * self.i + self.j * self.j + self.k * self.k);
        assert!(1.0001f32 > self.r * self.r + self.i * self.i + self.j * self.j + self.k * self.k);
        
        Matrix([
            Vector::new(self.r * self.r + self.i * self.i - self.j * self.j - self.k * self.k, 2.0f32 * (self.i * self.j + self.r * self.k), 2.0f32 * (self.i * self.k - self.r * self.j), 0.0f32),
            Vector::new(2.0f32 * (self.i * self.j - self.r * self.k), self.r * self.r - self.i * self.i + self.j * self.j - self.k * self.k, 2.0f32 * (self.j * self.k + self.r * self.i), 0.0f32),
            Vector::new(2.0f32 * (self.i * self.k + self.r * self.j), 2.0f32 * (self.j * self.k - self.r * self.i), self.r * self.r - self.i * self.i - self.j * self.j + self.k * self.k, 0.0f32),
            Vector::new(0.0, 0.0, 0.0, 1.0),
        ])
    }
}

impl Mul<Quaternion> for Quaternion {
    type Output = Quaternion;
    
    fn mul (self, rhs :Quaternion) -> Quaternion {
        Quaternion {
            r: self.r * rhs.r - self.i * rhs.i - self.j * rhs.j - self.k * rhs.k,
            i: self.r * rhs.i + self.i * rhs.r + self.j * rhs.k - self.k * rhs.j,
            j: self.r * rhs.j + self.j * rhs.r + self.k * rhs.i - self.i * rhs.k,
            k: self.r * rhs.k + self.k * rhs.r + self.i * rhs.j - self.j * rhs.i,
        }
    }
}
