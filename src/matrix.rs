use std::ops::*;

use vector::Vector;

#[derive(Clone, Copy)]
pub struct Matrix (pub [Vector; 4]);

#[allow(dead_code)]
impl Matrix {
    pub fn identity () -> Matrix {
        Matrix([
            Vector::new(1.0, 0.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 0.0, 1.0)
        ])
    }

    pub fn scaling (scale :f32) -> Matrix {
        Matrix([
            Vector::new(scale, 0.0,   0.0,   0.0),
            Vector::new(0.0,   scale, 0.0,   0.0),
            Vector::new(0.0,   0.0,   scale, 0.0),
            Vector::new(0.0,   0.0,   0.0,   1.0),
        ])
    }
    
    pub fn translation (mut v :Vector) -> Matrix {
        assert!(v.w == 0.0f32 || v.w == 1.0f32);

        v.w = 1.0f32;
        
        Matrix([
            Vector::new(1.0, 0.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 1.0, 0.0),
            v
        ])
    }

    pub fn pos_to_vector () -> Matrix {
        Matrix([
            Vector::new(1.0, 0.0, 0.0, 0.0),
            Vector::new(0.0, 1.0, 0.0, 0.0),
            Vector::new(0.0, 0.0, 1.0, 0.0),
            Vector::new(0.0, 0.0, 0.0, 0.0)
        ])
    }
    
    pub fn print (self) {
        println!("[");
        self.0[0].print();
        self.0[1].print();
        self.0[2].print();
        self.0[3].print();
        println!("]");
    }

    pub fn data (self) -> [[f32; 4]; 4] {
        [
            [self.0[0].x, self.0[0].y, self.0[0].z, self.0[0].w],
            [self.0[1].x, self.0[1].y, self.0[1].z, self.0[1].w],
            [self.0[2].x, self.0[2].y, self.0[2].z, self.0[2].w],
            [self.0[3].x, self.0[3].y, self.0[3].z, self.0[3].w]
        ]
    }
}

impl Mul<Vector> for Matrix {
    type Output = Vector;

    fn mul (self, rhs :Vector) -> Vector {
        let mut result = Vector::null();

        result += self.0[0] * rhs.x;
        result += self.0[1] * rhs.y;
        result += self.0[2] * rhs.z;
        result += self.0[3] * rhs.w;

        result
    }
}

impl Mul<Matrix> for Matrix {
    type Output = Matrix;

    fn mul (self, rhs :Matrix) -> Matrix {
        Matrix([
            self * rhs.0[0],
            self * rhs.0[1],
            self * rhs.0[2],
            self * rhs.0[3]
        ])
    }
}
