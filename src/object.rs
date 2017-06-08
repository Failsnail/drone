use vector::Vector;
use quaternion::Quaternion;

pub struct Object {
    pub scale:            f32,
    pub mass:             f32, //if mass == Infinity -> object is fixed (but might still rotate!)
    pub angular_inertia:  f32,
    pub position:         Vector,
    pub velocity:         Vector,
    pub rotation:         Quaternion,
    pub angular_velocity: Vector,
    
    pub graphicsmodel_id: usize,
}

#[allow(dead_code)]
impl Object {
    pub fn update (&mut self, force: Vector, torque: Vector, dt: f32) {
        assert!(force.w == 0f32);
        assert!(torque.w == 0f32);

        //spatial
        let acceleration = force / self.mass;
        self.velocity += acceleration * dt;
        self.position += self.velocity * dt;

        //rotational
        self.angular_velocity += torque * (dt / self.angular_inertia);
        self.rotation = Quaternion::from_vector(self.angular_velocity * dt) * self.rotation;
        self.rotation.normalize();
    }
}
