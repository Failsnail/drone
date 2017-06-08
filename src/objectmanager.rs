use vector::Vector;
use object::Object;
use graphicsmanager::GraphicsManager;

pub struct ObjectTag (usize);

pub struct ObjectManager {
    objects: Vec<Object>,
    forces: Vec<Vector>,
    torques: Vec<Vector>,
}

#[allow(dead_code)]
impl ObjectManager {
    pub fn new () -> ObjectManager {
        ObjectManager {
            objects: Vec::<Object>::new(),
            forces:  Vec::<Vector>::new(),
            torques: Vec::<Vector>::new(),
        }
    }

    pub fn push_object (&mut self, new_object: Object) {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());

        self.objects.push(new_object);
        self.forces.push(Vector::null());
        self.torques.push(Vector::null());
    }

    pub fn push_object_tagged (&mut self, new_object: Object) -> ObjectTag {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());

        self.objects.push(new_object);
        self.forces.push(Vector::null());
        self.torques.push(Vector::null());

        ObjectTag(self.objects.len() - 1)
    }

    pub fn get_object (&self, tag: &ObjectTag) -> &Object {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());

        &self.objects[tag.0]
    }

    pub fn get_mut_object (&mut self, tag: &ObjectTag) -> &mut Object {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());

        &mut self.objects[tag.0]
    }

    pub fn apply_force (&mut self, force: Vector, tag: &ObjectTag) {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());

        self.forces[tag.0] += force;
    }

    pub fn apply_torque (&mut self, torque: Vector, tag: &ObjectTag) {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());
        
        self.torques[tag.0] += torque;
    }

    pub fn apply_force_torque (&mut self, force: Vector, torque: Vector, tag: &ObjectTag) {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());

        self.forces[tag.0] += force;
        self.torques[tag.0] += torque;
    }

    pub fn update_physics (&mut self, dt: f32) {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());
        
        for n in 0 .. self.objects.len() {
            if self.objects[n].mass.is_finite() {
                self.objects[n].update(self.forces[n], self.torques[n], dt);
                self.forces[n] = Vector::null();
                self.torques[n] = Vector::null();
            }
        }
    }

    pub fn draw (&self, gm: &mut GraphicsManager) {
        assert!(self.objects.len() == self.forces.len());
        assert!(self.objects.len() == self.torques.len());
        
        for obj in &self.objects {
            gm.draw_object(obj);
        }
    }
}
