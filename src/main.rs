#[macro_use]
extern crate glium;
extern crate image;
extern crate rand;

mod object;
mod graphicsmanager;
mod utils;

mod drone;
mod spline;
use spline::Spline;
use spline::Bezier;
mod vector;
use vector::Vector;
mod matrix;
//use matrix::Matrix;
mod quaternion;
use quaternion::Quaternion;
mod objectmanager;


const MERGE_TIME :f32 = 1f32;
const DT :f32 = 0.02f32;  //timestep size
const DISTURB :bool = true;    

pub fn make_object (gm :&mut graphicsmanager::GraphicsManager, name: &str) -> object::Object {
    object::Object {
        scale: 1f32,
        mass: 1f32,
        angular_inertia: 600.0f32,
        position: Vector::origin(),
        velocity: Vector::null(),
        rotation: Quaternion::identity(),
        angular_velocity: Vector::null(),
        graphicsmodel_id:  gm.load_model(name),
    }
}

fn main () {
    let mut gm = graphicsmanager::GraphicsManager::new();
    let mut object_manager = objectmanager::ObjectManager::new();

    let target       :objectmanager::ObjectTag;
    let merge_target :objectmanager::ObjectTag;
    let drone        :objectmanager::ObjectTag;
    
    { //target
        let mut new_cube = make_object(&mut gm, "cube");
        new_cube.scale = 0.25f32;

        target = object_manager.push_object_tagged(new_cube);
    }
    { //merge_target
        let mut new_cube = make_object(&mut gm, "cube");
        new_cube.scale = 0.25f32;

        merge_target = object_manager.push_object_tagged(new_cube);
    }

    { //drone
        let mut new_drone = make_object(&mut gm, "drone");
        new_drone.scale = 0.6f32;
        new_drone.mass = 1f32;
        new_drone.angular_inertia = 1f32;
        new_drone.position = Vector::new(0f32, 0f32, -6f32, 1f32);

        drone = object_manager.push_object_tagged(new_drone);
    }

    let spline = spline::test_spline();

    { //put drone in initial state
        let (p, v, _) = spline.sample_all(0f32);
        let obj = object_manager.get_mut_object(&drone);
        obj.position = p;
        obj.velocity = v;
    }

    let mut t :f32 = 0f32;  //current time  

    println!("running!");
    
    loop {
        if gm.exit() {
            break;
        }

        
        t += DT;
        println!("t: {}", t);
        let t2 = t % spline.duration();
        let t3 = (t + MERGE_TIME) % spline.duration();

        
        {   //update target
            let (p, _, _) = spline.sample_all(t2);
            let obj = object_manager.get_mut_object(&target);
            obj.position = p;
        }

        {   //update merge_target
            let (p, _, _) = spline.sample_all(t3);
            let obj = object_manager.get_mut_object(&merge_target);
            obj.position = p;
        }

        
        //let gravity = Vector::null();
        let gravity = Vector::ey() * -10f32;
        
        let (mut pwm1, mut pwm2, mut pwm3, mut pwm4) = {
            let obj = object_manager.get_object(&drone);

            let (p_merge, v_merge, _) = spline.sample_all(t3);

            let mut s2 = Spline::new();
            s2.push_curve(
                Bezier::merge(
                    obj.position,
                    obj.velocity,
                    t,
                    p_merge,
                    v_merge,
                    t + MERGE_TIME
                )
            );

            let a = s2.sample_acceleration(t);

            drone::get_pwm(obj, a - gravity)
        };

        pwm1 = pwm1.max(0f32);
        pwm2 = pwm2.max(0f32);
        pwm3 = pwm3.max(0f32);
        pwm4 = pwm4.max(0f32);
        
        drone::apply_pwm(&mut object_manager, &drone, pwm1, pwm2, pwm3, pwm4);
        object_manager.apply_force(gravity, &drone);
        
        if DISTURB {
            object_manager.apply_force(Vector::random_unitvector() * 2f32, &drone);
            object_manager.apply_torque(Vector::random_unitvector() * 5f32, &drone);

            if t % 10f32 < 1f32 {
                object_manager.apply_force(Vector::ez() * -3f32, &drone);
            }
        }

            
        object_manager.update_physics(DT);
        
        //rendering
        gm.setup();
        object_manager.draw(&mut gm);
        gm.finish_frame();
    }
}

