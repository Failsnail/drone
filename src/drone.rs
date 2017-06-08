use object::Object;
use vector::Vector;
use matrix::Matrix;
use objectmanager;
use objectmanager::ObjectManager;


/* drone convention, topdown view;
   x
 -   -
|2| |1|
 -> <-  z
|3| |4|
 -   -

roll - rotation around x
pitch - rotation around z
yaw - rotation around y
*/

const BETA: f32 = -25f32; //15f32
const ALPHA: f32 = BETA * BETA / 4f32;

#[allow(dead_code)]
pub fn get_pwm (drone :&Object, a_target: Vector) -> (f32, f32, f32, f32) {
    //let beta = -25f32; //15f32
    //let alpha = beta * beta / 4f32;
    
    let local_to_world = drone.rotation.to_matrix();

    let derivative_action = drone.angular_velocity * BETA;
    
    let local_x :Vector = local_to_world.0[0];
    let local_y :Vector = local_to_world.0[1];
    let local_z :Vector = local_to_world.0[2];

    let axis_desired = Vector::normalize(
        Vector::cross(local_y, a_target)
    ).unwrap_or(Vector::null());

    let angle = Vector::angle(local_y, a_target);

    let proportional_action = axis_desired * angle * ALPHA;

    let total_action = derivative_action + proportional_action;
    
    let roll_action  = Vector::dot(total_action, local_x);
    let pitch_action = Vector::dot(total_action, local_z);
    let yaw_action   = Vector::dot(total_action, local_y);

    let thrust = Vector::magnitude(a_target) * drone.mass * Vector::cos_angle(local_y, a_target);

    (
        (thrust - roll_action + pitch_action + yaw_action) * 0.25f32,
        (thrust + roll_action + pitch_action - yaw_action) * 0.25f32,
        (thrust + roll_action - pitch_action + yaw_action) * 0.25f32,
        (thrust - roll_action - pitch_action - yaw_action) * 0.25f32,
    )
}

#[allow(dead_code)]
pub fn apply_pwm (object_manager: &mut ObjectManager, drone_tag: &objectmanager::ObjectTag , pwm1: f32, pwm2: f32, pwm3: f32, pwm4: f32) {
    let local_to_world :Matrix = object_manager.get_object(drone_tag).rotation.to_matrix();
        
    let force_local = Vector::ey() * (pwm1 + pwm2 + pwm3 + pwm4);
    let torque_local = Vector {
        x: -pwm1 + pwm2 + pwm3 - pwm4,
        y:  pwm1 - pwm2 + pwm3 - pwm4,
        z:  pwm1 + pwm2 - pwm3 - pwm4,
        w: 0f32,
    };
    
    let force  = local_to_world * force_local;
    let torque = local_to_world * torque_local;

    object_manager.apply_force_torque(force, torque, drone_tag);
}
