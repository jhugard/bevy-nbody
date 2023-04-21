use std::f32::consts::PI;

use bevy::{prelude::*, diagnostic::{LogDiagnosticsPlugin, FrameTimeDiagnosticsPlugin}};
use bevy_prototype_lyon::prelude::*;
use bhtree::{BBox3};
use components::*;
use rand::prelude::*;

mod components;
mod bhtree;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_startup_system(setup_global)
        .add_startup_system(setup_bodies)
        .add_system(player_camera_control)
        .add_system(bh_gravity_acceleration_system)
        .add_system(collision_system.after(bh_gravity_acceleration_system))
        .add_system(apply_acceleration_system.after(collision_system))
        .add_system(movement_system.after(apply_acceleration_system))
        .add_system(position_update_system.after(movement_system))
        .add_system(direction_update_system.after(apply_acceleration_system))
        .run();
}

const G: f32 = 6.674*10e-11;
const SPEED: f32 = 10e4;
const OMEGA: f32 = 1.0;    // Ignore gravity calculations on bodies closer than this to each other

fn gravity_acceleration_system(
    mut q: Query<(&Position, &Mass, &Radius, &mut Acceleration)>,
) {

    let mut others: Vec<(&Position, &Mass, &Radius, Mut<Acceleration>)> = Vec::new();

    for (pos, mass, radius, mut accel) in q.iter_mut() {

        accel.0 = Vec3::ZERO;

        for (opos, omass, oradius, oaccel) in others.iter_mut() {
            
            let diff = opos.0 - pos.0;
            let dist2 = diff.length_squared();
            let radii = radius.0 + oradius.0;
            let radii2 = radii*radii;

            if dist2 > radii2 {
                if let Some(dir) = diff.try_normalize() {
                    let f  = G * omass.0 / dist2;
                    let of = G *  mass.0 / dist2;
                    
                    oaccel.0 -= of * dir;
                }
            }
        }
        others.push( (pos,mass,radius,accel) );

    }
}

fn bh_gravity_acceleration_system(
    mut q: Query<(Entity, &Position, &Mass, &Radius, &mut Acceleration)>,
    mut commands: Commands,
) {

    let bounds = BBox3::from( q.iter().map(|(_,p,_,_,_)| &p.0));
    let bhtree = bhtree::BHTreeNode::from(&bounds, q.iter().map(|(e,p,m,r,_)| (e,p,m,r)));

    bhtree.collect_accelerations().iter()
        .for_each(|(ent,newaccel,collisions)| {
            if let Ok(mut accel) = q.get_component_mut::<Acceleration>(*ent) {
                accel.0 = *newaccel;
            }
            commands.entity(*ent).insert( Collisions(collisions.to_owned()) );
        });
}

fn collision_system(
    mut q: Query<(Entity, &Mass, &Position, &Velocity, &Acceleration, &Collisions)>,
    mut commands: Commands,
) {
    let mut updates = Vec::new();
    
    for (entity, mass, position, velocity, accel, collisions ) in q.iter() {
        let mut newmass = mass.0;
        let position = position.0;
        let mut newvelocity = velocity.0;
        let mut newaccel = accel.0;
        let despawns = collisions.0.clone();
    
        for centity in &collisions.0 {
            if let (Ok(cmass), Ok(cvelocity), Ok(caccel)) = (
                q.get_component::<Mass>(*centity),
                q.get_component::<Velocity>(*centity),
                q.get_component::<Acceleration>(*centity),
            ) {
                let cmass = cmass.0;
                let cvelocity = cvelocity.0;
                let caccel = caccel.0;
                let totalmass = newmass + cmass;

                newaccel = ( (newaccel * newmass) + (caccel * cmass) ) / totalmass;
                newvelocity = ( (newvelocity * newmass) + (cvelocity * cmass ) ) / totalmass;
                newmass = totalmass;

                commands.entity(*centity).despawn_recursive();

            }
        }         
        updates.push( (entity,newmass,position,newvelocity,newaccel,despawns) );
    }

    for (entity,newmass,_,newvelocity,_,despawns) in updates {
        if let Ok((_, mass, position, velocity, accel,_)) = q.get_mut(entity) {
            if !despawns.is_empty() {
                commands.entity(entity).despawn_recursive();
                setup_body( &mut commands, newmass, position.0, newvelocity );
            }
        }
    }
}



fn apply_acceleration_system(
    time: Res<Time>,
    mut q: Query<(&mut Velocity, &Acceleration)>
) {
    for (mut v, acc) in q.iter_mut() {
        v.0 += SPEED * time.delta_seconds() * acc.0;
    }
}

fn movement_system(
    time: Res<Time>,
    mut q: Query<(&mut Position, &Velocity)>,
) {
    for (mut position, velocity) in q.iter_mut() {
        position.0 += SPEED * time.delta_seconds() * velocity.0;
    }
}

fn position_update_system(
    mut q: Query<(&mut Transform, &Position)>,
) {
    for (mut transform, position ) in q.iter_mut() {
        transform.translation = position.0;
    }
}

fn direction_update_system(
    mut q: Query<(&mut Transform, &Velocity)>,
) {
    for (mut transform, velocity) in q.iter_mut() {
        if let Some(dir) = velocity.0.try_normalize() {
            let angle = dir.y.atan2(dir.x);
            transform.rotation = Quat::from_rotation_z(angle);
        }
    }
}

// fn collision_system(
//     mut q: Query<(&Entity, &Transform, &Radius, &mut Velocity)>
// ) {
//     let other = Vec::new();
//     for (e,t,r,v,a) in q.iter() {
//         for (oe,ot,or, mut oa) in other.iter() {
//             collide_dist = or + r;
//             dist2 = (ot.translation - t.translation).length_squared();
//             if collide_dist <= dist2 && collide_dist <= dist2.sqrt() {
//                 let dv = ov.0 + v.0;

//             }
//         }
//         other.push( (e,t,r,v,a) );
//     }
// }

// const MSOL : f32 = 1.989e+30;
// const MSOL : f32 = 1989.0;
// const MEARTH : f32 = MSOL / 333_000.0;
// const MVENUS : f32 = MSOL /   1_047.0;

//const AU : f32 = 149_597_870.7 * 1000.0;
const AU : f32 = 149.0;

fn setup_global(mut commands: Commands)
{
    let mut camera = Camera2dBundle::default();
    camera.projection.scale = 5.0;

    commands
        .spawn( camera )
        ;

}

const CAMERA_ZOOM_SPEED_PER_SEC : f32 = 2.0;
const CAMERA_PAN_SPEED_PER_SEC : f32 = 1.0;

fn player_camera_control(kb: Res<Input<KeyCode>>, time: Res<Time>, mut query: Query<&mut OrthographicProjection>) {
    let dist = CAMERA_ZOOM_SPEED_PER_SEC * time.delta().as_secs_f32();

    let mut dorg = Vec2::ZERO;

    for mut projection in query.iter_mut() {
        let mut log_scale = projection.scale.ln();

        if kb.pressed(KeyCode::PageUp) {
            log_scale -= dist;
        }
        if kb.pressed(KeyCode::PageDown) {
            log_scale += dist;
        }
        if kb.pressed(KeyCode::Left) {
            dorg.x = 1.0;
        }
        else if kb.pressed(KeyCode::Right) {
            dorg.x = -1.0;
        }
        if kb.pressed(KeyCode::Up) {
            dorg.y = -1.0;
        } else if kb.pressed(KeyCode::Down) {
            dorg.y = 1.0;
        }

        projection.scale = log_scale.exp();
        projection.viewport_origin += dorg * CAMERA_PAN_SPEED_PER_SEC * time.delta().as_secs_f32();
    }
}

fn stable_orbit_particles(central_mass:f32, num_bodies:usize, radius:f32) -> Vec<(f32,Vec3,Vec3)> {
    let mut particles = Vec::new();
    let mut rng = rand::thread_rng();

    // Set up the center particle with mass M
    let center_pos = Vec3::new(0.0, 0.0, 0.0);
    let center_vel = Vec3::new(0.0, 0.0, 0.0);
    let center_mass = central_mass;
    let center_particle = (center_mass, center_pos, center_vel);
    particles.push(center_particle);

    // Set up the orbiting particles with random positions and velocities
    for _ in 1..num_bodies {
        let r = radius * (1.0 + 0.2 * (rng.gen::<f32>() - 0.5));
        let theta = 2.0 * PI * rng.gen::<f32>();
        let x = r * theta.cos();
        let y = r * theta.sin();
        let z = 0.0;
        let pos = Vec3::new(x, y, z);

        let v_circ = (G * central_mass / r).sqrt();
        let vx = -v_circ * theta.sin();
        let vy = v_circ * theta.cos();
        let vz = 0.0;
        let vel = Vec3::new(vx, vy, vz);

        let mass = rng.gen_range(0.1..1.0) * 10.0;
        let particle = (mass, pos, vel);
        particles.push(particle);
    }

    particles
}


fn setup_bodies(mut commands: Commands)
{

    // // SOL
    // setup_body(&mut commands, 1.0 * MSOL, Vec3::ZERO, Vec3::ZERO);

    // // EARTH
    // setup_body(&mut commands, MEARTH, Vec3::new(1.0*AU, 0.0), Vec3::new( 0.0, 10.0) );

    for (mass, pos, deltav) in stable_orbit_particles(200000.0, 100, 400.0) {
         setup_body(&mut commands, mass, pos, deltav );
    }


    // let mut rng = rand::thread_rng();

    // let rotation = Quat::from_rotation_z(PI/2.0);     // i.e., 90 degrees around Z axis
    // let rot_90 = Mat4::from_quat(rotation);

    // for x in 1..=100 {
    //     for y in 1..=100 {
    //         let mass = 600_000.0;
    //         let pos = Vec3::new( x as f32 * 100.0, y as f32 * 100.0, 0.0 );
    //         let deltav = Vec3::ZERO;

    //         setup_body(&mut commands, mass, pos, deltav);
    //     }
    // }

    // for _ in 1..=10000 {
    //     let mass = rng.gen_range( 2000.0 ..= 20000000.0 );
    //     let pos = Vec3::new( rng.gen_range( -40000.0 ..= 40000.0 ), rng.gen_range( -40000.0 ..= 40000.0 ), 0.0 );

    //     let distance_to_origin = pos.distance(Vec3::ZERO);
    //     let dir = rot_90.transform_vector3((Vec3::ZERO - pos).normalize());
    //     let speed = 250.0;
    //     let deltav = dir*speed;

    //     //let deltav = Vec3::new( rng.gen_range( -10.0 ..= 10.0 ), rng.gen_range( -10.0 ..= 10.0 ), 0.0 );
    //     //let deltav = Vec3::ZERO;

    //     setup_body(&mut commands, mass, pos, deltav );
    // }

}

fn setup_body(commands: &mut Commands, mass_kg: f32, center: Vec3, deltav_mps: Vec3 )
{
    let density = 10.0;
    let volume = mass_kg / density;
    let radius = ((3.0 * volume) / (4.0 * std::f32::consts::PI)).cbrt();

    let components = (
        Position(center),
        Radius(radius),
        Mass(mass_kg),
        Velocity(deltav_mps),
        Acceleration(Vec3::ZERO),
        );

    let surface = shapes::Circle {
            center: Vec2::ZERO,
            radius };
    let dir = shapes::Line(
            Vec2::new(radius, 0.0),
            Vec2::new(radius + (radius * 0.50), 0.0) );
    let path = ShapePath::new()
        .add(&surface)
        .add(&dir)
        .build()
        ;
    
    let transform = Transform::from_translation( Vec3::new( center.x, center.y, 0.0 ) );
    
    commands.spawn((
        ShapeBundle {
            path,
            transform,
            ..default()
        },
        Stroke::new(Color::WHITE, 1.0),
        Fill::color(Color::WHITE),
    )).insert(components);
}
