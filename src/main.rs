use bevy::{prelude::*, core::Zeroable, ecs::system::InsertBundle, render::camera::ScalingMode, diagnostic::{LogDiagnosticsPlugin, FrameTimeDiagnosticsPlugin}};
use bevy_prototype_lyon::prelude::*;
use components::*;
use rand::prelude::*;

mod components;
mod octtree;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(ShapePlugin)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_startup_system(setup_global)
        .add_startup_system(setup_bodies)
        .add_system(gravity_acceleration_system)
        .add_system(acceleration_system.after(gravity_acceleration_system))
        .add_system(movement_system.after(acceleration_system))
        .add_system(position_update_system.after(movement_system))
        .add_system(direction_update_system.after(acceleration_system))
        .run();
}

const G: f32 = 6.674*10e-11;
const SPEED: f32 = 10e7;
const OMEGA: f32 = 1.0;    // Ignore gravity calculations on bodies closer than this to each other

fn gravity_acceleration_system(
    mut q: Query<(&Position, &Mass, &mut Acceleration)>,
) {

    let mut others: Vec<(&Position, &Mass, Mut<Acceleration>)> = Vec::new();

    for (pos, mass, mut accel) in q.iter_mut() {

        accel.0 = Vec3::ZERO;

        for (opos, omass, oaccel) in others.iter_mut() {
            
            let diff = opos.0 - pos.0;
            let dist2 = diff.length_squared();

            if dist2 > OMEGA {
                if let Some(dir) = diff.try_normalize() {
                    let f  = G * omass.0 / dist2;
                    let of = G *  mass.0 / dist2;
                    
                    accel.0 += f * dir;
                    oaccel.0 -= of * dir;
                }
            }
        }
        others.push( (pos,mass,accel) );

    }
}

fn acceleration_system(
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
        position.0 += time.delta_seconds() * velocity.0;
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

//const MSOL : f32 = 1.989e+30;
const MSOL : f32 = 1989.0;
const MEARTH : f32 = MSOL / 333_000.0;
const MVENUS : f32 = MSOL / 1_047.0;

//const AU : f32 = 149_597_870.7 * 1000.0;
const AU : f32 = 149.0;

fn setup_global(mut commands: Commands)
{
    let mut camera = OrthographicCameraBundle::new_2d();
    camera.orthographic_projection.scale = 1.0;

    commands
        .spawn_bundle( camera )
        ;

}

fn setup_bodies(mut commands: Commands)
{

    // // SOL
    // setup_body(&mut commands, 1.0 * MSOL, Vec3::ZERO, Vec3::ZERO);

    // // EARTH
    // setup_body(&mut commands, MEARTH, Vec3::new(1.0*AU, 0.0), Vec3::new( 0.0, 10.0) );

    let mut rng = rand::thread_rng();

    for _ in 1..=100 {
        let mass = rng.gen_range( 2000.0 ..= 20000.0 );
        let pos = Vec3::new( rng.gen_range( -400.0 ..= 400.0 ), rng.gen_range( -400.0 ..= 400.0 ), 0.0 );
        //let deltav = Vec3::new( rng.gen_range( -10.0 ..= 10.0 ), rng.gen_range( -10.0 ..= 10.0 ), 0.0 );
        let deltav = Vec3::ZERO;

        setup_body(&mut commands, mass, pos, deltav );
    }

}

fn setup_body(commands: &mut Commands, mass_kg: f32, center: Vec3, deltav_mps: Vec3 )
{
    let density = 1.0;
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
    let shape_path = ShapePath::new()
        .add(&surface)
        .add(&dir)
        .build()
        ;
    
    let transform = Transform::from_translation( Vec3::new( center.x, center.y, 0.0 ) );
    
    let geometry = GeometryBuilder::new()
        .add(&shape_path)
        .build(
            DrawMode::Stroke( StrokeMode::new(Color::WHITE, 1.0)),
            transform)
        ;

    let entity = commands.spawn().id();

    commands.entity(entity)
        .insert_bundle(components)
        .insert_bundle(geometry)
        ;

}