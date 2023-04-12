use bevy::{prelude::*};

#[derive(Component)]
pub struct Position(pub Vec3);

#[derive(Component)]
pub struct Mass(pub f32);

#[derive(Component)]
pub struct Velocity(pub Vec3);

#[derive(Component)]
pub struct Acceleration(pub Vec3);

#[derive(Component)]
pub struct Radius(pub f32);
