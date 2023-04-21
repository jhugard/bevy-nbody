use bevy::prelude::*;
use rayon::prelude::*;

use crate::components::{Position, Mass, Radius};

/// 3D Bounding Box
#[derive(Clone,Copy)]
pub struct BBox3 {
    pmin: Vec3,
    pmax: Vec3,
}


/// 3D Bounding Box Implementation
impl BBox3 {

   pub fn new( p1:&Vec3, p2:&Vec3 ) -> Self {
        let (xmin,xmax) = if p1.x < p2.x {(p1.x,p2.x)} else {(p2.x,p1.x)};
        let (ymin,ymax) = if p1.y < p2.y {(p1.y,p2.y)} else {(p2.y,p1.y)};
        let (zmin,zmax) = if p1.z < p2.z {(p1.z,p2.z)} else {(p2.z,p1.z)};
        BBox3 { pmin:Vec3::new(xmin,ymin,zmin), pmax:Vec3::new(xmax,ymax,zmax) }
    }

    pub fn from<'a,I>(mut points:I) -> Self
    where I:Iterator<Item=&'a Vec3>
    {
        if let Some(head) = points.next() {
            let mut bounds = Self::new(&head,&head);
            bounds.encompass_all(points);
            bounds
        } else {
            Self::new(&Vec3::default(),&Vec3::default())
        }
    }

    pub fn center(&self) -> Vec3 {
        let (minx,miny,minz) = (self.pmin.x, self.pmin.y, self.pmin.z);
        let (maxx,maxy,maxz) = (self.pmax.x, self.pmax.y, self.pmax.z);
        Vec3::new(
            minx + (maxx-minx) / 2.0,
            miny + (maxy-miny) / 2.0,
            minz + (maxz-minz) / 2.0)
    }

    /// Split the bounding box into 8 sub-spaces
    pub fn subdivide(&self) -> [BBox3; 8] {

        let pmid = self.center();

        let (minx,miny,minz) = (self.pmin.x, self.pmin.y, self.pmin.z);
        let (maxx,maxy,maxz) = (self.pmax.x, self.pmax.y, self.pmax.z);

        [
            // NOTE: THe following order must be retained to use quadrant_index_for()
            // Note the pattern: x^0 y^1 z^2

            BBox3::new( &Vec3::new(minx, miny, minz), &pmid ),
            BBox3::new( &Vec3::new(maxx, miny, minz), &pmid ),
            BBox3::new( &Vec3::new(minx, maxy, minz), &pmid ),
            BBox3::new( &Vec3::new(maxx, maxy, minz), &pmid ),
            BBox3::new( &Vec3::new(minx, miny, maxz), &pmid ),
            BBox3::new( &Vec3::new(maxx, miny, maxz), &pmid ),
            BBox3::new( &Vec3::new(minx, maxy, maxz), &pmid ),
            BBox3::new( &Vec3::new(maxx, maxy, maxz), &pmid ),

        ]
    }

    /// Return the index value a point falls in, assuming 8 subspaces as created
    /// by the subdivide() method
    pub fn quadrant_index_for(&self, p:&Vec3) -> usize {
        let c = self.center();
        let mut index = 0;

        if p.z > c.z {
            index += 4;
        }
        if p.y > c.y {
            index += 2;
        }
        if p.x > c.x {
            index += 1;
        }
        index
    }

    /// Extend the Bounding Box to include the specifid point
    pub fn encompass( &mut self, p: &Vec3 ) {
        if p.x < self.pmin.x { self.pmin.x = p.x }
        else if p.x > self.pmax.x { self.pmax.x = p.x }

        if p.y < self.pmin.y { self.pmin.y = p.y }
        else if p.y > self.pmax.y { self.pmax.y = p.y }
        
        if p.z < self.pmin.z { self.pmin.z = p.z }
        else if p.z > self.pmax.z { self.pmax.z = p.z }
    }

    /// Extend the Bounding Box to include all points in an iterator
    pub fn encompass_all<'a, I> ( &mut self, points: I)
        where I : Iterator<Item=&'a Vec3>
    {
        for p in points {
            self.encompass(p);
        }
    }

    /// Is the bounding box empty?
    pub fn is_empty(&self) -> bool {
        self.pmin == self.pmax
    }

    pub fn contains(&self, p: &Vec3) -> bool {
        let not_contains =
            self.pmin.x > p.x || p.x > self.pmax.x ||
            self.pmin.y > p.y || p.y > self.pmax.y ||
            self.pmin.z > p.z || p.z > self.pmax.z;
        !not_contains
    }
}

impl Default for BBox3 {
    fn default() -> BBox3 {
        BBox3 { pmin:Vec3::ZERO, pmax:Vec3::ZERO }
    }
}

// NBody
pub struct NBody {
    pub entity: Entity,
    pub position: Vec3,
    pub mass: f32,
    pub radius: f32,
}

impl NBody {
    pub fn new(entity:Entity, position: Vec3, mass: f32, radius: f32 ) -> Self
    {
        let density = 10.0;
        Self { entity, position, mass, radius }
    }
}

pub struct BHTreeNode {
    mass: f32,
    center_of_mass: Vec3,
    bounds: BBox3,
    children: Option<Box<[BHTreeNode; 8]>>,
    body: Option<NBody>,
 }

 impl<'a> BHTreeNode {

    /// Construct a new Barnes-Hut tree node, given a bounding box
    pub fn new(bounds:&BBox3) -> Self {
        BHTreeNode { mass:0.0, center_of_mass:bounds.center(), bounds:bounds.clone(), children:None, body:None }
    }

    /// Create a BHTree from an iterator and calculate bounds from the bodies
    /// as well as total mass and center of mass for each node.
    pub fn from<I>(bounds:&BBox3, bodies:I) -> BHTreeNode
    where I:Iterator<Item=(Entity,&'a Position,&'a Mass, &'a Radius)>
    {
        // Create our top-level tree node
        let mut root = BHTreeNode::new(bounds);

        // Insert all bodies
        for (e,p,m,r) in bodies {
            root.insert( NBody::new(e,p.0,m.0,r.0));
            //root.insert_no_update( NBody::new(e,p.0,m.0));
        }

        // // Update total mass and center of mass for root and all children
        // root.update_all();

        root
    }

    /// Insert a node into the tree, and recalculate mass and center
    /// of mass.
    pub fn insert( &mut self, body:NBody) {
        // If node has children, insert new body into the proper child
        // Otherwise, if the node has no body, save the new body
        // Otherwise, move both new body and current body into the proper child

        match &mut self.children {
            Some(children) => {
                let ix = self.bounds.quadrant_index_for(&body.position);
                children[ix].insert(body);
            },
            None => {
                if self.body.is_none() {
                    self.body = Some(body)
                } else {
                    self.children = Some(self.subdivide());
                    let obody = self.body.take().unwrap();
                    self.insert(obody);
                    self.insert(body);
                }
            }
        }
        self.update();
    }

    /// Recalculate the total mass and center of mass of self from immediate
    /// children, after inserting one or more NBodies into this node.
    pub fn update(&mut self) {
        // Set mass and center of mass if we are an exterior node
        if self.body.is_some() {
            let body = self.body.as_ref().unwrap();
            self.center_of_mass = body.position;
            self.mass = body.mass;
        } else {
            match &self.children {
                None => (),
                Some(children) => {
                    let (total_mass,center_of_mass) =
                        BHTreeNode::total_mass_and_center_of_mass(children.iter());
                    self.mass = total_mass;
                    self.center_of_mass = center_of_mass;
                }
            }
        }
    }

    /// Insert a node into the tree, but do NOT recalculate mass and center
    /// of mass.  Useful for doing a bulk insert of all bodies, followed
    /// by a single update.
    pub fn insert_no_update( &mut self, body:NBody) {
        // If node has children, insert new body into the proper child
        // Otherwise, if the node has no body, save the new body
        // Otherwise, move both new body and current body into the proper child

        match &mut self.children {
            Some(children) => {
                let ix = self.bounds.quadrant_index_for(&body.position);
                children[ix].insert(body);
            },
            None => {
                if self.body.is_none() {
                    self.body = Some(body);
                } else {
                    self.children = Some(self.subdivide());
                    let obody = self.body.take().unwrap();
                    self.insert_no_update(body);
                    self.insert_no_update(obody);
                }
            }
        }
    }

    /// Recalculate the total mass and center of mass for all children
    pub fn update_all(&mut self) {
        if let Some(children) = &mut self.children {
            for child in children.as_mut_slice() {
                child.update_all();
            }
        }
        self.update();
    }

    fn total_mass<I>(nodes:I) -> f32
        where I : Iterator<Item=&'a BHTreeNode>
    {
        nodes.fold(0.0, |acc,c| acc + c.mass )
    }

    const THETA:f32 = 0.5;

    /// This is probably wrong, but return the maximum dimention from amongst x,y,z
    fn size(&self) -> f32 {
        let dim = self.bounds.pmax - self.bounds.pmin;
        dim.x.max(dim.y.max(dim.z))
    }

    /// Calculate the forces against the specified body
    fn calculate_acceleration(&self, body: &NBody ) -> (Vec3, Vec<Entity>) {

        let mut accel = Vec3::ZERO;
        let mut collided_with = Vec::new();

        // Process exterior node (no children, ends recursion)
        if let Some(other) = self.body.as_ref() {
            if std::ptr::eq(body,other) {
                // accel = Vec3::ZERO;
            }
            else {
                let dir = (other.position - body.position).normalize();
                let dist2 = other.position.distance_squared(body.position);
                let radaii = body.radius+other.radius;
                let radaii2 = radaii * radaii;
                if dist2 > radaii2 {
                    accel = dir * (crate::G * other.mass / dist2);
                } else {
                    collided_with.push(body.entity);
                }
            }
        }

        // If point is in this node OR is close to this node, recurse into children
        else if self.bounds.contains(&body.position)
            || self.size() / self.center_of_mass.distance(body.position) >= Self::THETA
        {
            //let mut accel = Vec3::ZERO;
            if let Some(children) = &self.children {
                for child in children.iter() {
                    let (deltav,mut collisions) = child.calculate_acceleration(body); 
                    accel += deltav;
                    collided_with.append(&mut collisions);
                }
            }
        }

        // Else, process using this node approx center of mass
        // (ends recursion)
        else
        {
            let dir = (self.center_of_mass - body.position).normalize();
            let dist2 = self.center_of_mass.distance_squared(body.position);
            let radaii = body.radius + body.radius; // approx 
            if dist2 >= radaii {
                accel = dir * (crate::G * self.mass / dist2);
            }
        }

        if !accel.x.is_finite() {
            accel.x = 0.0;
        }
        if !accel.y.is_finite() {
            accel.y = 0.0;
        }
        if !accel.z.is_finite()
        {
            accel.z = 0.0;
        }
        
        (accel,collided_with)
    }

    /// update_forces
    pub fn collect_accelerations(self) -> Vec<(Entity,Vec3,Vec<Entity>)> {

        self.iter()
            .par_bridge()
            .map( | body | {
                let (accel,collisions) = self.calculate_acceleration(body);
                (body.entity,accel,collisions)
            })
            .collect()
    }


    /// Calculate total mass and center of mass using Kahan summation algorithm
    fn total_mass_and_center_of_mass<I>(nodes:I) -> (f32,Vec3)
        where I : Iterator<Item=&'a BHTreeNode>
    {
        let mut total_mass = 0.0;
        let mut cm = Vec3::ZERO;
        let mut compensation = Vec3::ZERO;

        for node in nodes {
            let y = node.center_of_mass * node.mass - compensation;
            let t = cm + y;
            compensation = (t - cm) - y;
            cm = t;
            total_mass += node.mass;
        }

        cm = cm / total_mass;

        (total_mass, cm)
    }


    // Split this node into 8 sub nodes
    fn subdivide(&self) -> Box<[Self;8]> {
        let subbounds = self.bounds.subdivide();
        Box::new([        
            BHTreeNode::new(&subbounds[0]),
            BHTreeNode::new(&subbounds[1]),
            BHTreeNode::new(&subbounds[2]),
            BHTreeNode::new(&subbounds[3]),
            BHTreeNode::new(&subbounds[4]),
            BHTreeNode::new(&subbounds[5]),
            BHTreeNode::new(&subbounds[6]),
            BHTreeNode::new(&subbounds[7]),
        ])
    }

    pub fn iter(&'a self) -> BHTreeNodeIter<'a> {
        BHTreeNodeIter::new(self)
    }

    pub fn iter_mut(&'a mut self) -> BHTreeNodeIterMut<'a> {
        BHTreeNodeIterMut::new(self)
    }
 }


 pub struct BHTreeNodeIter<'a>
 {
     stack : Vec<&'a BHTreeNode>,
 }
 
 impl<'a> BHTreeNodeIter<'a> {
     fn new(root: &'a BHTreeNode) -> Self {
         let mut stack = Vec::new();
         stack.push(root); 
         BHTreeNodeIter { stack }
     }
 }
 
 impl<'a> Iterator for BHTreeNodeIter<'a> {
     type Item = &'a NBody;
 
     fn next(&mut self) -> Option<Self::Item> {
 
         let next_node = self.stack.pop();
 
         match next_node {
             None => None,
             Some(node) => {
                 if let Some(body) = node.body.as_ref() {
                     Some(body)
                 } else {
                     if let Some(children) = node.children.as_ref() {
                         for child in children.iter() {
                             self.stack.push(child)
                         }
                     }
                     self.next()
                 }
             }
         }
     }
 }
 
pub struct BHTreeNodeIterMut<'a>
{
    stack : Vec<&'a mut BHTreeNode>,
}

impl<'a> BHTreeNodeIterMut<'a> {
    fn new(root: &'a mut BHTreeNode) -> Self {
        let mut stack = Vec::new();
        stack.push(root); 
        BHTreeNodeIterMut { stack }
    }
}

impl<'a> Iterator for BHTreeNodeIterMut<'a> {
    type Item = &'a mut NBody;

    fn next(&mut self) -> Option<Self::Item> {

        let next_node = self.stack.pop();

        match next_node {
            None => None,
            Some(node) => {
                if let Some(body) = node.body.as_mut() {
                    Some(body)
                } else {
                    if let Some(children) = node.children.as_mut() {
                        for child in children.iter_mut() {
                            self.stack.push(child)
                        }
                    }
                    self.next()
                }
            }
        }
    }
}
