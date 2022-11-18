use serde::Serialize;
use std::{fmt::Debug, hash::Hash, marker::PhantomData};

use super::{IsManifold, Differentiable};

pub trait IsRobotFrame: Debug + Default + Copy + Eq + Hash + Serialize {}

pub trait IsRobotPart: IsRobotFrame {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct CoordinateFrame<RobotFrame: IsRobotFrame> {
    pub time: u64,
    pub robot_frame: RobotFrame,
}

impl<RobotFrame: IsRobotFrame> CoordinateFrame<RobotFrame> {
    pub fn at_time(time: u64) -> Self {
        Self { time, robot_frame: RobotFrame::default() }
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct CoordinateSystem<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize> {
    _repr: PhantomData<Repr>,
    pub frame: CoordinateFrame<RobotFrame>,
    pub manifold: Manifold,
}

impl<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize>
    CoordinateSystem<RobotFrame, Manifold, Repr>
{
    pub fn at_frame(id: CoordinateFrame<RobotFrame>) -> Self {
        Self { _repr: PhantomData, frame: id, manifold: Manifold::default() }
    }

    pub fn at_time(time: u64) -> Self {
        Self::at_frame(CoordinateFrame::at_time(time))
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct ManifoldElement<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize> {
    pub coordinate_system: CoordinateSystem<RobotFrame, Manifold, Repr>,
    pub coordinates: Repr,
}

impl<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize>
    ManifoldElement<RobotFrame, Manifold, Repr>
{
    pub fn new(coordinate_system: CoordinateSystem<RobotFrame, Manifold, Repr>, coordinates: Repr) -> Self {
        Self { coordinate_system, coordinates }
    }
}

pub trait IsManifoldElement<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize>:
    Debug + Copy + Serialize
{
    fn coordinate_system(self) -> CoordinateSystem<RobotFrame, Manifold, Repr>;
}

impl<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize>
    IsManifoldElement<RobotFrame, Manifold, Repr> for ManifoldElement<RobotFrame, Manifold, Repr>
{
    fn coordinate_system(self) -> CoordinateSystem<RobotFrame, Manifold, Repr> {
        self.coordinate_system
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct FrontLeftCameraFrame {}
impl IsRobotFrame for FrontLeftCameraFrame {}
impl IsRobotPart for FrontLeftCameraFrame {}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct FrontRightCameraFrame {}
impl IsRobotFrame for FrontRightCameraFrame {}
impl IsRobotPart for FrontRightCameraFrame {}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct RearLeftCameraFrame {}
impl IsRobotFrame for RearLeftCameraFrame {}
impl IsRobotPart for RearLeftCameraFrame {}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct RearRightCameraFrame {}
impl IsRobotFrame for RearRightCameraFrame {}
impl IsRobotPart for RearRightCameraFrame {}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct ImuFrame {}
impl IsRobotFrame for ImuFrame {}
impl IsRobotPart for ImuFrame {}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct CadFrontLeftCameraFrame {}
impl IsRobotFrame for CadFrontLeftCameraFrame {}
impl IsRobotPart for CadFrontLeftCameraFrame {}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct RobotBaseFrame {}
impl IsRobotFrame for RobotBaseFrame {}
impl IsRobotPart for RobotBaseFrame {}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct GravityCorrectedFrontLeftCameraFrame {}
impl IsRobotFrame for GravityCorrectedFrontLeftCameraFrame {}

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct GravityCorrectedRobotBaseFrame {}
impl IsRobotFrame for GravityCorrectedRobotBaseFrame {}

impl<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize> PartialEq
    for CoordinateSystem<RobotFrame, Manifold, Repr>
{
    fn eq(&self, other: &Self) -> bool {
        self.frame == other.frame && self.manifold == other.manifold
    }
}
impl<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize> Eq
    for CoordinateSystem<RobotFrame, Manifold, Repr>
{
}
impl<RobotFrame: IsRobotFrame, Manifold: IsManifold + Differentiable, Repr: Debug + Copy + Serialize> Hash
    for CoordinateSystem<RobotFrame, Manifold, Repr>
{
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        (self.frame, self.manifold).hash(state)
    }
}
