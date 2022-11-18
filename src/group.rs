use nalgebra::{Isometry3, RealField, Translation3, UnitQuaternion};
use serde::Serialize;
use std::fmt::Debug;

use super::{
    ComposableTransform, CoordinateSystem, Differentiable, IdentityTransform, InvertableTransform, IsGroup, IsManifold,
    IsManifoldElement, IsRobotFrame, IsTransform, ManifoldElement, Transform, R3, SE3, SO3,
};

pub trait IsGroupElement<
    RobotFrame: IsRobotFrame,
    Group: IsGroup + IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
>: IsManifoldElement<RobotFrame, Group, Repr>
{
    fn group_mul(self, rhs: Self) -> Self {
        assert!(self.coordinate_system() == rhs.coordinate_system());
        todo!()
    }
    fn identity_at(coordinate_system: CoordinateSystem<RobotFrame, Group, Repr>) -> Self;
    fn invert(self) -> Self;
}

impl<DstRobotFrame, SrcRobotFrame, Group, Repr> IsTransform<DstRobotFrame, Group, Repr, SrcRobotFrame, Group, Repr>
    for Transform<Repr, DstRobotFrame, Group, Repr, SrcRobotFrame, Group, Repr>
where
    DstRobotFrame: IsRobotFrame,
    SrcRobotFrame: IsRobotFrame,
    Group: IsGroup + IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
    ManifoldElement<DstRobotFrame, Group, Repr>: IsGroupElement<DstRobotFrame, Group, Repr>,
{
    fn dst(self) -> CoordinateSystem<DstRobotFrame, Group, Repr> {
        self.dst
    }
    fn src(self) -> CoordinateSystem<SrcRobotFrame, Group, Repr> {
        self.src
    }
    fn transform(
        self,
        point: ManifoldElement<SrcRobotFrame, Group, Repr>,
    ) -> ManifoldElement<DstRobotFrame, Group, Repr> {
        assert!(self.src() == point.coordinate_system());
        let src_in_dst_as_point = ManifoldElement::new(self.dst(), self.transform);
        let point_in_dst = ManifoldElement::new(self.dst(), point.coordinates);
        src_in_dst_as_point.group_mul(point_in_dst)
    }
}

impl<RobotFrame, Group, Repr> IdentityTransform<Repr, RobotFrame, Group, Repr, Repr>
    for Transform<Repr, RobotFrame, Group, Repr, RobotFrame, Group, Repr>
where
    RobotFrame: IsRobotFrame,
    Group: IsGroup + IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
    ManifoldElement<RobotFrame, Group, Repr>: IsGroupElement<RobotFrame, Group, Repr>,
{
    fn identity_at(
        coordinate_system: CoordinateSystem<RobotFrame, Group, Repr>,
    ) -> Transform<Repr, RobotFrame, Group, Repr, RobotFrame, Group, Repr> {
        let identity_point = ManifoldElement::identity_at(coordinate_system);
        Transform::new(coordinate_system, coordinate_system, identity_point.coordinates)
    }
}

impl<DstRobotFrame, SrcRobotFrame, Group, Repr> InvertableTransform<Repr, DstRobotFrame, SrcRobotFrame, Group, Repr>
    for Transform<Repr, DstRobotFrame, Group, Repr, SrcRobotFrame, Group, Repr>
where
    DstRobotFrame: IsRobotFrame,
    SrcRobotFrame: IsRobotFrame,
    Group: IsGroup + IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
    ManifoldElement<DstRobotFrame, Group, Repr>: IsGroupElement<DstRobotFrame, Group, Repr>,
{
    fn invert(self) -> Transform<Repr, SrcRobotFrame, Group, Repr, DstRobotFrame, Group, Repr> {
        let src_in_dst_as_point = ManifoldElement::new(self.dst(), self.transform);
        Transform::new(self.src(), self.dst(), src_in_dst_as_point.invert().coordinates)
    }
}

impl<RobotFrameA, RobotFrameB, RobotFrameC, Group, Repr>
    ComposableTransform<Transform<Repr, RobotFrameB, Group, Repr, RobotFrameC, Group, Repr>>
    for Transform<Repr, RobotFrameA, Group, Repr, RobotFrameB, Group, Repr>
{
    type Output = Transform<Repr, RobotFrameA, Group, Repr, RobotFrameC, Group, Repr>;

    fn compose_with(
        self,
        rhs: Transform<Repr, RobotFrameB, Group, Repr, RobotFrameC, Group, Repr>,
    ) -> Transform<Repr, RobotFrameA, Group, Repr, RobotFrameC, Group, Repr> {
        assert!(self.src() == rhs.dst());
        let b_in_a_as_pt = ManifoldElement::new(self.dst(), self.transform);
        let c_in_b_as_pt = ManifoldElement::new(self.dst(), rhs.transform);
        let c_in_a_as_pt = b_in_a_as_pt.group_mul(c_in_b_as_pt);
        Transform::new(self.dst(), rhs.src(), c_in_a_as_pt.coordinates)
    }
}

impl<T: Copy + RealField + Serialize, RobotFrame: IsRobotFrame> IsGroupElement<RobotFrame, SO3, UnitQuaternion<T>>
    for ManifoldElement<RobotFrame, SO3, UnitQuaternion<T>>
{
    fn group_mul(self, rhs: Self) -> Self {
        assert!(self.coordinate_system == rhs.coordinate_system);
        ManifoldElement::new(self.coordinate_system, self.coordinates * rhs.coordinates)
    }
    fn identity_at(coordinate_system: CoordinateSystem<RobotFrame, SO3, UnitQuaternion<T>>) -> Self {
        ManifoldElement::new(coordinate_system, UnitQuaternion::identity())
    }
    fn invert(self) -> Self {
        ManifoldElement::new(self.coordinate_system, self.coordinates.inverse())
    }
}

impl<T: Copy + RealField + Serialize, RobotFrame: IsRobotFrame> IsGroupElement<RobotFrame, SE3, Isometry3<T>>
    for ManifoldElement<RobotFrame, SE3, Isometry3<T>>
{
    fn group_mul(self, rhs: Self) -> Self {
        assert!(self.coordinate_system == rhs.coordinate_system);
        ManifoldElement::new(self.coordinate_system, self.coordinates * rhs.coordinates)
    }
    fn identity_at(coordinate_system: CoordinateSystem<RobotFrame, SE3, Isometry3<T>>) -> Self {
        ManifoldElement::new(coordinate_system, Isometry3::identity())
    }
    fn invert(self) -> Self {
        ManifoldElement::new(self.coordinate_system, self.coordinates.inverse())
    }
}

impl<T: Copy + RealField + Serialize, RobotFrame: IsRobotFrame> IsGroupElement<RobotFrame, R3, Translation3<T>>
    for ManifoldElement<RobotFrame, R3, Translation3<T>>
{
    fn group_mul(self, rhs: Self) -> Self {
        assert!(self.coordinate_system == rhs.coordinate_system);
        ManifoldElement::new(self.coordinate_system, self.coordinates * rhs.coordinates)
    }
    fn identity_at(coordinate_system: CoordinateSystem<RobotFrame, R3, Translation3<T>>) -> Self {
        ManifoldElement::new(coordinate_system, Translation3::identity())
    }
    fn invert(self) -> Self {
        ManifoldElement::new(self.coordinate_system, self.coordinates.inverse())
    }
}
