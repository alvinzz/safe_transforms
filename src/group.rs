use nalgebra::{Isometry3, RealField, Translation3, UnitQuaternion};
use serde::Serialize;
use std::fmt::Debug;

use super::{
    ComposableTransform, CoordinateSystem, Differentiable, IdentityTransform, InvertableTransform,
    IsCoordinateFrameId, IsGroup, IsManifold, IsManifoldElement, IsTransform, ManifoldElement,
    Transform, R3, SE3, SO3,
};

pub trait IsGroupElement<
    CoordinateFrameId: IsCoordinateFrameId,
    Group: IsGroup + IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
>: IsManifoldElement<CoordinateFrameId, Group, Repr>
{
    fn group_mul(self, rhs: Self) -> Self {
        assert!(self.coordinate_system() == rhs.coordinate_system());
        todo!()
    }
    fn identity_at(coordinate_system: CoordinateSystem<CoordinateFrameId, Group, Repr>) -> Self;
    fn invert(self) -> Self;
}

impl<DstCoordinateFrameId, SrcCoordinateFrameId, Group, Repr>
    IsTransform<DstCoordinateFrameId, Group, Repr, SrcCoordinateFrameId, Group, Repr>
    for Transform<Repr, DstCoordinateFrameId, Group, Repr, SrcCoordinateFrameId, Group, Repr>
where
    DstCoordinateFrameId: IsCoordinateFrameId,
    SrcCoordinateFrameId: IsCoordinateFrameId,
    Group: IsGroup + IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
    ManifoldElement<DstCoordinateFrameId, Group, Repr>:
        IsGroupElement<DstCoordinateFrameId, Group, Repr>,
{
    fn dst(self) -> CoordinateSystem<DstCoordinateFrameId, Group, Repr> {
        self.dst
    }
    fn src(self) -> CoordinateSystem<SrcCoordinateFrameId, Group, Repr> {
        self.src
    }
    fn transform(
        self,
        point: ManifoldElement<SrcCoordinateFrameId, Group, Repr>,
    ) -> ManifoldElement<DstCoordinateFrameId, Group, Repr> {
        assert!(self.src() == point.coordinate_system());
        let src_in_dst_as_point = ManifoldElement::new(self.dst(), self.transform);
        let point_in_dst = ManifoldElement::new(self.dst(), point.coordinates);
        src_in_dst_as_point.group_mul(point_in_dst)
    }
}

impl<CoordinateFrameId, Group, Repr> IdentityTransform<Repr, CoordinateFrameId, Group, Repr, Repr>
    for Transform<Repr, CoordinateFrameId, Group, Repr, CoordinateFrameId, Group, Repr>
where
    CoordinateFrameId: IsCoordinateFrameId,
    Group: IsGroup + IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
    ManifoldElement<CoordinateFrameId, Group, Repr>: IsGroupElement<CoordinateFrameId, Group, Repr>,
{
    fn identity_at(
        coordinate_system: CoordinateSystem<CoordinateFrameId, Group, Repr>,
    ) -> Transform<Repr, CoordinateFrameId, Group, Repr, CoordinateFrameId, Group, Repr> {
        let identity_point = ManifoldElement::identity_at(coordinate_system);
        Transform::new(
            coordinate_system,
            coordinate_system,
            identity_point.coordinates,
        )
    }
}

impl<DstCoordinateFrameId, SrcCoordinateFrameId, Group, Repr>
    InvertableTransform<Repr, DstCoordinateFrameId, SrcCoordinateFrameId, Group, Repr>
    for Transform<Repr, DstCoordinateFrameId, Group, Repr, SrcCoordinateFrameId, Group, Repr>
where
    DstCoordinateFrameId: IsCoordinateFrameId,
    SrcCoordinateFrameId: IsCoordinateFrameId,
    Group: IsGroup + IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
    ManifoldElement<DstCoordinateFrameId, Group, Repr>:
        IsGroupElement<DstCoordinateFrameId, Group, Repr>,
{
    fn invert(
        self,
    ) -> Transform<Repr, SrcCoordinateFrameId, Group, Repr, DstCoordinateFrameId, Group, Repr> {
        let src_in_dst_as_point = ManifoldElement::new(self.dst(), self.transform);
        Transform::new(
            self.src(),
            self.dst(),
            src_in_dst_as_point.invert().coordinates,
        )
    }
}

impl<CoordinateFrameIdA, CoordinateFrameIdB, CoordinateFrameIdC, Group, Repr>
    ComposableTransform<
        Repr,
        CoordinateFrameIdA,
        Group,
        Repr,
        CoordinateFrameIdB,
        Group,
        Repr,
        CoordinateFrameIdC,
        Group,
        Repr,
    > for Transform<Repr, CoordinateFrameIdA, Group, Repr, CoordinateFrameIdB, Group, Repr>
where
    CoordinateFrameIdA: IsCoordinateFrameId,
    CoordinateFrameIdB: IsCoordinateFrameId,
    CoordinateFrameIdC: IsCoordinateFrameId,
    Repr: Debug + Copy + Serialize,
    Group: IsGroup + IsManifold + Differentiable,
    ManifoldElement<CoordinateFrameIdA, Group, Repr>:
        IsGroupElement<CoordinateFrameIdA, Group, Repr>,
    ManifoldElement<CoordinateFrameIdB, Group, Repr>:
        IsGroupElement<CoordinateFrameIdB, Group, Repr>,
    ManifoldElement<CoordinateFrameIdC, Group, Repr>:
        IsGroupElement<CoordinateFrameIdC, Group, Repr>,
{
    type TReprAC = Repr;

    fn compose_with(
        self,
        rhs: Transform<Repr, CoordinateFrameIdB, Group, Repr, CoordinateFrameIdC, Group, Repr>,
    ) -> Transform<Repr, CoordinateFrameIdA, Group, Repr, CoordinateFrameIdC, Group, Repr> {
        assert!(self.src() == rhs.dst());
        let b_in_a_as_pt = ManifoldElement::new(self.dst(), self.transform);
        let c_in_b_as_pt = ManifoldElement::new(self.dst(), rhs.transform);
        let c_in_a_as_pt = b_in_a_as_pt.group_mul(c_in_b_as_pt);
        Transform::new(self.dst(), rhs.src(), c_in_a_as_pt.coordinates)
    }
}

impl<T: Copy + RealField + Serialize, CoordinateFrameId: IsCoordinateFrameId>
    IsGroupElement<CoordinateFrameId, SO3, UnitQuaternion<T>>
    for ManifoldElement<CoordinateFrameId, SO3, UnitQuaternion<T>>
{
    fn group_mul(self, rhs: Self) -> Self {
        assert!(self.coordinate_system == rhs.coordinate_system);
        ManifoldElement::new(self.coordinate_system, self.coordinates * rhs.coordinates)
    }
    fn identity_at(
        coordinate_system: CoordinateSystem<CoordinateFrameId, SO3, UnitQuaternion<T>>,
    ) -> Self {
        ManifoldElement::new(coordinate_system, UnitQuaternion::identity())
    }
    fn invert(self) -> Self {
        ManifoldElement::new(self.coordinate_system, self.coordinates.inverse())
    }
}

impl<T: Copy + RealField + Serialize, CoordinateFrameId: IsCoordinateFrameId>
    IsGroupElement<CoordinateFrameId, SE3, Isometry3<T>>
    for ManifoldElement<CoordinateFrameId, SE3, Isometry3<T>>
{
    fn group_mul(self, rhs: Self) -> Self {
        assert!(self.coordinate_system == rhs.coordinate_system);
        ManifoldElement::new(self.coordinate_system, self.coordinates * rhs.coordinates)
    }
    fn identity_at(
        coordinate_system: CoordinateSystem<CoordinateFrameId, SE3, Isometry3<T>>,
    ) -> Self {
        ManifoldElement::new(coordinate_system, Isometry3::identity())
    }
    fn invert(self) -> Self {
        ManifoldElement::new(self.coordinate_system, self.coordinates.inverse())
    }
}

impl<T: Copy + RealField + Serialize, CoordinateFrameId: IsCoordinateFrameId>
    IsGroupElement<CoordinateFrameId, R3, Translation3<T>>
    for ManifoldElement<CoordinateFrameId, R3, Translation3<T>>
{
    fn group_mul(self, rhs: Self) -> Self {
        assert!(self.coordinate_system == rhs.coordinate_system);
        ManifoldElement::new(self.coordinate_system, self.coordinates * rhs.coordinates)
    }
    fn identity_at(
        coordinate_system: CoordinateSystem<CoordinateFrameId, R3, Translation3<T>>,
    ) -> Self {
        ManifoldElement::new(coordinate_system, Translation3::identity())
    }
    fn invert(self) -> Self {
        ManifoldElement::new(self.coordinate_system, self.coordinates.inverse())
    }
}
