use serde::Serialize;
use std::fmt::Debug;

use super::{
    CoordinateSystem, Differentiable, IsCoordinateFrameId, IsManifold, IsManifoldElement,
    ManifoldElement,
};

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Transform<
    TransformRepr,
    DstCoordinateFrameId,
    DstManifold,
    DstRepr,
    SrcCoordinateFrameId,
    SrcManifold,
    SrcRepr,
> where
    DstCoordinateFrameId: IsCoordinateFrameId,
    DstManifold: IsManifold + Differentiable,
    DstRepr: Debug + Copy + Serialize,
    SrcCoordinateFrameId: IsCoordinateFrameId,
    SrcManifold: IsManifold + Differentiable,
    SrcRepr: Debug + Copy + Serialize,
{
    pub dst: CoordinateSystem<DstCoordinateFrameId, DstManifold, DstRepr>,
    pub src: CoordinateSystem<SrcCoordinateFrameId, SrcManifold, SrcRepr>,
    pub transform: TransformRepr,
}

impl<
        TransformRepr,
        DstCoordinateFrameId,
        DstManifold,
        DstRepr,
        SrcCoordinateFrameId,
        SrcManifold,
        SrcRepr,
    >
    Transform<
        TransformRepr,
        DstCoordinateFrameId,
        DstManifold,
        DstRepr,
        SrcCoordinateFrameId,
        SrcManifold,
        SrcRepr,
    >
where
    DstCoordinateFrameId: IsCoordinateFrameId,
    DstManifold: IsManifold + Differentiable,
    DstRepr: Debug + Copy + Serialize,
    SrcCoordinateFrameId: IsCoordinateFrameId,
    SrcManifold: IsManifold + Differentiable,
    SrcRepr: Debug + Copy + Serialize,
{
    pub fn new(
        dst: CoordinateSystem<DstCoordinateFrameId, DstManifold, DstRepr>,
        src: CoordinateSystem<SrcCoordinateFrameId, SrcManifold, SrcRepr>,
        transform: TransformRepr,
    ) -> Self {
        Self {
            dst,
            src,
            transform,
        }
    }
}

pub trait IsTransform<
    DstCoordinateFrameId,
    DstManifold,
    DstRepr,
    SrcCoordinateFrameId,
    SrcManifold,
    SrcRepr,
>: Debug + Copy + Serialize where
    DstCoordinateFrameId: IsCoordinateFrameId,
    DstManifold: IsManifold + Differentiable,
    DstRepr: Debug + Copy + Serialize,
    SrcCoordinateFrameId: IsCoordinateFrameId,
    SrcManifold: IsManifold + Differentiable,
    SrcRepr: Debug + Copy + Serialize,
{
    fn dst(self) -> CoordinateSystem<DstCoordinateFrameId, DstManifold, DstRepr>;
    fn src(self) -> CoordinateSystem<SrcCoordinateFrameId, SrcManifold, SrcRepr>;
    fn transform(
        self,
        point: ManifoldElement<SrcCoordinateFrameId, SrcManifold, SrcRepr>,
    ) -> ManifoldElement<DstCoordinateFrameId, DstManifold, DstRepr> {
        assert!(self.src() == point.coordinate_system());
        todo!()
    }
}

pub trait IdentityTransform<TRepr, CoordinateFrameId, Manifold, DstRepr, SrcRepr>:
    Debug + Copy + Serialize
where
    TRepr: Debug + Copy + Serialize,
    CoordinateFrameId: IsCoordinateFrameId,
    Manifold: IsManifold + Differentiable,
    DstRepr: Debug + Copy + Serialize,
    SrcRepr: Debug + Copy + Serialize,
{
    fn identity_at(
        dst: CoordinateSystem<CoordinateFrameId, Manifold, DstRepr>,
    ) -> Transform<TRepr, CoordinateFrameId, Manifold, DstRepr, CoordinateFrameId, Manifold, SrcRepr>;
}

pub trait InvertableTransform<TRepr, DstCoordinateFrameId, SrcCoordinateFrameId, Manifold, Repr>:
    Debug
    + Copy
    + Serialize
    + IsTransform<DstCoordinateFrameId, Manifold, Repr, SrcCoordinateFrameId, Manifold, Repr>
where
    TRepr: Debug + Copy + Serialize,
    DstCoordinateFrameId: IsCoordinateFrameId,
    SrcCoordinateFrameId: IsCoordinateFrameId,
    Manifold: IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
{
    fn invert(
        self,
    ) -> Transform<TRepr, SrcCoordinateFrameId, Manifold, Repr, DstCoordinateFrameId, Manifold, Repr>;
}

pub trait ComposableTransform<
    TReprBC,
    CoordinateFrameIdA,
    ManifoldA,
    ReprA,
    CoordinateFrameIdB,
    ManifoldB,
    ReprB,
    CoordinateFrameIdC,
    ManifoldC,
    ReprC,
>:
    Debug
    + Copy
    + Serialize
    + IsTransform<CoordinateFrameIdA, ManifoldA, ReprA, CoordinateFrameIdB, ManifoldB, ReprB> where
    CoordinateFrameIdA: IsCoordinateFrameId,
    ManifoldA: IsManifold + Differentiable,
    ReprA: Debug + Copy + Serialize,
    CoordinateFrameIdB: IsCoordinateFrameId,
    ManifoldB: IsManifold + Differentiable,
    ReprB: Debug + Copy + Serialize,
    TReprBC: Debug + Copy + Serialize,
    CoordinateFrameIdC: IsCoordinateFrameId,
    ManifoldC: IsManifold + Differentiable,
    ReprC: Debug + Copy + Serialize,
{
    type TReprAC: Debug + Copy + Serialize;
    // type Output;

    fn compose_with(
        self,
        rhs: Transform<
            TReprBC,
            CoordinateFrameIdB,
            ManifoldB,
            ReprB,
            CoordinateFrameIdC,
            ManifoldC,
            ReprC,
        >,
    ) -> Transform<
        Self::TReprAC,
        CoordinateFrameIdA,
        ManifoldA,
        ReprA,
        CoordinateFrameIdC,
        ManifoldC,
        ReprC,
    >
    where
        Transform<
            TReprBC,
            CoordinateFrameIdB,
            ManifoldB,
            ReprB,
            CoordinateFrameIdC,
            ManifoldC,
            ReprC,
        >: IsTransform<CoordinateFrameIdB, ManifoldB, ReprB, CoordinateFrameIdC, ManifoldC, ReprC>,
    {
        assert!(self.src() == rhs.dst());
        todo!()
    }
}
