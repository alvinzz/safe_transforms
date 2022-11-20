use nalgebra::{Isometry3, RealField, Translation3, UnitQuaternion, Vector3};
use serde::Serialize;
use std::fmt::Debug;

use super::{
    se3, so3, CoordinateSystem, IsCoordinateFrameId, IsGroupElement, IsLieAlgebra, IsLieGroup,
    IsManifoldElement, ManifoldElement, SE3, SO3,
};

#[derive(Debug, Clone, Copy, Serialize)]
pub struct LieAlgebraPoint<CoordinateFrameId, LieAlgebra, Repr, GroupRepr>
where
    CoordinateFrameId: IsCoordinateFrameId,
    LieAlgebra: IsLieAlgebra,
    Repr: Debug + Copy + Serialize,
    GroupRepr: Debug + Copy + Serialize,
    ManifoldElement<CoordinateFrameId, <LieAlgebra as IsLieAlgebra>::Group, GroupRepr>:
        IsGroupElement<CoordinateFrameId, <LieAlgebra as IsLieAlgebra>::Group, GroupRepr>,
{
    pub tangent_at:
        ManifoldElement<CoordinateFrameId, <LieAlgebra as IsLieAlgebra>::Group, GroupRepr>,
    pub coordinate_system: CoordinateSystem<CoordinateFrameId, LieAlgebra, Repr>,
    pub coordinates: Repr,
}

impl<CoordinateFrameId, LieAlgebra, Repr, GroupRepr>
    LieAlgebraPoint<CoordinateFrameId, LieAlgebra, Repr, GroupRepr>
where
    CoordinateFrameId: IsCoordinateFrameId,
    LieAlgebra: IsLieAlgebra,
    Repr: Debug + Copy + Serialize,
    GroupRepr: Debug + Copy + Serialize,
    ManifoldElement<CoordinateFrameId, <LieAlgebra as IsLieAlgebra>::Group, GroupRepr>:
        IsGroupElement<CoordinateFrameId, <LieAlgebra as IsLieAlgebra>::Group, GroupRepr>,
{
    fn new(
        tangent_at: ManifoldElement<
            CoordinateFrameId,
            <LieAlgebra as IsLieAlgebra>::Group,
            GroupRepr,
        >,
        coordinate_system: CoordinateSystem<CoordinateFrameId, LieAlgebra, Repr>,
        coordinates: Repr,
    ) -> Self {
        assert!(coordinate_system.frame == tangent_at.coordinate_system().frame);
        Self {
            tangent_at,
            coordinate_system,
            coordinates,
        }
    }
}

pub trait IsLieAlgebraPoint<T, CoordinateFrameId, LieAlgebra, AlgebraRepr, GroupRepr>
where
    CoordinateFrameId: IsCoordinateFrameId,
    LieAlgebra: IsLieAlgebra,
    AlgebraRepr: Debug + Copy + Serialize,
    GroupRepr: Debug + Copy + Serialize,
    ManifoldElement<CoordinateFrameId, <LieAlgebra as IsLieAlgebra>::Group, GroupRepr>:
        IsGroupElement<CoordinateFrameId, <LieAlgebra as IsLieAlgebra>::Group, GroupRepr>,
{
    fn scale_by(self, scalar: T) -> Self;
}

pub trait IsLieGroupPoint<T, CoordinateFrameId, LieGroup, GroupRepr, AlgebraRepr>:
    IsGroupElement<CoordinateFrameId, LieGroup, GroupRepr>
where
    CoordinateFrameId: IsCoordinateFrameId,
    LieGroup: IsLieGroup,
    GroupRepr: Debug + Copy + Serialize,
    AlgebraRepr: Debug + Copy + Serialize,
    ManifoldElement<CoordinateFrameId, LieGroup, GroupRepr>:
        IsGroupElement<CoordinateFrameId, LieGroup, GroupRepr>,
    LieAlgebraPoint<CoordinateFrameId, <LieGroup as IsLieGroup>::Algebra, AlgebraRepr, GroupRepr>:
        IsLieAlgebraPoint<
            T,
            CoordinateFrameId,
            <LieGroup as IsLieGroup>::Algebra,
            AlgebraRepr,
            GroupRepr,
        >,
    Self: From<
        LieAlgebraPoint<
            CoordinateFrameId,
            <LieGroup as IsLieGroup>::Algebra,
            AlgebraRepr,
            GroupRepr,
        >,
    >,
{
    fn log_of(
        self,
        other: Self,
    ) -> LieAlgebraPoint<CoordinateFrameId, <LieGroup as IsLieGroup>::Algebra, AlgebraRepr, GroupRepr>
    {
        assert!(self.coordinate_system() == other.coordinate_system());
        todo!()
    }

    fn lerp_to(self, other: Self, alpha: T) -> Self {
        let other_as_lie_algebra_point = self.log_of(other);
        let other_scaled = other_as_lie_algebra_point.scale_by(alpha);
        Self::from(other_scaled)
    }
}

impl<T, CoordinateFrameId>
    IsLieAlgebraPoint<T, CoordinateFrameId, so3, Vector3<T>, UnitQuaternion<T>>
    for LieAlgebraPoint<CoordinateFrameId, so3, Vector3<T>, UnitQuaternion<T>>
where
    T: Copy + RealField + Serialize,
    CoordinateFrameId: IsCoordinateFrameId,
{
    fn scale_by(self, scalar: T) -> Self {
        Self::new(
            self.tangent_at,
            self.coordinate_system,
            self.coordinates * scalar,
        )
    }
}

impl<T, CoordinateFrameId>
    From<LieAlgebraPoint<CoordinateFrameId, so3, Vector3<T>, UnitQuaternion<T>>>
    for ManifoldElement<CoordinateFrameId, SO3, UnitQuaternion<T>>
where
    T: Copy + RealField + Serialize,
    CoordinateFrameId: IsCoordinateFrameId,
{
    fn from(
        algebra_point: LieAlgebraPoint<CoordinateFrameId, so3, Vector3<T>, UnitQuaternion<T>>,
    ) -> Self {
        let base = algebra_point.tangent_at;
        let point = Self {
            coordinate_system: base.coordinate_system(),
            coordinates: UnitQuaternion::from_scaled_axis(algebra_point.coordinates),
        };
        base.group_mul(point)
    }
}

impl<T, CoordinateFrameId> IsLieGroupPoint<T, CoordinateFrameId, SO3, UnitQuaternion<T>, Vector3<T>>
    for ManifoldElement<CoordinateFrameId, SO3, UnitQuaternion<T>>
where
    T: Copy + RealField + Serialize,
    CoordinateFrameId: IsCoordinateFrameId,
{
    fn log_of(
        self,
        other: Self,
    ) -> LieAlgebraPoint<CoordinateFrameId, so3, Vector3<T>, UnitQuaternion<T>> {
        assert!(self.coordinate_system() == other.coordinate_system());
        LieAlgebraPoint::new(
            self,
            CoordinateSystem::at_time(self.coordinate_system().frame.time),
            self.invert().group_mul(other).coordinates.scaled_axis(),
        )
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Twist<T>
where
    T: Copy + RealField + Serialize,
{
    pub w: Vector3<T>,
    pub v: Vector3<T>,
}

impl<T, CoordinateFrameId> IsLieAlgebraPoint<T, CoordinateFrameId, se3, Twist<T>, Isometry3<T>>
    for LieAlgebraPoint<CoordinateFrameId, se3, Twist<T>, Isometry3<T>>
where
    T: Copy + RealField + Serialize,
    CoordinateFrameId: IsCoordinateFrameId,
{
    fn scale_by(self, scalar: T) -> Self {
        Self::new(
            self.tangent_at,
            self.coordinate_system,
            Twist {
                w: self.coordinates.w * scalar,
                v: self.coordinates.v * scalar,
            },
        )
    }
}

impl<CoordinateFrameId> From<LieAlgebraPoint<CoordinateFrameId, se3, Twist<f32>, Isometry3<f32>>>
    for ManifoldElement<CoordinateFrameId, SE3, Isometry3<f32>>
where
    CoordinateFrameId: IsCoordinateFrameId,
{
    #[allow(non_snake_case)]
    fn from(
        algebra_point: LieAlgebraPoint<CoordinateFrameId, se3, Twist<f32>, Isometry3<f32>>,
    ) -> Self {
        let base = algebra_point.tangent_at;

        let w = algebra_point.coordinates.w;
        let rotation = UnitQuaternion::from_scaled_axis(w);
        let theta = w.norm();

        let t = if theta < 1e-6 {
            algebra_point.coordinates.v
        } else {
            let v = algebra_point.coordinates.v;
            let w_normed = w / theta;
            let v_proj_w = v.dot(&w_normed) * w_normed;
            let v_perp_w = v - v_proj_w;
            let t_prime = v_perp_w.cross(&w_normed) / theta;
            (rotation * t_prime - t_prime) + v_proj_w
        };
        let point = Self {
            coordinate_system: base.coordinate_system(),
            coordinates: Isometry3::from_parts(Translation3::from(t), rotation),
        };

        base.group_mul(point)
    }
}

impl<CoordinateFrameId> IsLieGroupPoint<f32, CoordinateFrameId, SE3, Isometry3<f32>, Twist<f32>>
    for ManifoldElement<CoordinateFrameId, SE3, Isometry3<f32>>
where
    CoordinateFrameId: IsCoordinateFrameId,
{
    #[allow(non_snake_case)]
    fn log_of(
        self,
        other: Self,
    ) -> LieAlgebraPoint<CoordinateFrameId, se3, Twist<f32>, Isometry3<f32>> {
        assert!(self.coordinate_system() == other.coordinate_system());
        let isometry = self.invert().group_mul(other).coordinates;
        let w = isometry.rotation.scaled_axis();
        let theta = isometry.rotation.angle();
        let v = if theta < 1e-6 {
            isometry.translation.vector
        } else {
            let t = isometry.translation.vector;
            let w_normed = w / theta;
            let t_proj_w = t.dot(&w_normed) * w_normed;
            let t_perp_w = t - t_proj_w;
            let t_prime_plus_t_perp_w_over_2 = t_perp_w.cross(&w_normed) / 2. / (theta / 2.).tan();
            let t_prime = t_prime_plus_t_perp_w_over_2 - t_perp_w / 2.;
            w.cross(&t_prime) + t_proj_w
        };
        LieAlgebraPoint::new(
            self,
            CoordinateSystem::at_time(self.coordinate_system().frame.time),
            Twist { w, v },
        )
    }
}

#[cfg(test)]
mod test {
    use crate::{
        CoordinateFrame, CoordinateFrameA, CoordinateSystem, IsGroupElement, IsLieGroupPoint,
        ManifoldElement, SE3,
    };
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    #[test]
    fn test_lerp() {
        let fl0 = CoordinateFrame::<CoordinateFrameA>::at_time(0);
        let fl0_so3 = CoordinateSystem::at_frame(fl0);
        let p = ManifoldElement::new(
            fl0_so3,
            Isometry3::from_parts(
                Translation3::new(0.5f32, 0.6, 0.7),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3)),
            ),
        );
        let q = ManifoldElement::new(
            fl0_so3,
            Isometry3::from_parts(
                Translation3::new(0.8, 0.9, 1.0),
                UnitQuaternion::from_scaled_axis(Vector3::new(1.1, 1.2, 1.3)),
            ),
        );

        let angle_p_q = p.invert().group_mul(q).coordinates.rotation.angle();

        let quarter_lerp = p.lerp_to(q, 0.25);
        let quarter_angle = p
            .invert()
            .group_mul(quarter_lerp)
            .coordinates
            .rotation
            .angle();
        assert!((quarter_angle - angle_p_q * 0.25).abs() < 1e-4);

        let half_lerp = p.lerp_to(q, 0.5);
        let half_angle = p.invert().group_mul(half_lerp).coordinates.rotation.angle();
        assert!((half_angle - angle_p_q * 0.5).abs() < 1e-4);

        let delta_quarter_p = p.invert().group_mul(quarter_lerp);
        let delta_half_p = p.invert().group_mul(half_lerp);
        let lerp_q = p
            .group_mul(delta_quarter_p)
            .group_mul(delta_half_p)
            .group_mul(delta_quarter_p);
        assert!(q.invert().group_mul(lerp_q).coordinates.rotation.angle() < 1e-4);
        assert!(
            q.invert()
                .group_mul(lerp_q)
                .coordinates
                .translation
                .vector
                .norm()
                < 1e-4
        );
    }

    #[test]
    fn test_circ() {
        let eps: f32 = 1e-6;

        let fl0 = CoordinateSystem::<CoordinateFrameA, SE3, Isometry3<f32>>::at_time(0);

        let p = ManifoldElement::new(
            fl0,
            Isometry3::from_parts(
                Translation3::new(eps.cos(), eps.sin(), 0.),
                UnitQuaternion::from_scaled_axis(Vector3::new(0., 0., eps)),
            ),
        );

        let pi_minus_eps = std::f32::consts::PI - eps;
        let q = ManifoldElement::new(
            fl0,
            Isometry3::from_parts(
                Translation3::new(pi_minus_eps.cos(), pi_minus_eps.sin(), 1.),
                UnitQuaternion::from_scaled_axis(Vector3::new(0., 0., pi_minus_eps)),
            ),
        );

        let half = p.lerp_to(q, 0.5);

        assert!((half.coordinates.translation.vector - Vector3::new(0., 1., 0.5)).norm() < eps);
        assert!(
            (half.coordinates.rotation.inverse()
                * UnitQuaternion::from_scaled_axis(Vector3::new(
                    0.,
                    0.,
                    std::f32::consts::FRAC_PI_2
                )))
            .angle()
                < eps
        );
    }
}
