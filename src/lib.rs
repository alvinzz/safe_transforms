mod coordinate_frames;
mod coordinate_system;
mod group;
mod lie;
mod manifold;
mod static_transform;
mod transform;

pub use coordinate_frames::*;
pub use coordinate_system::*;
pub use group::*;
pub use lie::*;
pub use manifold::*;
pub use static_transform::*;
pub use transform::*;

#[cfg(test)]
mod test {
    use super::*;
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn test() {
        let fl0 = CoordinateFrame::<CoordinateFrameA>::at_time(0);
        let fr1 = CoordinateFrame::<CoordinateFrameB>::at_time(1);
        let fl0_so3 = CoordinateSystem::at_frame(fl0);
        let fr1_so3 = CoordinateSystem::at_frame(fr1);
        let p_fl0 = ManifoldElement::new(
            fl0_so3,
            UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3)),
        );
        let fr1_from_fl0 = Transform::new(
            fr1_so3,
            fl0_so3,
            UnitQuaternion::from_scaled_axis(Vector3::new(0.3, 0.2, 0.1)),
        );

        let p_fr1 = fr1_from_fl0.transform(p_fl0);
        assert!(p_fr1.coordinate_system == fr1_so3);

        let p_fr1 = p_fr1.coordinates;
        let manual_p_fr1 = fr1_from_fl0.transform * p_fl0.coordinates;
        assert!((p_fr1 * manual_p_fr1.inverse()).angle() < 1e-4);

        let fr1_from_fr1 = Transform::identity_at(fr1_so3);
        let second_fr1_from_fl0 = fr1_from_fr1.compose_with(fr1_from_fl0);
        let fl0_from_fl0_cycle = second_fr1_from_fl0.invert().compose_with(fr1_from_fl0);
        assert!(fl0_from_fl0_cycle.dst() == fl0_so3);
        assert!(fl0_from_fl0_cycle.src() == fl0_so3);
        assert!(fl0_from_fl0_cycle.transform.angle() < 1e-4);

        let mut fl1 = fl0;
        fl1.time = 1;
        let fl1_so3 = CoordinateSystem::at_frame(fl1);
        let mut p_fl1 = p_fl0;
        p_fl1.coordinate_system = fl1_so3;

        let panic = std::panic::catch_unwind(|| fr1_from_fl0.transform(p_fl1));
        assert!(panic.is_err());

        // won't compile:
        // let err = fr1_from_fl0.compose(fr1_from_fr1);
    }
}
