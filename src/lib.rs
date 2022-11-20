mod coordinate_system;
mod coordinate_system_ids;
mod static_transform;
mod transform;

pub use coordinate_system::*;
pub use coordinate_system_ids::*;
pub use static_transform::*;
pub use transform::*;

#[cfg(test)]
mod test {
    use super::*;
    use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion, Vector2, Vector3};

    const BASELINE: f32 = 0.1;
    const LEFT_FOCAL_LEN: f32 = 100.;
    const RIGHT_FOCAL_LEN: f32 = 50.;
    const POINT_DISTANCE: f32 = 0.5;

    const ATOL: f32 = 1e-6;

    /// In this scenario, we have a stereo pair of cameras, which are attached
    /// to a rig so that their relative poses are fixed.
    ///
    /// We are going to track the coordinates of a point as the rig moves over time.
    #[test]
    fn test_stereo() {
        // Set Left-Right Extrinsics.
        let se3_left_from_right =
            StaticSE3Transform::<LeftCameraSE3, RightCameraSE3, _>::new(Isometry3::from_parts(
                Translation3::new(BASELINE, 0., 0.),
                UnitQuaternion::default(),
            ));
        // Set Camera Intrinsics.
        #[rustfmt::skip]
        let left_intrinsics =
            StaticProjectiveTransform::<LeftCameraImage, LeftCameraSE3, _>::new(Matrix3::new(
                LEFT_FOCAL_LEN, 0f32, 0f32,
                0f32, LEFT_FOCAL_LEN, 0f32,
                0f32, 0f32, 1f32,
            ));
        #[rustfmt::skip]
        let right_intrinsics =
            StaticProjectiveTransform::<RightCameraImage, RightCameraSE3, _>::new(Matrix3::new(
                RIGHT_FOCAL_LEN, 0f32, 0f32,
                0f32, RIGHT_FOCAL_LEN, 0f32,
                0f32, 0f32, 1f32,
            ));

        // These are our Coordinate Systems at Time = 0.
        //
        // Note that there are different ones for the Left and Right Cameras, as well as for 3-D Pose
        // and for the Image Planes.
        let left_se3_at_0 = CoordinateSystem::<LeftCameraSE3, Isometry3<f32>>::at_time(0);
        let right_se3_at_0 = CoordinateSystem::<RightCameraSE3, Isometry3<f32>>::at_time(0);
        let left_image_at_0 = CoordinateSystem::<LeftCameraImage, Vector2<f32>>::at_time(0);
        let right_image_at_0 = CoordinateSystem::<RightCameraImage, Vector2<f32>>::at_time(0);

        // Let's say that a Point is at (0, 0, POINT_DISTANCE) in the RightCameraSE3 CoordinateSystem at time 0.
        //
        // We are going to track its position in 3-D space relative both the Left and Right Cameras,
        // as well as its projection into both the Left and Right Camera Images.
        let point_in_right_se3_at_0 = Point::new(
            right_se3_at_0,
            Isometry3::from_parts(
                Translation3::new(0., 0., POINT_DISTANCE),
                UnitQuaternion::default(),
            ),
        );

        // What are the Point's coordinates in the LeftCameraSE3 CoordinateSystem at time 0?
        let point_in_left_se3_at_0 = se3_left_from_right
            .at_time(0)
            .transform(point_in_right_se3_at_0);
        assert!(point_in_left_se3_at_0.coordinate_system() == left_se3_at_0);
        assert!(
            (point_in_left_se3_at_0.coordinates().translation.vector
                - Vector3::new(BASELINE, 0., POINT_DISTANCE))
            .norm()
                < ATOL
        );

        // How about projecting into an image? What are the Point's coordinates in the RightCameraImage CoordinateSystem at time 0?
        let point_in_right_image_at_0 = right_intrinsics
            .at_time(0)
            .transform(point_in_right_se3_at_0);
        assert!(point_in_right_image_at_0.coordinate_system() == right_image_at_0);

        // How about the Left Camera? What are the Point's coordinates in the LeftCameraImage Coordinate System at time 0?
        let point_in_left_image_at_0 = left_intrinsics.at_time(0).transform(point_in_left_se3_at_0);
        assert!(point_in_left_image_at_0.coordinate_system() == left_image_at_0);
        assert!(
            (point_in_left_image_at_0.coordinates()
                - Vector2::new(BASELINE * LEFT_FOCAL_LEN / POINT_DISTANCE, 0.))
            .norm()
                < ATOL
        );

        // NOTE THAT:
        // ```
        // let point_in_left_image_at_0 = right_intrinsics
        //     .at_time(0)
        //     .transform(point_in_left_se3_at_0);
        // ```
        // WILL NOT COMPILE!!!
        // This is because the `right_intrinsics` Transform is applied to the RightCameraSE3
        // CoordinateSystem, but the Point is in the LeftCameraSE3 CoordinateSystem.
        //
        // This is what enumerating the possible "CoordinateSystemId"s in src/coordinate_system_ids.rs
        // allows us to do.

        // Now, suppose that at Time = 1, the Camera Rig has moved.
        // These are the Coordinate Systems at Time = 1.
        let left_se3_at_1 = CoordinateSystem::<LeftCameraSE3, Isometry3<f32>>::at_time(1);
        let right_se3_at_1 = CoordinateSystem::<RightCameraSE3, Isometry3<f32>>::at_time(1);
        let _left_image_at_1 = CoordinateSystem::<LeftCameraImage, Vector2<f32>>::at_time(1);
        let _right_image_at_1 = CoordinateSystem::<RightCameraImage, Vector2<f32>>::at_time(1);

        // Suppose that we've tracked the movement of the Left Camera.
        let se3_left_1_from_left_0 = SE3Transform::new(
            left_se3_at_1,
            left_se3_at_0,
            Isometry3::from_parts(
                Translation3::new(0.1, 0.2, 0.3),
                UnitQuaternion::from_scaled_axis(Vector3::new(0.1, 0.2, 0.3)),
            ),
        );

        // Then, what is the movement of the Right Camera?
        let se3_right_from_left = se3_left_from_right.invert();
        let se3_right_1_from_right_0 = (se3_right_from_left.at_time(1))
            .compose_with(se3_left_1_from_left_0)
            .compose_with(se3_left_from_right.at_time(0));

        // What are the coordinates of the Point in the RightCameraSE3 Coordinate System?
        // We can compute this in two ways, and confirm that the results are equivalent.
        let point_in_right_se3_at_1 = se3_right_1_from_right_0.transform(point_in_right_se3_at_0);
        assert!(point_in_right_se3_at_1.coordinate_system() == right_se3_at_1);

        let alt_point_in_right_se3_at_1 = (se3_right_from_left.at_time(1))
            .transform(se3_left_1_from_left_0.transform(point_in_left_se3_at_0));
        assert!(alt_point_in_right_se3_at_1.coordinate_system() == right_se3_at_1);

        assert!(
            (point_in_right_se3_at_1.coordinates().to_homogeneous()
                - alt_point_in_right_se3_at_1.coordinates().to_homogeneous())
            .norm()
                < ATOL
        );

        // NOTE THAT ATTEMPTING:
        // ```
        // se3_right_1_from_right_0.transform(point_in_right_se3_at_1)
        // ```
        // WILL PANIC (AT RUNTIME)!
        // It cannot be caught at compile-time, because the times of CoordinateSystems
        // are not enumerable. But, it will `panic!` at run-time, because the times
        // do not match up.
        let panic = std::panic::catch_unwind(|| {
            se3_right_1_from_right_0.transform(point_in_right_se3_at_1)
        });
        assert!(panic.is_err());
    }
}
