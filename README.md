# Type-Safe Coordinate Transforms

The goal is to provide a level of type-safety for Transforms between CoordinateSystems.
In robotics, one typically must keep track of many CoordinateSystems. This can lead to bugs:
for example, one might accidentally project a Point in the "LeftCamera" CoordinateSystem
into the image plane of the "RightCamera". This library adds both compile-time and
and run-time checks to prevent such bugs.

## Framework
Take the following motivating example:

On a quadcopter, one may have several Cameras. The number of Cameras is enumerable; and the relative Transform between
the Cameras remains fixed throughout the duration of the flight. If, however, one wishes to perform Visual Odometry,
then one must solve for the Transforms between different times. This is typically done by minimizing the re-projection error
of certain Landmark Points in the Image Planes of each of the Cameras.

This setting presents a number of challenges:
1. Two "types" of "CoordinateSystem"s: each Camera has a Coordinate System in 3-D Pose (SE3) Space, and in Image-Plane (R2) Space.
2. Three "types" of "Transform"s:
  1. Fixed rigid-body Transforms (e.g., between LeftCamera and RightCamera)
  2. Time-varying rigid-body Transforms (e.g., between LeftCamera(t=0) and LeftCamera(t=7), which is optimized during re-projection error minimization)
  3. Projective transforms (e.g., between a Landmark Point and its Image Pixel)

To address these issues, this library identifies a `CoordinateSystem` by an `id`, a `time`, and a "Representation" (`Repr`).
The `id` is **enumerable**, while the `time` is not. `Point`s in a given `CoordinateSystem` have coordinates in the `Repr` Representation.

For example, for the quadcopter, the `id` could be "LeftCameraImagePlane" or "RightCameraSE3". The `time` could be `0` or `7`. The `Repr` could be a 4x4 homogenous matrix for SE3 Poses or a Vector in R^2 for Image Pixels.

Then, a "Transform" has the characteristic (the `IsTransform` Trait) that it converts a `Point`s coordinates in one `CoordinateSystem` into its coordinates in a different `CoordinateSystem`. A "Static Transform" is a special kind of "Transform" that is fixed over time (for example, the transform from LeftCamera(t=0) to RightCamera(t=0) is the same as the transform from LeftCamera(t=7) to RightCamera(t=7)).

## Usage
An example, which showcases both the compile-time and run-time checks, is provided by the `test_stereo()` function in [src/lib.rs](src/lib.rs).

If you wish to add your own "CoordinateSystemId"s (for example, if you have an IMU or Rear Cameras), please do so in [src/coordinate_system_ids.rs](src/coordinate_system_ids.rs).