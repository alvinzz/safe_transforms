/// Define Coordinate Frame IDs here.
use serde::Serialize;

use crate::IsCoordinateSystemId;

#[macro_export]
macro_rules! define_coordinate_system_id {
    ($id:ident) => {
        #[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
        pub struct $id {}
        impl IsCoordinateSystemId for $id {}
    };
}

define_coordinate_system_id!(LeftCameraSE3);
define_coordinate_system_id!(LeftCameraImage);
define_coordinate_system_id!(RightCameraSE3);
define_coordinate_system_id!(RightCameraImage);
