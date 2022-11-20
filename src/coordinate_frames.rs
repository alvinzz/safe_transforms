use serde::Serialize;

use crate::IsCoordinateFrameId;

#[macro_export]
macro_rules! define_coordinate_frame_id {
    ($id:ident) => {
        #[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
        pub struct $id {}
        impl IsCoordinateFrameId for $id {}
    };
}

define_coordinate_frame_id!(CoordinateFrameA);
define_coordinate_frame_id!(CoordinateFrameB);
define_coordinate_frame_id!(CoordinateFrameC);
