use serde::{Deserialize, Serialize};
use std::{fmt::Debug, hash::Hash};

pub trait IsManifold: Debug + Default + Copy + Eq + Hash + Serialize {}

pub trait Differentiable: Debug + Default + Copy + Eq + Hash + Serialize {}

pub trait IsGroup: Debug + Default + Copy + Eq + Hash + Serialize {}

pub trait IsVectorSpace: IsGroup + IsManifold + Differentiable {}

pub trait IsLieGroup: IsGroup + IsManifold + Differentiable
where
    Self::Algebra: IsLieAlgebra<Group = Self>,
{
    type Algebra: IsLieAlgebra;
}

pub trait IsLieAlgebra: IsVectorSpace
where
    Self::Group: IsLieGroup<Algebra = Self>,
{
    type Group: IsLieGroup;
}

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct RP2 {}
impl IsManifold for RP2 {}
impl Differentiable for RP2 {}

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct SE3 {}
impl IsManifold for SE3 {}
impl Differentiable for SE3 {}
impl IsGroup for SE3 {}
impl IsLieGroup for SE3 {
    type Algebra = se3;
}

#[allow(non_camel_case_types)]
#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct se3 {}
impl IsManifold for se3 {}
impl Differentiable for se3 {}
impl IsGroup for se3 {}
impl IsVectorSpace for se3 {}
impl IsLieAlgebra for se3 {
    type Group = SE3;
}

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct SO3 {}
impl IsManifold for SO3 {}
impl Differentiable for SO3 {}
impl IsGroup for SO3 {}
impl IsLieGroup for SO3 {
    type Algebra = so3;
}

#[allow(non_camel_case_types)]
#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct so3 {}
impl IsManifold for so3 {}
impl Differentiable for so3 {}
impl IsGroup for so3 {}
impl IsVectorSpace for so3 {}
impl IsLieAlgebra for so3 {
    type Group = SO3;
}

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct R3 {}
impl IsManifold for R3 {}
impl Differentiable for R3 {}
impl IsGroup for R3 {}
impl IsVectorSpace for R3 {}
impl IsLieGroup for R3 {
    type Algebra = R3;
}
impl IsLieAlgebra for R3 {
    type Group = R3;
}

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct SE2 {}
impl IsManifold for SE2 {}
impl Differentiable for SE2 {}

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct SO2 {}
impl IsManifold for SO2 {}
impl Differentiable for SO2 {}

#[derive(Debug, Default, Clone, Copy, Hash, PartialEq, Eq, Serialize, Deserialize)]
pub struct R2 {}
impl IsManifold for R2 {}
impl Differentiable for R2 {}
