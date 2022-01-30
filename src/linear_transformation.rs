//! Booster NGFW linear-transformation routines
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

use miniconf::Miniconf;

/// A structure for mapping values between two different domains.
#[derive(Miniconf, serde::Serialize, serde::Deserialize, Copy, Clone, PartialEq)]
pub struct LinearTransformation {
    slope: f32,
    offset: f32,
}

impl LinearTransformation {
    /// Construct a new linear transformation.
    ///
    /// # Note
    /// A linear transformation is used for mapping values between two different domains.
    ///
    /// # Args
    /// * `slope` - The slope of the y = mx + b equation.
    /// * `offset` - The slope of the y-intercept. Equals the b portion of y = mx + b.
    pub fn new(slope: f32, offset: f32) -> Self {
        LinearTransformation { slope, offset }
    }

    /// Convert a value from the Y-domain into the X-domain.
    ///
    /// # Note
    /// This is accomplished by inverting the equation y=mx + b, such that y, m, and b are known.
    ///
    /// # Args
    /// * `vertical` - The Y-axis value to convert into the X-axis.
    pub fn invert(&self, vertical: f32) -> f32 {
        (vertical - self.offset) / self.slope
    }

    /// Map a value from the X-domain into the Y-domain using a linear equation.
    ///
    /// # Note
    /// This is accomplished by using the equation y=mx + b.
    ///
    /// # Args
    /// * `horizontal` - The X-axis value to into the Y-axis.
    pub fn map(&self, horizontal: f32) -> f32 {
        horizontal * self.slope + self.offset
    }
}
