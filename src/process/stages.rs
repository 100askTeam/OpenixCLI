//! Flash stages definition
//!
//! Provides stage definitions for FEL and FES mode flash operations

use super::global_progress::StageType;

/// Flash stages container
///
/// Defines the sequence of stages for different flash modes
pub struct FlashStages {
    stages: Vec<StageType>,
}

impl FlashStages {
    /// Create a new empty flash stages container
    pub fn new() -> Self {
        Self { stages: Vec::new() }
    }

    /// Create stages for FEL mode (USB boot)
    ///
    /// FEL mode requires additional stages for DRAM init and U-Boot download
    pub fn for_fel_mode() -> Self {
        let mut instance = Self::new();
        instance.stages = vec![
            StageType::Init,
            StageType::FelDram,
            StageType::FelUboot,
            StageType::FelReconnect,
            StageType::FesQuery,
            StageType::FesErase,
            StageType::FesMbr,
            StageType::FesPartitions,
            StageType::FesBoot,
            StageType::FesMode,
        ];
        instance
    }

    /// Create stages for FES mode (U-Boot)
    ///
    /// FES mode skips FEL-specific stages
    pub fn for_fes_mode() -> Self {
        let mut instance = Self::new();
        instance.stages = vec![
            StageType::Init,
            StageType::FesQuery,
            StageType::FesErase,
            StageType::FesMbr,
            StageType::FesPartitions,
            StageType::FesBoot,
            StageType::FesMode,
        ];
        instance
    }

    /// Create stages for a raw whole-disk write that starts in FEL mode and
    /// uses a vendor package only to boot the RAM FES service.
    pub fn for_raw_fel_mode() -> Self {
        let mut instance = Self::new();
        instance.stages = vec![
            StageType::Init,
            StageType::FelDram,
            StageType::FelUboot,
            StageType::FelReconnect,
            StageType::FesErase,
            StageType::FesQuery,
            StageType::FesMbr,
            StageType::FesPartitions,
            StageType::FesMode,
        ];
        instance
    }

    /// Create stages for a raw whole-disk write when FES is already running.
    pub fn for_raw_fes_mode() -> Self {
        let mut instance = Self::new();
        instance.stages = vec![
            StageType::Init,
            StageType::FesErase,
            StageType::FesQuery,
            StageType::FesMbr,
            StageType::FesPartitions,
            StageType::FesMode,
        ];
        instance
    }

    /// Get the stages as a slice
    pub fn stages(&self) -> &[StageType] {
        &self.stages
    }
}

impl Default for FlashStages {
    fn default() -> Self {
        Self::new()
    }
}
