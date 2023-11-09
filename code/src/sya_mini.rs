use syact::prelude::*;
use sybot::prelude::*;

#[derive(SyncCompGroup)]
pub struct SyaMiniComps {
    pub base : Gear<Stepper>,
    pub arm1 : Gear<Stepper>,
    pub arm2 : Gear<Stepper>,
    pub arm3 : Gear<Stepper>
}

// pub type SYA_MINI = StepperRobot<