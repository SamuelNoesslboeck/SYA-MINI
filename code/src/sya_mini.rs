use syact::prelude::*;
use sybot::prelude::*;

// Robot
    #[derive(StepperCompGroup)]
    pub struct SyaMiniComps {
        pub base : Gear<Stepper>,
        pub arm1 : Gear<Stepper>,
        pub arm2 : Gear<Stepper>,
        pub arm3 : Gear<Stepper>
    }

    pub type SyaMiniRobot = StepperRobot<SyaMiniComps, Gear<Stepper>, 4>;
// 

// Descriptor
    pub struct SyaMiniDesc { }

    impl Descriptor<SyaMiniComps, Gear<Stepper>, 4> for SyaMiniDesc {
        
    }
// 

// System
    pub struct SyaMiniSystem {

    }
// 