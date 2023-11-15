use core::f32::consts::PI;

use syact::prelude::*;
use sybot::prelude::*;

use syact::meas::SimpleMeasResult;
use syact::tool::Tongs;

// Constants
    const STEP_PINS : [u8; 4] = [ 14, 15, 18, 23 ];
    const DIR_PINS : [u8; 4] = [ 24, 25, 8, 7 ];
    const MEAS_PINS : [u8; 4] = [ 4, 17, 27, 22 ];

    const BASE_FORCES : [Force; 4] = [ Force(0.0), Force(3.0), Force(2.0), Force(1.0) ];
    const BASE_INERTIAS : [Inertia; 4] = [ Inertia(0.5), Inertia(0.25), Inertia(0.1), Inertia(0.1) ];

    pub const MEAS_DATA : [SimpleMeasData; 4] = [
        SimpleMeasData {
            set_gamma: Gamma::ZERO,
            max_dist: Delta(9.0 * 2.0 * PI),
            meas_speed_f: 0.2,
            add_samples: 0,

            sample_dist: Some(Delta(1.5 * PI))
        },
        SimpleMeasData {
            set_gamma: Gamma::ZERO,
            max_dist: Delta(9.0 * 2.0 * PI),
            meas_speed_f: 0.2,
            add_samples: 0,

            sample_dist: Some(Delta(1.5 * PI))
        },
        SimpleMeasData {
            set_gamma: Gamma::ZERO,
            max_dist: Delta(9.0 * 2.0 * PI),
            meas_speed_f: 0.2,
            add_samples: 0,

            sample_dist: Some(Delta(1.5 * PI))
        },
        SimpleMeasData {
            set_gamma: Gamma::ZERO,
            max_dist: Delta(9.0 * 2.0 * PI),
            meas_speed_f: 0.2,
            add_samples: 0,

            sample_dist: Some(Delta(1.5 * PI))
        }
    ];
// 

// Robot
    #[derive(StepperCompGroup)]
    pub struct SyaMiniComps {
        pub base : Gear<Stepper>,
        pub arm1 : Gear<Stepper>,
        pub arm2 : Gear<Stepper>,
        pub arm3 : Gear<Stepper>
    }

    pub type SyaMiniRobot = StepperRobot<SyaMiniComps, dyn StepperComp, 4>;

    pub fn sya_mini_rob() -> Result<SyaMiniRobot, syact::Error> {
        let mut rob = SyaMiniRobot::new(
            [ AngConf::default(); 4],
            SyaMiniComps {
                base: Gear::new(
                    Stepper::new(
                        GenericPWM::new(
                            STEP_PINS[0],
                            DIR_PINS[0]
                        )?,
                        StepperConst::MOT_17HE15_1504S
                    ),
                    1.0 / 9.0
                ),
                arm1: Gear::new(
                    Stepper::new(
                        GenericPWM::new(
                            STEP_PINS[1],
                            DIR_PINS[1]
                        )?,
                        StepperConst::MOT_17HE15_1504S
                    ),
                    1.0 / 9.0
                ),
                arm2: Gear::new(
                    Stepper::new(
                        GenericPWM::new(
                            STEP_PINS[2],
                            DIR_PINS[2]
                        )?,
                        StepperConst::MOT_17HE15_1504S
                    ),
                    1.0 / 9.0
                ),
                arm3: Gear::new(
                    Stepper::new(
                        GenericPWM::new(
                            STEP_PINS[3],
                            DIR_PINS[3]
                        )?,
                        StepperConst::MOT_17HE15_1504S
                    ),
                    1.0 / 9.0
                )
            },
            vec![
                Box::new(Tongs::new(
                    Servo::new(ServoConst::MG996R, 0),
                    0.4, 
                    0.8,
                    0.0,
                    Inertia(0.1)
                ))
            ]
        );

        rob.comps_mut().try_for_each_mut(|c, i| -> Result<(), syact::Error> {
            let mut meas = EndSwitch::new(true, None, UniInPin::new(MEAS_PINS[i]));
            meas.setup()?;
            c.add_interruptor(Box::new(meas));
            Ok(())
        })?;

        rob.apply_forces(&BASE_FORCES)?;
        rob.apply_inertias(&BASE_INERTIAS);

        Ok(rob)
    }
// 

// Descriptor
    pub struct SyaMiniDesc { }

    // impl Descriptor<SyaMiniComps, Gear<Stepper>, 4> for SyaMiniDesc {
        
    // }
// 

// System
    #[derive(Debug)]
    pub struct SyaMiniStation {
        meas_res : [SimpleMeasResult; 4]
    }

    impl SyaMiniStation {
        #[allow(invalid_value)]
        pub fn new() -> Self {
            Self {
                meas_res: unsafe { core::mem::zeroed() }
            }
        }
    }

    impl Station<SyaMiniComps, dyn StepperComp, 4> for SyaMiniStation {
        fn home(&mut self, rob : &mut impl Robot<SyaMiniComps, dyn StepperComp, 4>) -> Result<(), sybot::Error> {
            self.meas_res = rob.comps_mut().try_for_each_mut(|c, i| -> Result<SimpleMeasResult, syact::Error> { 
                syact::meas::take_simple_meas(c, &MEAS_DATA[i], 1.0)
            })?; 

            Ok(())
        }
    }
// 
