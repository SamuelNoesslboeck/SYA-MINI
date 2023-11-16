use core::f32::consts::PI;

use syact::prelude::*;
use sybot::conf::AxisConf;
use sybot::prelude::*;

use syact::meas::SimpleMeasResult;
use syact::tool::Tongs;

// Constants
    const STEP_PINS : [u8; 4] = [ 14, 15, 18, 23 ];
    const DIR_PINS : [u8; 4] = [ 24, 25, 8, 7 ];
    const MEAS_PINS : [u8; 4] = [ 4, 17, 27, 22 ];

    const BASE_FORCES : [Force; 4] = [ Force(0.0), Force(3.0), Force(2.0), Force(1.0) ];
    const BASE_INERTIAS : [Inertia; 4] = [ Inertia(0.5), Inertia(0.25), Inertia(0.1), Inertia(0.1) ];

    const COMP_DATA : CompData = CompData {
        u: 24.0,
        s_f: 1.5
    };

    pub const MEAS_DATA : [SimpleMeasData; 4] = [
        SimpleMeasData {
            set_gamma: Gamma::ZERO,
            max_dist: Delta(2.0 * PI),
            meas_speed_f: 0.75,
            add_samples: 0,

            sample_dist: Some(Delta(0.25 * PI))
        },
        SimpleMeasData {
            set_gamma: Gamma::ZERO,
            max_dist: Delta(2.0 * PI),
            meas_speed_f: 0.75,
            add_samples: 0,

            sample_dist: Some(Delta(0.25 * PI))
        },
        SimpleMeasData {
            set_gamma: Gamma::ZERO,
            max_dist: Delta(2.0 * PI),
            meas_speed_f: 0.75,
            add_samples: 0,

            sample_dist: Some(Delta(0.25 * PI))
        },
        SimpleMeasData {
            set_gamma: Gamma::ZERO,
            max_dist: Delta(2.0 * PI),
            meas_speed_f: 0.75,
            add_samples: 0,

            sample_dist: Some(Delta(0.25 * PI))
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
            if i == 0 {
                return Ok(())
            }

            let mut meas = EndSwitch::new(false, None, UniInPin::new(MEAS_PINS[i]));
            meas.setup()?;
            c.add_interruptor(Box::new(meas));
            Ok(())
        })?;

        rob.comps_mut().write_data(COMP_DATA);

        rob.apply_forces(&BASE_FORCES)?;
        rob.apply_inertias(&BASE_INERTIAS);

        Ok(rob)
    }
// 

// Descriptor
    pub struct SyaAxisConf {
        pub final_angle : Phi
    }

    impl AxisConf for SyaAxisConf {
        fn configure(&mut self, phis : Vec<Phi>) -> Result<(), sybot::Error> {
            self.final_angle = *phis.first().ok_or("Not enough phis given for the AxisConf!".into())?;
            Ok(())
        }
    }

    pub struct SyaMiniDesc {
        _aconf : SyaAxisConf
    }

    impl Descriptor<SyaMiniComps, dyn StepperComp, 4> for SyaMiniDesc {
        // Axis conf
            fn aconf(&self) -> &dyn AxisConf {
                &self._aconf
            }

            fn aconf_mut(&mut self) -> &mut dyn AxisConf {
                &mut self._aconf
            }
        //
    
        // Events
            fn update(&mut self, rob : &mut dyn Robot<G, T, C>, phis : &[Phi; C]) -> Result<(), crate::Error>;
        // 
    
        // Calculation
            fn convert_pos(&self, rob : &dyn Robot<G, T, C>, pos : Position) -> Result<[Phi; C], crate::Error>;
        //
    
        // World object
            fn wobj<'a>(&'a self) -> &'a WorldObj;
    
            fn wobj_mut<'a>(&'a mut self) -> &'a mut WorldObj;
    
            fn current_tcp(&self) -> &PointRef;
    
            /// Create a Vec3 from optional coordinates 
            fn cache_tcp(&self, x_opt : Option<f32>, y_opt : Option<f32>, z_opt : Option<f32>) -> Vec3;
        // 
    }
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
            log::trace!("Starting measurement ... ");

            self.meas_res = rob.comps_mut().try_for_each_mut(|c, i| -> Result<SimpleMeasResult, syact::Error> { 
                if i == 0 {
                    return Ok(SimpleMeasResult::default());
                }
            
                let res = syact::meas::take_simple_meas(c, &MEAS_DATA[i], 1.0);
                log::trace!("- Measurement for component {} done!", i);
                res
            })?; 

            log::trace!("Measurement done!");

            Ok(())
        }
    }
// 
