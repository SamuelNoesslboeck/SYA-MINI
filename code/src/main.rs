use syact::{prelude::*, tool::Tongs};
use sybot::prelude::*;

mod sya_mini;
use sya_mini::*;

fn main() -> Result<(), sybot::Error> {
    let sya_mini = SyaMiniRobot::new(
        [ AngConf::default(); 4],
        SyaMiniComps {
            base: Gear::new(
                Stepper::new(
                    GenericPWM::new(
                        0,
                        0
                    )?,
                    StepperConst::MOT_17HE15_1504S
                ),
                1.0 / 9.0
            ),
            arm1: Gear::new(
                Stepper::new(
                    GenericPWM::new(
                        0,
                        0
                    )?,
                    StepperConst::MOT_17HE15_1504S
                ),
                1.0 / 9.0
            ),
            arm2: Gear::new(
                Stepper::new(
                    GenericPWM::new(
                        0,
                        0
                    )?,
                    StepperConst::MOT_17HE15_1504S
                ),
                1.0 / 9.0
            ),
            arm3: Gear::new(
                Stepper::new(
                    GenericPWM::new(
                        0,
                        0
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

    

    Ok(())
}
