use core::f32::consts::PI;

use syact::prelude::*;
use sybot::prelude::*;

use crate::sya_mini::debug_robot;

mod sya_mini;

fn main() -> Result<(), sybot::Error> {
    // Logging
        env_logger::init();
        std::env::set_var("RUST_BACKTRACE", "1");
    // 

    log::info!("Logging enabled!");

    let mut rob = sya_mini::sya_mini_rob()?;
    let mut desc = sya_mini::SyaMiniDesc::new();
    let mut stat = sya_mini::SyaMiniStation::new();

    rob.setup()?;

    debug_robot(&rob);

    // Actions
    stat.home(&mut rob)?;

    // dbg!(desc.wobj());
    // dbg!(desc.kin().calculate_end());
    // desc.update(&mut rob, &[ Phi(PI / 2.0), Phi(PI / 2.0), Phi::ZERO, Phi::ZERO ])?;
    // dbg!(desc.wobj());
    // dbg!(desc.kin().calculate_end());
    
    Ok(())
}
