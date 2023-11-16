use log::info;

use syact::prelude::*;
use sybot::prelude::*;

mod sya_mini;

fn main() -> Result<(), sybot::Error> {
    // Logging
        env_logger::init();
        std::env::set_var("RUST_BACKTRACE", "1");
    // 

    info!("Logging enabled!");

    let mut rob= sya_mini::sya_mini_rob()?;
    let mut stat = sya_mini::SyaMiniStation::new();

    rob.setup()?;
    stat.home(&mut rob)?;

    dbg!(stat);

    Ok(())
}
