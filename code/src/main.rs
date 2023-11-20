use syact::prelude::*;
use sybot::prelude::*;

mod sya_mini;

fn main() -> Result<(), sybot::Error> {
    // Logging
        env_logger::init();
        std::env::set_var("RUST_BACKTRACE", "1");
    // 

    log::info!("Logging enabled!");

    let mut rob= sya_mini::sya_mini_rob()?;
    let mut desc = sya_mini::SyaMiniDesc::new();
    let mut stat = sya_mini::SyaMiniStation::new();

    rob.setup()?;
    stat.home(&mut rob)?;

    dbg!(desc.wobj());
    desc.update(&mut rob, &[ Phi(1.0), Phi::ZERO, Phi::ZERO, Phi::ZERO ])?;
    dbg!(desc.wobj());
    
    Ok(())
}
