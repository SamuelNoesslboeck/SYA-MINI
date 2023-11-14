use syact::prelude::*;
use sybot::prelude::*;

mod sya_mini;

fn main() -> Result<(), sybot::Error> {
    let mut rob= sya_mini::sya_mini_rob()?;
    let mut stat = sya_mini::SyaMiniStation::new();

    stat.home(&mut rob)?;

    Ok(())
}
