extern crate linux_embedded_hal as linux_hal;
extern crate embedded_hal;
extern crate bma2xx;

use std::{thread, time};

use linux_hal::{
    I2cdev,
};

use bma2xx::{
    Bma2xx,
    FIFOConfig,
    AxisData,
};

fn main() {
    let i2c = I2cdev::new("/dev/i2c-1").unwrap();
    // On real hardware, development was done with the alternate address
    let mut accel = Bma2xx::new(i2c, bma2xx::Address::Alternate);
    // How long between axis readings
    let sleep_time = time::Duration::from_millis(20);

    accel.reset().unwrap();

    let chip_id = accel.who_am_i().unwrap();
    println!("About to Begin. ID is {} and should be {}", chip_id, bma2xx::IDENTIFIER);

    let temp = accel.temperature().unwrap();
    println!("Temperature: {}", temp);

    let writes_remaining = accel.eeprom_writes_remaining().unwrap();
    println!("EEPROM Writes Remaining: {}", writes_remaining);

    let nvm_data = accel.eeprom_data().unwrap();
    print!("EEPROM Data: ");
    for byte in &nvm_data {
        print!("{:02X} ", byte);
    }
    println!();


    // FIFO-related goodness!
    // TODO: Find out why the output data wasn't consistent
    let fifo_size = accel.fifo_size().unwrap();
    println!("Events in the FIFO: {}", fifo_size);
    let mut events = [AxisData {value: 0, changed: false}; 3];
    accel.fifo_set_mode(FIFOConfig::BYPASS).unwrap();
    accel.fifo_get(&mut events).unwrap();

    loop {
        let x = accel.axis_x().unwrap();
        let y = accel.axis_y().unwrap();
        let z = accel.axis_z().unwrap();
        println!("X:{0:03} Y:{1:03} Z:{2:03}",
            x.value,
            y.value,
            z.value);

        thread::sleep(sleep_time);
    }
}

