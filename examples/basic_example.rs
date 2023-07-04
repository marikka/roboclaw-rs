use std::time::Duration;

use roboclaw::Roboclaw;
use serial::SerialPort;

// WARNING: Running this example will actually run the motors
fn main() -> Result<(), std::io::Error> {
    let serial_port_device_name =
        std::env::var("SERIAL_PORT").expect("SERIAL_PORT env variable not set");
    let mut port = serial::open(&serial_port_device_name).expect("opening serial failed");
    port.reconfigure(&|settings| {
        settings.set_baud_rate(serial::Baud9600)?;
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    })?;
    port.set_timeout(Duration::from_millis(1000))?;

    let mut roboclaw = Roboclaw::new(port);

    let config = roboclaw.get_config()?;
    let status = roboclaw.read_error()?;
    let main_battery_voltage = roboclaw.read_main_battery_voltage()?;
    let main_voltage_settings = roboclaw.read_min_max_main_voltages()?;

    println!("config: {:?}", config);
    println!("status: {:?}", status);
    println!("main battery voltage: {}", main_battery_voltage);
    println!(
        "main voltage settigns: min: {} max:{}",
        main_voltage_settings.0, main_voltage_settings.1
    );

    println!("driving");
    roboclaw.forward_mixed(100)?;
    roboclaw.turn_left_mixed(0)?;
    std::thread::sleep(Duration::from_millis(1000));
    println!("stopping");
    roboclaw.forward_mixed(0)?;
    println!("end");
    Ok(())
}
