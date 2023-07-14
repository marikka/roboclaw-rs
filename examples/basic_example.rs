use std::time::Duration;

use roboclaw::Roboclaw;
use serialport::{DataBits, FlowControl, Parity, StopBits};
use std::io::{ErrorKind as IoErrorKind, Read, Write};

pub struct SerialWrapper(pub Box<dyn serialport::SerialPort>);
impl SerialWrapper {
    pub fn open() -> Self {
        let port = serialport::new(
            std::env::var("SERIAL_PORT").expect("SERIAL_PORT env variable not set"),
            9600,
        )
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(1000))
        .open()
        .expect("Failed to open serial port");

        port.clear(serialport::ClearBuffer::Input)
            .expect("error clearing input buffer");
        SerialWrapper(port)
    }
}

/// Helper to convert std::io::Error to the nb::Error
fn translate_io_errors(err: std::io::Error) -> nb::Error<IoErrorKind> {
    match err.kind() {
        IoErrorKind::WouldBlock | IoErrorKind::TimedOut | IoErrorKind::Interrupted => {
            nb::Error::WouldBlock
        }
        err => nb::Error::Other(err),
    }
}

impl embedded_hal::serial::Read<u8> for SerialWrapper {
    type Error = IoErrorKind;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        let mut buffer = [0; 1];
        let bytes_read = self.0.read(&mut buffer).map_err(translate_io_errors)?;
        if bytes_read == 1 {
            Ok(buffer[0])
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl embedded_hal::serial::Write<u8> for SerialWrapper {
    type Error = IoErrorKind;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.0.write(&[word]).map_err(translate_io_errors)?;
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.0.flush().map_err(translate_io_errors)
    }
}

// WARNING: Running this example will actually run the motors
fn main() -> Result<(), nb::Error<roboclaw::Error>> {
    let port = SerialWrapper::open();

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
