use bitflags::bitflags;

bitflags! {
    pub struct ConfigFlags: u16 {
    const RC_MODE = 0x0000;
    const ANALOG_MODE = 0x0001;
    const SIMPLE_SERIAL_MODE = 0x0002;
    const PACKET_SERIAL_MODE = 0x0003;
    const BATTERY_MODE_OFF = 0x0000;
    const BATTERY_MODE_AUTO = 0x0004;
    const BATTERY_MODE_2_CELL = 0x0008;
    const BATTERY_MODE_3_CELL = 0x000C;
    const BATTERY_MODE_4_CELL = 0x0010;
    const BATTERY_MODE_5_CELL = 0x0014;
    const BATTERY_MODE_6_CELL = 0x0018;
    const BATTERY_MODE_7_CELL = 0x001C;
    const MIXING = 0x0020;
    const EXPONENTIAL = 0x0040;
    const MCU = 0x0080;
    const BAUDRATE_2400 = 0x0000;
    const BAUDRATE_9600 = 0x0020;
    const BAUDRATE_19200 = 0x0040;
    const BAUDRATE_38400 = 0x0060;
    const BAUDRATE_57600 = 0x0080;
    const BAUDRATE_115200 = 0x00A0;
    const BAUDRATE_230400 = 0x00C0;
    const BAUDRATE_460800 = 0x00E0;
    const FLIPSWITCH = 0x0100;
    const PACKET_ADDRESS_0x80 = 0x0000;
    const PACKET_ADDRESS_0x81 = 0x0100;
    const PACKET_ADDRESS_0x82 = 0x0200;
    const PACKET_ADDRESS_0x83 = 0x0300;
    const PACKET_ADDRESS_0x84 = 0x0400;
    const PACKET_ADDRESS_0x85 = 0x0500;
    const PACKET_ADDRESS_0x86 = 0x0600;
    const PACKET_ADDRESS_0x87 = 0x0700;
    const SLAVE_MODE = 0x0800;
    const RELAY_MODE = 0x1000;
    const SWAP_ENCODERS = 0x2000;
    const SWAP_BUTTONS = 0x4000;
    const MULTI_UNIT_MODE = 0x8000;
    }
}

bitflags! {
    pub struct StatusFlags: u16 {
    const NORMAL = 0x0000;
    const M1_OVERCURRENT_WARNING = 0x0001;
    const M2_OVERCURRENT_WARNING = 0x0002;
    const E_STOP = 0x0004;
    const TEMPERATURE_ERROR = 0x0008;
    const TEMPERATURE2_ERROR = 0x0010;
    const MAIN_BATTERY_HIGH_ERROR = 0x0020;
    const LOGIC_BATTERY_HIGH_ERROR = 0x0040;
    const LOGIC_BATTERY_LOW_ERROR = 0x0080;
    const M1_DRIVER_FAULT = 0x0100;
    const M2_DRIVER_FAULT = 0x0200;
    const MAIN_BATTERY_HIGH_WARNING = 0x0400;
    const MAIN_BATTERY_LOW_WARNING = 0x0800;
    const TERMPERATURE_WARNING = 0x1000;
    const TEMPERATURE2_WARNING = 0x2000;
    const M1_HOME = 0x4000;
    const M2_HOME = 0x8000;
    }
}

const ADDRESS: u8 = 0x80;

#[repr(u8)]
enum Command {
    M1FORWARD = 0,
    M1BACKWARD = 1,
    SETMINMB = 2,
    SETMAXMB = 3,
    M2FORWARD = 4,
    M2BACKWARD = 5,
    M17BIT = 6,
    M27BIT = 7,
    MIXEDFORWARD = 8,
    MIXEDBACKWARD = 9,
    MIXEDRIGHT = 10,
    MIXEDLEFT = 11,
    MIXEDFB = 12,
    MIXEDLR = 13,
    GETM1ENC = 16,
    GETM2ENC = 17,
    GETM1SPEED = 18,
    GETM2SPEED = 19,
    RESETENC = 20,
    GETVERSION = 21,
    SETM1ENCCOUNT = 22,
    SETM2ENCCOUNT = 23,
    GETMBATT = 24,
    GETLBATT = 25,
    SETMINLB = 26,
    SETMAXLB = 27,
    SETM1PID = 28,
    SETM2PID = 29,
    GETM1ISPEED = 30,
    GETM2ISPEED = 31,
    M1DUTY = 32,
    M2DUTY = 33,
    MIXEDDUTY = 34,
    M1SPEED = 35,
    M2SPEED = 36,
    MIXEDSPEED = 37,
    M1SPEEDACCEL = 38,
    M2SPEEDACCEL = 39,
    MIXEDSPEEDACCEL = 40,
    M1SPEEDDIST = 41,
    M2SPEEDDIST = 42,
    MIXEDSPEEDDIST = 43,
    M1SPEEDACCELDIST = 44,
    M2SPEEDACCELDIST = 45,
    MIXEDSPEEDACCELDIST = 46,
    GETBUFFERS = 47,
    GETPWMS = 48,
    GETCURRENTS = 49,
    MIXEDSPEED2ACCEL = 50,
    MIXEDSPEED2ACCELDIST = 51,
    M1DUTYACCEL = 52,
    M2DUTYACCEL = 53,
    MIXEDDUTYACCEL = 54,
    READM1PID = 55,
    READM2PID = 56,
    SETMAINVOLTAGES = 57,
    SETLOGICVOLTAGES = 58,
    GETMINMAXMAINVOLTAGES = 59,
    GETMINMAXLOGICVOLTAGES = 60,
    SETM1POSPID = 61,
    SETM2POSPID = 62,
    READM1POSPID = 63,
    READM2POSPID = 64,
    M1SPEEDACCELDECCELPOS = 65,
    M2SPEEDACCELDECCELPOS = 66,
    MIXEDSPEEDACCELDECCELPOS = 67,
    SETM1DEFAULTACCEL = 68,
    SETM2DEFAULTACCEL = 69,
    SETPINFUNCTIONS = 74,
    GETPINFUNCTIONS = 75,
    SETDEADBAND = 76,
    GETDEADBAND = 77,
    GETENCODERS = 78,
    GETISPEEDS = 79,
    RESTOREDEFAULTS = 80,
    GETTEMP = 82,
    GETTEMP2 = 83, //Only valid on some models
    GETERROR = 90,
    GETENCODERMODE = 91,
    SETM1ENCODERMODE = 92,
    SETM2ENCODERMODE = 93,
    WRITENVM = 94,
    READNVM = 95, //Reloads values from Flash into Ram
    SETCONFIG = 98,
    GETCONFIG = 99,
    SETM1MAXCURRENT = 133,
    SETM2MAXCURRENT = 134,
    GETM1MAXCURRENT = 135,
    GETM2MAXCURRENT = 136,
    SETPWMMODE = 148,
    GETPWMMODE = 149,
    FLAGBOOTLOADER = 255, //Only available via USB communications
}

#[derive(PartialEq, Debug)]
pub enum BufferStatus {
    NotEmpty(u8),
    Empty,
    LastCommandExecuting,
}

fn split_u16_u8(x: u16) -> [u8; 2] {
    [(x >> 8) as u8, x as u8]
}

fn split_i16_u8(x: i16) -> [u8; 2] {
    [(x >> 8) as u8, x as u8]
}

fn split_u32_u8(x: u32) -> [u8; 4] {
    [(x >> 24) as u8, (x >> 16) as u8, (x >> 8) as u8, x as u8]
}

fn split_i32_u8(x: i32) -> [u8; 4] {
    [(x >> 24) as u8, (x >> 16) as u8, (x >> 8) as u8, x as u8]
}

fn join_u8(high: u8, low: u8) -> u16 {
    ((high as u16) << 8) | low as u16
}

fn join_u8_u32(byte0: u8, byte1: u8, byte2: u8, byte3: u8) -> u32 {
    ((byte0 as u32) << 24) | ((byte1 as u32) << 16) | ((byte2 as u32) << 8) | (byte3 as u32)
}

fn crc(buf: &Vec<u8>) -> Vec<u8> {
    let crc = crc16::State::<crc16::XMODEM>::calculate(&buf);
    split_u16_u8(crc).to_vec()
}

pub struct Roboclaw {
    port: Box<dyn serialport::SerialPort>,
}

impl Roboclaw {
    pub fn new(port: Box<dyn serialport::SerialPort>) -> Self {
        Roboclaw { port }
    }

    fn read_command(&mut self, command_code: u8, num_bytes: usize) -> std::io::Result<Vec<u8>> {
        const CRC_SIZE: usize = 2;
        let command = vec![ADDRESS, command_code];
        self.port.write(&command[..])?;
        let mut buf = vec![0; num_bytes + CRC_SIZE];
        self.port.read(&mut buf)?;
        let crc = buf.split_off(num_bytes);
        let crc_read = join_u8(crc[0], crc[1]);
        let crc_calc = crc16::State::<crc16::XMODEM>::calculate(&[&command[..], &buf].concat());
        if crc_read == crc_calc {
            Ok(buf)
        } else {
            Err(std::io::Error::new(std::io::ErrorKind::Other, "crc error"))
        }
    }

    fn write_simple_command(&mut self, command_code: u8) -> std::io::Result<()> {
        let command = vec![ADDRESS, command_code];
        let crc = crc(&command);
        let command_bytes = [&[ADDRESS], &command[..], &crc[..]].concat();
        self.port.write(&command_bytes)?;
        let mut buf = vec![0; 1];
        self.port.read(&mut buf)?;
        if buf[0] == 0xFF {
            Ok(())
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "return value error",
            ))
        }
    }

    fn write_command(&mut self, command_code: u8, data: &Vec<u8>) -> std::io::Result<()> {
        let mut command = vec![ADDRESS, command_code];
        let mut data_copy = data.clone();
        command.append(&mut data_copy);
        let crc = crc(&command);
        let command_bytes = [&[ADDRESS], &command[..], &crc[..]].concat();
        self.port.write(&command_bytes)?;
        let mut buf = vec![0; 1];
        self.port.read(&mut buf)?;
        if buf[0] == 0xFF {
            Ok(())
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "return value error",
            ))
        }
    }

    pub fn forward_m1(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::M1FORWARD as u8, &vec![speed])
    }

    pub fn backward_m1(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::M1BACKWARD as u8, &vec![speed])
    }

    pub fn set_min_voltage_main_battery(voltage: u8) {
        unimplemented!()
    }

    pub fn set_max_voltage_main_battery(voltage: u8) {
        unimplemented!()
    }

    pub fn forward_m2(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::M2FORWARD as u8, &vec![speed])
    }

    pub fn backward_m2(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::M2BACKWARD as u8, &vec![speed])
    }

    pub fn forward_backward_m1(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::M17BIT as u8, &vec![speed])
    }

    pub fn forward_backward_m2(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::M27BIT as u8, &vec![speed])
    }

    pub fn forward_mixed(&mut self, speed: u8) -> Result<(), std::io::Error> {
        self.write_command(Command::MIXEDFORWARD as u8, &vec![speed])
    }

    pub fn backward_mixed(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::MIXEDBACKWARD as u8, &vec![speed])
    }

    pub fn turn_right_mixed(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::MIXEDRIGHT as u8, &vec![speed])
    }

    pub fn turn_left_mixed(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::MIXEDLEFT as u8, &vec![speed])
    }

    pub fn forward_backward_mixed(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::MIXEDFB as u8, &vec![speed])
    }

    pub fn left_right_mixed(&mut self, speed: u8) -> std::io::Result<()> {
        self.write_command(Command::MIXEDLR as u8, &vec![speed])
    }

    //uint32_t ReadEncM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
    pub fn read_enc_m1(&mut self) -> Result<u32, &str> {
        unimplemented!()
    }

    //uint32_t ReadEncM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
    pub fn read_enc_m2(&mut self) -> Result<u32, &str> {
        unimplemented!()
    }

    //bool SetEncM1(uint8_t address, int32_t val);
    pub fn set_enc_m1(&mut self, value: i32) -> Result<(), &str> {
        unimplemented!()
    }

    //bool SetEncM2(uint8_t address, int32_t val);
    pub fn set_enc_m2(&mut self, value: i32) -> Result<(), &str> {
        unimplemented!()
    }

    /*
    uint32_t ReadSpeedM1(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
    uint32_t ReadSpeedM2(uint8_t address, uint8_t *status=NULL,bool *valid=NULL);
    */
    //bool ResetEncoders(uint8_t address);
    pub fn reset_encoders(&mut self) -> Result<(), std::io::Error> {
        self.write_simple_command(Command::RESETENC as u8)
    }

    /*
    bool ReadVersion(uint8_t address,char *version);
    */

    //uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid=NULL);
    pub fn read_main_battery_voltage(&mut self) -> Result<f32, std::io::Error> {
        self.read_command(Command::GETMBATT as u8, 2)
            .map(|data| (join_u8(data[0], data[1]) as f32) / 10.0)
    }

    //uint16_t ReadLogicBatteryVoltage(uint8_t address,bool *valid=NULL);
    pub fn read_logic_battery_voltage(&mut self) -> Result<f32, std::io::Error> {
        self.read_command(Command::GETLBATT as u8, 2)
            .map(|data| (join_u8(data[0], data[1]) as f32) / 10.0)
    }

    /*
    bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage);
    bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage);
    bool SetM1VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
    bool SetM2VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
    uint32_t ReadISpeedM1(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);
    uint32_t ReadISpeedM2(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);
    */

    //bool DutyM1(uint8_t address, uint16_t duty);
    pub fn duty_m1(&mut self, duty: i16) -> std::io::Result<()> {
        self.write_command(Command::M1DUTY as u8, &split_i16_u8(duty).to_vec())
    }

    //bool DutyM2(uint8_t address, uint16_t duty);
    pub fn duty_m2(&mut self, duty: i16) -> std::io::Result<()> {
        self.write_command(Command::M2DUTY as u8, &split_i16_u8(duty).to_vec())
    }

    //bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);
    pub fn duty_m1_m2(&mut self, duty1: i16, duty2: i16) -> std::io::Result<()> {
        self.write_command(
            Command::MIXEDDUTY as u8,
            &[&split_i16_u8(duty1)[..], &split_i16_u8(duty2)[..]].concat(),
        )
    }

    /*


    bool SpeedM1(uint8_t address, uint32_t speed);
    bool SpeedM2(uint8_t address, uint32_t speed);
    */
    //bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2);
    pub fn speed_m1_m2(&mut self, speed_1: i32, speed_2: i32) -> Result<(), std::io::Error> {
        let speed_1_bytes = split_i32_u8(speed_1);
        let speed_2_bytes = split_i32_u8(speed_2);
        let data = [&speed_1_bytes[..], &speed_2_bytes[..]].concat();
        self.write_command(Command::MIXEDSPEED as u8, &data)
    }
    /*
    bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);
    bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);
    bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2);
    */
    //bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);
    pub fn speed_distance_m1(&mut self, speed: i32, distance: u32) -> Result<(), std::io::Error> {
        let speed_bytes = split_i32_u8(speed);
        let distance_bytes = split_u32_u8(distance);
        let data = [&speed_bytes[..], &distance_bytes[..], &vec![1u8]].concat();
        self.write_command(Command::M1SPEEDDIST as u8, &data)
    }

    //bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);
    pub fn speed_distance_m2(&mut self, speed: i32, distance: u32) -> Result<(), std::io::Error> {
        let speed_bytes = split_i32_u8(speed);
        let distance_bytes = split_u32_u8(distance);
        let data = [&speed_bytes[..], &distance_bytes[..], &vec![1u8]].concat();
        self.write_command(Command::M2SPEEDDIST as u8, &data)
    }

    //bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
    pub fn speed_distance_m1_m2(
        &mut self,
        speed_1: i32,
        distance_1: u32,
        speed_2: i32,
        distance_2: u32,
    ) -> Result<(), std::io::Error> {
        let speed_1_bytes = split_i32_u8(speed_1);
        let distance_1_bytes = split_u32_u8(distance_1);
        let speed_2_bytes = split_i32_u8(speed_2);
        let distance_2_bytes = split_u32_u8(distance_2);
        let data = [
            &speed_1_bytes[..],
            &distance_1_bytes[..],
            &speed_2_bytes[..],
            &distance_2_bytes[..],
            &vec![1u8],
        ]
        .concat();
        self.write_command(Command::MIXEDSPEEDDIST as u8, &data)
    }

    /*
    bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);
    bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);
    */

    //bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
    pub fn speed_accel_distance_m1_m2(
        &mut self,
        accel: u32,
        speed_1: i32,
        distance_1: u32,
        speed_2: i32,
        distance_2: u32,
    ) -> Result<(), std::io::Error> {
        let accel_bytes = split_u32_u8(accel);
        let speed_1_bytes = split_i32_u8(speed_1);
        let distance_1_bytes = split_u32_u8(distance_1);
        let speed_2_bytes = split_i32_u8(speed_2);
        let distance_2_bytes = split_u32_u8(distance_2);
        let data = [
            &accel_bytes[..],
            &speed_1_bytes[..],
            &distance_1_bytes[..],
            &speed_2_bytes[..],
            &distance_2_bytes[..],
            &vec![1u8],
        ]
        .concat();
        self.write_command(Command::MIXEDSPEEDACCELDIST as u8, &data)
    }

    //bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2);
    pub fn read_buffers(&mut self) -> std::io::Result<(BufferStatus, BufferStatus)> {
        self.read_command(Command::GETBUFFERS as u8, 2).map(|data| {
            (
                match data[0] {
                    0x0 => BufferStatus::LastCommandExecuting,
                    0x80 => BufferStatus::Empty,
                    num => BufferStatus::NotEmpty(num),
                },
                match data[1] {
                    0x0 => BufferStatus::LastCommandExecuting,
                    0x80 => BufferStatus::Empty,
                    num => BufferStatus::NotEmpty(num),
                },
            )
        })
    }
    /*
    bool ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2);
    bool ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2);
    bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t accel2, uint32_t speed2);
    bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1, uint32_t distance1, uint32_t accel2, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
    bool DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel);
    bool DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel);
    bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1, uint16_t duty2, uint32_t accel2);
    bool ReadM1VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps);
    bool ReadM2VelocityPID(uint8_t address,float &Kp_fp,float &Ki_fp,float &Kd_fp,uint32_t &qpps);
    bool SetMainVoltages(uint8_t address,uint16_t min,uint16_t max);
    bool SetLogicVoltages(uint8_t address,uint16_t min,uint16_t max);
    */

    //bool ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max);

    pub fn read_min_max_main_voltages(&mut self) -> Result<(f32, f32), std::io::Error> {
        self.read_command(Command::GETMINMAXMAINVOLTAGES as u8, 4)
            .map(|data| {
                (
                    join_u8(data[0], data[1]) as f32 / 10.0,
                    join_u8(data[2], data[3]) as f32 / 10.0,
                )
            })
    }

    /*
    bool ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max);
    bool SetM1PositionPID(uint8_t address,float kp,float ki,float kd,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
    bool SetM2PositionPID(uint8_t address,float kp,float ki,float kd,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
    bool ReadM1PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
    bool ReadM2PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
    bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
    bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
    */
    //bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag);
    pub fn speed_accel_deccel_position_m1_m2(
        &mut self,
        accel_1: u32,
        speed_1: i32,
        deccel_1: u32,
        position_1: u32,
        accel_2: u32,
        speed_2: i32,
        deccel_2: u32,
        position_2: u32,
    ) -> Result<(), std::io::Error> {
        let accel_1_bytes = split_u32_u8(accel_1);
        let speed_1_bytes = split_i32_u8(speed_1);
        let deccel_1_bytes = split_u32_u8(deccel_1);
        let position_1_bytes = split_u32_u8(position_1);

        let accel_2_bytes = split_u32_u8(accel_2);
        let speed_2_bytes = split_i32_u8(speed_2);
        let deccel_2_bytes = split_u32_u8(deccel_2);
        let position_2_bytes = split_u32_u8(position_2);

        let data = [
            &accel_1_bytes[..],
            &speed_1_bytes[..],
            &deccel_1_bytes[..],
            &position_1_bytes[..],
            &accel_2_bytes[..],
            &speed_2_bytes[..],
            &deccel_2_bytes[..],
            &position_2_bytes[..],
            &vec![1u8],
        ]
        .concat();
        self.write_command(Command::MIXEDSPEEDACCELDECCELPOS as u8, &data)
    }

    /*
    bool SetM1DefaultAccel(uint8_t address, uint32_t accel);
    bool SetM2DefaultAccel(uint8_t address, uint32_t accel);
    bool SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode);
    bool GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode);
    bool SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max);
    bool GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max);
    */
    //bool ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2);
    pub fn read_encoders(&mut self) -> Result<(u32, u32), std::io::Error> {
        self.read_command(Command::GETENCODERS as u8, 8)
            .map(|data| {
                (
                    join_u8_u32(data[0], data[1], data[2], data[3]),
                    join_u8_u32(data[4], data[5], data[6], data[7]),
                )
            })
    }

    /*
    bool ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2);
    bool RestoreDefaults(uint8_t address);
    bool ReadTemp(uint8_t address, uint16_t &temp);
    bool ReadTemp2(uint8_t address, uint16_t &temp);
    */

    //uint16_t ReadError(uint8_t address,bool *valid=NULL);
    pub fn read_error(&mut self) -> Result<StatusFlags, std::io::Error> {
        self.read_command(Command::GETERROR as u8, 2)
            .map(|data| StatusFlags::from_bits(join_u8(data[0], data[1])).unwrap())
    }

    /*
    bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode);
    bool SetM1EncoderMode(uint8_t address,uint8_t mode);
    bool SetM2EncoderMode(uint8_t address,uint8_t mode);
    bool WriteNVM(uint8_t address);
    bool ReadNVM(uint8_t address);
    bool SetConfig(uint8_t address, uint16_t config);
    */
    //bool GetConfig(uint8_t address, uint16_t &config);
    pub fn get_config(&mut self) -> Result<ConfigFlags, std::io::Error> {
        self.read_command(Command::GETCONFIG as u8, 2)
            .map(|data| ConfigFlags::from_bits(join_u8(data[0], data[1])).unwrap())
    }

    /*
    bool SetM1MaxCurrent(uint8_t address,uint32_t max);
    bool SetM2MaxCurrent(uint8_t address,uint32_t max);
    bool ReadM1MaxCurrent(uint8_t address,uint32_t &max);
    bool ReadM2MaxCurrent(uint8_t address,uint32_t &max);
    bool SetPWMMode(uint8_t address, uint8_t mode);
    bool GetPWMMode(uint8_t address, uint8_t &mode);
    */
}
