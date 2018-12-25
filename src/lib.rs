extern crate serial;
use serial::prelude::*;

extern crate crc16;

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
			SETDEADBAND	= 76,
			GETDEADBAND	= 77,
			GETENCODERS = 78,
			GETISPEEDS = 79,
			RESTOREDEFAULTS = 80,
			GETTEMP = 82,
			GETTEMP2 = 83,	//Only valid on some models
			GETERROR = 90,
			GETENCODERMODE = 91,
			SETM1ENCODERMODE = 92,
			SETM2ENCODERMODE = 93,
			WRITENVM = 94,
			READNVM = 95,	//Reloads values from Flash into Ram
			SETCONFIG = 98,
			GETCONFIG = 99,
			SETM1MAXCURRENT = 133,
			SETM2MAXCURRENT = 134,
			GETM1MAXCURRENT = 135,
			GETM2MAXCURRENT = 136,
			SETPWMMODE = 148,
			GETPWMMODE = 149,
			FLAGBOOTLOADER = 255 //Only available via USB communications
}

fn split_u16(x: u16) -> (u8, u8) {
    let high: u8 = (x >> 8) as u8;
    let low: u8 = x as u8;
    (high, low)
}

fn crc(buf: &Vec<u8>) -> Vec<u8> {
    let crc = crc16::State::<crc16::XMODEM>::calculate(&buf);
    let (high, low) = split_u16(crc);
    vec![high, low]
}

fn speed_command_bytes(command_code: u8, speed: u8) -> Vec<u8> {
    vec![ADDRESS, command_code, speed]
}

pub struct Roboclaw<'a> {
    port: &'a mut SerialPort
}

impl <'a>Roboclaw<'a> {
    pub fn new<T: SerialPort>(port: &'a mut T) -> Self {
        Roboclaw {port: port}
    }

/*
    pub fn run_command(&mut self, command: Command) -> Result<(), &str> {
        let command_bytes = match command {
            Command::ForwardM1(speed) | Command::BackwardsM1(speed)
            | Command::ForwardM2(speed) | Command::BackwardsM2(speed)
            | Command::DriveM1(speed) | Command::DriveM2(speed)
            | Command::MixedDriveForward(speed) | Command::MixedDriveBackwards(speed)
            | Command::MixedTurnRight(speed) | Command::MixedTurnLeft(speed)
            | Command::MixedDrive(speed) | Command::MixedTurn(speed) => speed_command_bytes(command, speed),
            Command::ReadFirmwareVersion => vec![21]
        };
        let checksum = crc(&command_bytes);
        let command_bytes = [&[ADDRESS], &command_bytes[..], &checksum[..]].concat();
        self.port.write(&command_bytes[..]).unwrap();
        let mut buf = [0u8];
        self.port.read(&mut buf).unwrap();
        if buf[0] != 0xFF {
            Err("error reading value")
        } else {
            Ok(())
        }
    }
    */

    fn run_speed_command(&mut self, command: Command, speed: u8) -> Result<(), &str>  {
        let command_bytes = speed_command_bytes(command as u8, speed);
        let checksum = crc(&command_bytes);
        let command_bytes = [&[ADDRESS], &command_bytes[..], &checksum[..]].concat();
        self.port.write(&command_bytes[..]).unwrap();
        let mut buf = [0u8];
        self.port.read(&mut buf).unwrap();
        if buf[0] != 0xFF {
            Err("error reading value")
        } else {
            Ok(())
        }
    }

    pub fn forward_m1(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::M1FORWARD, speed)
    }

    pub fn backward_m1(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::M1BACKWARD, speed)
    }

    pub fn set_min_voltage_main_battery(voltage: u8) {
        unimplemented!()
    }

    pub fn set_max_voltage_main_battery(voltage: u8) {
        unimplemented!()
    }

    pub fn forward_m2(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::M2FORWARD, speed)
    }

    pub fn backward_m2(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::M2BACKWARD, speed)
    }

    pub fn forward_backward_m1(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::M17BIT, speed)
    }

    pub fn forward_backward_m2(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::M27BIT, speed)
    }

    pub fn forward_mixed(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::MIXEDFORWARD, speed)
    }

    pub fn backward_mixed(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::MIXEDBACKWARD, speed)
    }

    pub fn turn_right_mixed(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::MIXEDRIGHT, speed)
    }

    pub fn turn_left_mixed(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::MIXEDLEFT, speed)
    }

    pub fn forward_backward_mixed(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::MIXEDFB, speed)
    }

    pub fn left_right_mixed(&mut self, speed: u8) -> Result<(), &str> {
        self.run_speed_command(Command::MIXEDLR, speed)
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
	bool ResetEncoders(uint8_t address);
	bool ReadVersion(uint8_t address,char *version);
	uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid=NULL);
	uint16_t ReadLogicBatteryVoltage(uint8_t address,bool *valid=NULL);
	bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage);
	bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage);
	bool SetM1VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
	bool SetM2VelocityPID(uint8_t address, float Kp, float Ki, float Kd, uint32_t qpps);
	uint32_t ReadISpeedM1(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);
	uint32_t ReadISpeedM2(uint8_t address,uint8_t *status=NULL,bool *valid=NULL);
	bool DutyM1(uint8_t address, uint16_t duty);
	bool DutyM2(uint8_t address, uint16_t duty);
	bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);
	bool SpeedM1(uint8_t address, uint32_t speed);
	bool SpeedM2(uint8_t address, uint32_t speed);
	bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2);
	bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);
	bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);
	bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t speed2);
	bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);
	bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance, uint8_t flag=0);
	bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
	bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);
	bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed, uint32_t distance, uint8_t flag=0);
	bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t speed1, uint32_t distance1, uint32_t speed2, uint32_t distance2, uint8_t flag=0);
	bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2);
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
	bool ReadMinMaxMainVoltages(uint8_t address,uint16_t &min,uint16_t &max);
	bool ReadMinMaxLogicVoltages(uint8_t address,uint16_t &min,uint16_t &max);
	bool SetM1PositionPID(uint8_t address,float kp,float ki,float kd,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
	bool SetM2PositionPID(uint8_t address,float kp,float ki,float kd,uint32_t kiMax,uint32_t deadzone,uint32_t min,uint32_t max);
	bool ReadM1PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
	bool ReadM2PositionPID(uint8_t address,float &Kp,float &Ki,float &Kd,uint32_t &KiMax,uint32_t &DeadZone,uint32_t &Min,uint32_t &Max);
	bool SpeedAccelDeccelPositionM1(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
	bool SpeedAccelDeccelPositionM2(uint8_t address,uint32_t accel,uint32_t speed,uint32_t deccel,uint32_t position,uint8_t flag);
	bool SpeedAccelDeccelPositionM1M2(uint8_t address,uint32_t accel1,uint32_t speed1,uint32_t deccel1,uint32_t position1,uint32_t accel2,uint32_t speed2,uint32_t deccel2,uint32_t position2,uint8_t flag);
	bool SetM1DefaultAccel(uint8_t address, uint32_t accel);
	bool SetM2DefaultAccel(uint8_t address, uint32_t accel);
	bool SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode, uint8_t S5mode);
	bool GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode, uint8_t &S5mode);
	bool SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max);
	bool GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max);
	bool ReadEncoders(uint8_t address,uint32_t &enc1,uint32_t &enc2);
	bool ReadISpeeds(uint8_t address,uint32_t &ispeed1,uint32_t &ispeed2);
	bool RestoreDefaults(uint8_t address);
	bool ReadTemp(uint8_t address, uint16_t &temp);
	bool ReadTemp2(uint8_t address, uint16_t &temp);
	uint16_t ReadError(uint8_t address,bool *valid=NULL);
	bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode);
	bool SetM1EncoderMode(uint8_t address,uint8_t mode);
	bool SetM2EncoderMode(uint8_t address,uint8_t mode);
	bool WriteNVM(uint8_t address);
	bool ReadNVM(uint8_t address);
	bool SetConfig(uint8_t address, uint16_t config);
	bool GetConfig(uint8_t address, uint16_t &config);
	bool SetM1MaxCurrent(uint8_t address,uint32_t max);
	bool SetM2MaxCurrent(uint8_t address,uint32_t max);
	bool ReadM1MaxCurrent(uint8_t address,uint32_t &max);
	bool ReadM2MaxCurrent(uint8_t address,uint32_t &max);
	bool SetPWMMode(uint8_t address, uint8_t mode);
	bool GetPWMMode(uint8_t address, uint8_t &mode);
    */
}