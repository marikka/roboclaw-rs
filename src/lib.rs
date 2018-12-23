extern crate serial;
use serial::prelude::*;

extern crate crc16;

const ADDRESS: u8 = 0x80;

pub enum Command {
    ForwardM1(u8),
    BackwardsM1(u8),
    ForwardM2(u8),
    BackwardsM2(u8),
    DriveM1(u8),
    DriveM2(u8),
    DriveForward(u8),
    DriveBacwards(u8),
    TurnRight(u8),
    TurnLeft(u8),
    Drive(u8),
    Turn(u8),
}

impl Command {
    pub fn code(&self) -> u8 {
        match self {
            Command::ForwardM1(_) => 0,
            Command::BackwardsM1(_) => 1,
            Command::ForwardM2(_) => 4,
            Command::BackwardsM2(_) => 5,
            Command::DriveM1(_) => 6,
            Command::DriveM2(_) => 7,
            Command::DriveForward(_) => 8,
            Command::DriveBacwards(_) => 9,
            Command::TurnRight(_) => 10,
            Command::TurnLeft(_) => 11,
            Command::Drive(_) => 12,
            Command::Turn(_) => 13,
        }
    }
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

pub fn speed_command_bytes(command: Command, speed: u8) -> Vec<u8> {
    vec![ADDRESS, command.code(), speed]
}

pub struct Roboclaw<'a> {
    port: &'a mut SerialPort
}

impl <'a>Roboclaw<'a> {
    pub fn new<T: SerialPort>(port: &'a mut T) -> Self {
        Roboclaw {port: port}
    }

    pub fn run_command(&mut self, command: Command) {
        let command_bytes = match command {
            Command::ForwardM1(speed) | Command::BackwardsM1(speed)
            | Command::ForwardM2(speed) | Command::BackwardsM2(speed)
            | Command::DriveM1(speed) | Command::DriveM2(speed)
            | Command::DriveForward(speed) | Command::DriveBacwards(speed)
            | Command::TurnRight(speed) | Command::TurnLeft(speed)
            | Command::Drive(speed) | Command::Turn(speed) => speed_command_bytes(command, speed)
        };
        let checksum = crc(&command_bytes);
        let command_bytes = [&[ADDRESS], &command_bytes[..], &checksum[..]].concat();
        self.port.write(&command_bytes[..]).unwrap();
    }
}