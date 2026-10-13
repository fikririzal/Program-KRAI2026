# UDPreadRobot

C++ program for Raspberry Pi that receives DualShock 4 joystick data over UDP, verifies its CRC-8 checksum, and forwards it to a robot controller through UART.

## Defaults

- UDP address: `192.25.2.1:1412`
- UART: `115200 baud`, 8N1
- Build target: Linux ARM64

## Run

```bash
./UDPreadRobot.out /dev/serial0 1412 192.25.2.1
```

