# R1 Profiler Rizal

Profiler ini dipasang untuk mencari bagian program yang pernah memakan waktu lama / hang sesaat.
Data disimpan di variabel global `g_prof[]`, lalu ikut dicetak dari `StartT_MonitorTask()` bersama monitoring task yang sudah ada.

## Compiler flag

Default profiler = OFF.

Untuk menyalakan:

```text
R1_PROFILER_ENABLE=1
```

Untuk mematikan:

```text
R1_PROFILER_ENABLE=0
```

Di STM32CubeIDE:

```text
Project Properties
-> C/C++ Build
-> Settings
-> MCU GCC Compiler
-> Preprocessor
-> Defined symbols
-> Add: R1_PROFILER_ENABLE=1
```

Kalau mau matikan untuk match, ganti jadi `R1_PROFILER_ENABLE=0` atau hapus symbol tersebut.
Saat OFF, macro `PROF_BEGIN()` dan `PROF_END()` menjadi no-op.

## File baru

```text
Core/Inc/Profiler.h
Core/Src/Profiler.c
```

## File yang sudah dipasang profiler

```text
Core/Src/main.c
Core/Src/Swerve.c
Core/Src/USART.c
Core/Src/Joystick.c
Core/Src/joystickRizal.c
Core/Src/dyn.c
```

## Section yang dipantau

```text
Task1.Loop
Task1.JoyDeadzone
Task2.Loop
SEQ.BaseCurrent
SEQ.BaseSlow
UART.Frame
UART.DMA_Start
UART.WaitDone
IMU.Loop
IMU.ParseByte
Joy.DMA_Check
Joy.ReadArduino
Joy.ParsePacket
Joy.HeadingRizal
Swerve.Kinematik
Swerve.RodaStop
USART.SetRoda
USART.QueuePut
DYN.MoveServo
DYN.SetSpeed
```

## Cara membaca output

Contoh:

```text
------ PROFILER ------
CFG       : R1_PROFILER_ENABLE=1 print=12
TOP MAX   : [13] Joy.HeadingRizal max=235 ms
TOP ACTIVE: [7] UART.WaitDone active=0 ms
ID Name               LastMS  MaxMS  LastUS   MaxUS    Hit  Over  ActMS Re
13 Joy.HeadingRizal        6    235    6120  235000    800     1      0  0
 7 UART.WaitDone          20     25   20000   25000   2600     3      0  0
```

Artinya:

- `TOP MAX` = section yang pernah paling lama sejak boot / reset profiler.
- `TOP ACTIVE` = section yang saat print masih aktif paling lama.
- `LastMS` = durasi panggilan terakhir.
- `MaxMS` = durasi maksimum yang pernah terjadi.
- `Over` = berapa kali durasi melewati batas warning.
- `ActMS` = sudah berapa ms section itu sedang aktif.
- `Re` = re-enter count, biasanya tanda section yang sama masuk lagi sebelum keluar.

## Variable penting untuk Live Expressions

```c
g_prof[PROF_TASK2_LOOP].max_ms
g_prof[PROF_SEQ_BASE_CURRENT].max_ms
g_prof[PROF_JOYSTICK_RIZAL].max_ms
g_prof[PROF_SWERVE_KINEMATIK].max_ms
g_prof[PROF_SWERVE_RODA_STOP].max_ms
g_prof[PROF_USART_SET_RODA].max_ms
g_prof[PROF_UART_WAIT_DONE].max_ms

prof_top_max_id
prof_top_max_ms
prof_top_active_id
prof_top_active_ms
```

## Titik awal analisa kalau robot hang sesaat

1. Kalau `Joy.HeadingRizal` tinggi, cek fungsi `JoystickHeadingLock_RIZAL()` dan output ke `Kinematik()` / `RodaStop()`.
2. Kalau `Swerve.Kinematik` tinggi, cek hitungan swerve dan jumlah kirim roda.
3. Kalau `USART.SetRoda` / `USART.QueuePut` tinggi, cek queue UART motor.
4. Kalau `UART.WaitDone` tinggi, cek DMA TX USART3, semaphore callback, dan error UART.
5. Kalau `DYN.MoveServo` / `DYN.SetSpeed` tinggi, cek transmit blocking servo/dynamixel.
6. Kalau `Joy.ReadArduino` / `Joy.DMA_Check` tinggi, cek parser joystick DMA.

## Reset data profiler saat runtime

Bisa panggil:

```c
PROF_RESET();
```

Atau langsung:

```c
Prof_Reset();
```

Jika profiler OFF, fungsi ini tidak melakukan apa-apa.
