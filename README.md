Rewrite of the 2024 robot in preperation to the 2024 offseason.

## Robot Description

### Drive System

The drive system is a 4 by 6 tank drive, that is, it has 6 wheels and 4 motors, 3 wheels on each side and 2 motors on each side. The motors are CIM motors.

Each side is composed of 1 _TalonSRX_ and 1 _VictorSPX_ motor controllers.

Each _TalonSRX_ has an _SRX Magnetic Encoder_ connect to it. These sensors are placed post-gearbox.

A Pigeon 2 is placed on robot as well.

Specs:
- Motor Controllers
  - Left Front: TalonSRX, ID: 4
  - Left Back: VictorSPX, ID: 5
  - Right Front: VictorSPX, ID: 2
  - Right Back: TalonSRX, ID: 3
- Pigeon 2, Phoenix V6, ID: 7
- Motor to Wheel Gear Ratio: $8.45 : 1$
- Wheel Radius: 3 inch
- SRX Magnetic Encoder, Relative, Magnetic, PPR: 4096

### Intake

The intake system is a simple set of wheels connected to a single NEO 1.1 motor via belts. It is capable of holding a single note. Rotating the wheels inward will pull a note into the system, where it will be held and stored for use. Rotating the wheels outward will push a stored note out of the system.

The motor is controlled by a single _SparkMax_ motor controller. Since the motor is NEO, it has an integrated relative encoder which is connected
to the motor controller.

There is also a single mechanical limit switch inside the system. When a note is pulled into the system, it presses against this sensor. So this sensor can be used to identify if a note is in the system. The switch is connected in a _normally open_ configuration.

Specs:
- Motor Controllers
  - SparkMAX, ID: 9, Brushless
- Limit Switch, Mechanical, RoboRIO DIO: 5

### Shooter

The shooter system is a set of 4 wheels, placed on a straight board with 2 wheels per side. A note which is released from the intake into the system will be propelled out of the robot at speed. Each wheel is powered by a NEO 1.1 motor.

For each motor, there is a SparkMAX motor controller. Since the motors are NEO, each has an integrated relative encoder which is connected
to the motor controller.

Specs:
- Motor Controllers
  - Left Top, SparkMAX, ID: 13, Brushless
  - Left Bottom, SparkMAX, ID: 12, Brushless
  - Right Top, SparkMAX, ID: 15, Brushless
  - Right Bottom, SparkMAX, ID: 14, Brushless
- NEO 1.1 Encoder, Relative, Magnetic, CPR: 42

### Arm

The arm system is a single-jointed arm with the intake system mounted on its end. It is power by a single NEO 1.1 motor and has a range of motion of around 150 degrees. It is generally itended to be at one of 2 positions: floor position for collecting notes, shoot position for shooting notes.

The motor is controlled by a SparkMAX motor controller. Since the motor is NEO, it has an integrated relative encoder which is connected
to the motor controller.

There is also a Through-Bore Encoder placed on the shaft of the arm, after the motor and its gearbox.

Specs:
- Motor Controllers:
  - SparkMAX, ID: 8, Brushless
- NEO 1.1 Encoder, Relative, Magnetic, CPR: 42 
- Through-Bore Encoder, Absolute, Magnetic
- Motor to Shaft Gear Ratio: $20 : 1$
