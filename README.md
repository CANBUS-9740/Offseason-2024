Rewrite of the 2024 robot in preperation to the 2024 offseason.

## Robot Description

### Drive

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
- Through-Bore Encoder, Absolute, Magnetic, RoboRIO DIO: 0
- Motor to Shaft Gear Ratio: $20 : 1$

## Phases

Our work will be divided into different phases. During each phase we will have a specific set of goals to accomplish for each system in the robot. Read and follow the requirements for your system in each phase.

The code for each phase will be done in a branch unique to that phase and system. Once coding and testing are finished, a review of the code will be done in a Pull Request. When the Pull Request is approved, the code will be merged into a mainline branch for that system. 

### Phase 1 - Base 

In this phase we will be implementing the base of all subsystems, as well as minimal teleop capability. The intention for this is mostly to test the base code and make sure we understand the basic mechanics and behaviour of both the systems and their sensors.

For each system, create a subsystem class, implement basic access to sensors and control over the motors. After which create a command for use with an Xbox controller to test the system. See following details for specifics.

#### Drive

Implement subsystem. Include definitions for the motor controllers and pigeon. For the pigeon use Phoenix V6. 

Create methods for basic PercentVBus tank-drive control as well as method for accessing sensor information (for both encoders and pigeon). Add dashboard displays of each sensor in `periodic`.

Create a command to control the system with an `XboxController`. Use axes and not buttons.

For the code to be finished:
- [ ] make sure you have
  - [ ] a way to rotate the motors based on PercentVBus
  - [ ] a way to stop the motor rotation
  - [ ] a way to access position/velocity of each side of the drive
  - [ ] a way to access yaw information
  - [ ] prints of all sensor information to the dashboard 
- [ ] run the drive command and test the system motion
  - [ ] make sure it moves as expected (direction for forward/backward motion, rotation)
  - [ ] check sensor values to make sure encoders show correct position and velocity information
    - [ ] place the robot on the ground, compare encoder measurement of movement to measurements done by a tape measure
    - [ ] for velocity, keep the robot at a steady speed while moving it along a known distance; measure the time it took and compare this with encoder velocity measurement
  - [ ] test the top speed of the robot on the ground and list it in a comment in your code.
  - [ ] check pigeon to make sure it displays expected YAW information when rotating
    - [ ] rotate the robot on the ground around a bit and make sure the measurement is correct
    - [ ] try rotating around in a circle multiple times and check the values. 
- consider about the pigeon
  - [ ] you should probably reset both the pigeon yaw and encoder positions in the constructor.
  - [ ] the pigeon Yaw is usually tracking rotating in an opposite direction then expected. 
  - [ ] consider how to handle clamping the pigeon Yaw between 0 and 360, as it is not limited to this range.
  - [ ] Detail in a comment in your code on how the pigeon values change with rotation of the robot
 
#### Intake  

Implement the subsystem. Include definitions for the motor controller and limit switch.

Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the limit switch state. Add dashboard display of the limit switch state.

Create a command to rotate the intake motor based on the `XboxController`. Use axes and not buttons.

For the code to be finished:
- [ ] make sure you have
  - [ ] a way to rotate the motor based on PercentVBus
  - [ ] a way to stop the motor rotation
  - [ ] a way to access limit switch information
  - [ ] prints of all sensor information to the dashboard 
- [ ] run the command and test system motion
  - [ ] make sure it moves as expected in both speed and direction
  - [ ] check different speeds for note in and note out. Find optimal speeds and note them in your code.
- [ ] test the limit switch detection
  - [ ] test the limit switch to make sure it works (use shuffleboard to view its state)
  - [ ] push in a note and see when the limit switch detects the note
  - [ ] pull the note out and see when the limit switch no longer detects the note

#### Shooter

Implement the subsystem. Include definitions for the motor controller and encoder sensors.

Create methods for basic PercentVBus rotation of the motors, as well as methods for accessing the velocity measured by each encoder. Add dashboard display of these values.

Create a command to rotate the motors based on the `XboxController`. Use axes and not buttons. So moving the Y axis up, rotates the motors.

For the code to be finished:
- [ ] make sure you have
  - [ ] a way to rotate the motors based on PercentVBus
  - [ ] a way to stop the motor rotation
  - [ ] a way to access velocity of each motor (methods)
  - [ ] prints of all sensor information to the dashboard
  - [ ] your command uses an `XboxController` axis to rotate the motors (the choice of axis is yours).
- [ ] run the command and test system motion
  - [ ] make sure the motors all rotate outwards from the system at differing speeds depending on the `XboxController` state.
- [ ] test sensor values
  - [ ] make sure all the encoders show expected velocity measurements
  - [ ] values should be rather similar
  - [ ] test for different speeds
  - [ ] velocities for the different motors should be similar to each other
- [ ] try inserting a note and rotate the motors to shoot it
  - [ ] note the affect the note has on the speed of the motors
  - [ ] try different speeds and see how far you can shoot the note
    - [ ] list in a comment in you code the maximum distance and minimum distance you managed to acheive
  - [ ] test and note the lowest speed at which a note will be fired and how far it reaches
  - [ ] test and note the maximum speed of the motors and how stable the system is when running it these speeds.

#### Arm

Implement the subsystem. Include definition for the motor controller and Through-Bore encoder.

Create methods for basic PercentVBus rotation of the motor to move the arm up and down, as well as methods for accessing the position measured by the Through-Bore and NEO encoder, and velocity for NEO encoder. Add dashboard display of these sensor values.

Create a command to rotate the motors based on the `XboxController`. Use axes and not buttons. So moving the Y axis up, raises the arm, and down lowers the arm. Use the speed parameter from the axis but be wary of high values as we don't want to damage the arm (you may wish to limit the values up to 0.5).

For the code to be finished:
- [ ] make sure you have
  - [ ] a way to rotate the motor based on PercentVBus
  - [ ] a way to stop the motor rotation
  - [ ] a way to access position based on both encoders (methods)
  - [ ] a way to access velocity based on NEO encoder.
  - [ ] prints of all sensor information to the dashboard
  - [ ] your command uses an `XboxController` axis to move the arm up and down (the choice of axis is yours).
- [ ] run the command and test system motion
  - [ ] make sure the arm is capabile of moving up and down
  - [ ] note the limits of motion for the arm (how far it can go in each direction)
  - [ ] try moving the arm at different speeds, note the slowest speed for the arm to even move and the fastest speed in which the arm moves without damaging anything.
- [ ] test sensor values
  - [ ] check that the position information from both encoders updates consistently and reflects the actual position of the arm
  - [ ] note the differences between information from the through-bore and information from the NEO encoder.
  - [ ] find a way to convert the Through-Bore encoder such that 0 position is with the arm on the floor, and raising the arm increases the sensor value  

### Phase 2 - Improved Control

In this phase we will improve the capability of each system by adding more capable control capabilities, that is, adding control loops to have more efficent and capable systems. This will mean different things for each system, but in general, we want to offload basic work from the robot operators to algorithms.

#### Drive

#### Intake

#### Shooter

#### Arm
