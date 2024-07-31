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

## Desired Capabilities

We have compiled a set of capabilities we would like to see at the end of work. Some we will complete, others we might not.
- Quick and Accurate Control Loops
  - For all control loops (mostly Arm and Shooter)
  - Quick and accurate set-point settling
  - Based on SparkMAX-integrated PIDF for all loops
  - Possible usage of SmartMotion
- Capable Auto-Firing
  - Usage of Interpolation, accurate Arm and Shooter control and target aquisition
  - Be capable of looking-on and firing notes at targets from different orientations and ranges
  - As quick a process as possible (as soon as a note is collected, be ready to shoot at an instant)
  - Odometery or Vision based targeting
  - Possibility for firing without human operator approval (?)
- Accurate Field Odometery
  - Usage of Pigeon and encoders for basic Odometery
  - Integration of Vision for error correction and initial position configuration
- Auto-Collection
  - Detect and reach notes on the floor
  - Collect them once reached
  - Likely vision-based note locating
- PathPlanner
  - Usage of PathPlanner to plan and execute complex paths for autonomous mode
  - Integrated with event markers
  - Quick, accurate and smooth motion
  - Possibilty of on-the-fly paths in teleop.

## Phases

Our work will be divided into different phases. During each phase we will have a specific set of goals to accomplish for each system in the robot. Read and follow the requirements for your system in each phase. Depending on the phase, some systems will have less or more work required. This is very dependent on the system and cannot be avoided really. Each system in a phase will also have a set of guidelines to help direct you on what you should be doing.

The code for each phase will be done in a branch unique to that phase and system. Once coding and testing are finished, a review of the code will be done in a Pull Request. When the Pull Request is approved, the code will be merged into a mainline branch for that system. 

Make sure to commit and push at the end of a work period, as to save the progress and have a backup. 

### Phase 1 - Base 

In this phase we will be implementing the base of all subsystems, as well as minimal teleop capability. The intention for this is mostly to test the base code and make sure we understand the basic mechanics and behaviour of both the systems and their sensors.

For each system, create a subsystem class, implement basic access to sensors and control over the motors. After which create a command for use with an Xbox controller to test the system. See following details for specifics.

#### Drive

Implement subsystem. Include definitions for the motor controllers and pigeon. For the pigeon use Phoenix V6. 

Create methods for basic PercentVBus tank-drive control as well as method for accessing sensor information (for both encoders and pigeon). Add dashboard displays of each sensor in `periodic`.

Create a command to control the system with an `XboxController`. Use axes and not buttons.

Guidelines:
- [ ] Subsystem creation
  - Define all motor controllers and sensors used in the system
  - Construct these components in the constructor
  - Remember to configure the controllers properly
    - at the very least reset all devices to factory default
  - Use follow mode so that _VictorSPX_ controllers follow the _TalonSRX_ controllers.
  - Remember that once side should be inverted and that inverted isn't taken into account when using follow.
  - You will need to have the following set of method
    - a way to rotate the motors based on PercentVBus
    - a way to stop the motor rotation
    - a way to access position/velocity of each side of the drive
    - a way to access yaw information
  - Remember to add prints of all sensor information to the dashboard
    - we would want to see position and velocity information from the sensors
    - and yaw information from the pigeon 
- [ ] Create Command
  - You'll need a command to run your system with an xbox controller.
  - Create this command and use Y axes from the two sticks to drive the left and right sides of the system
  - You may want to consider adding a deadband elimination, so that when the xbox isn't touched, even if it isn't at zero exactly we still stop the system
- [ ] Testing
  - Make sure the drive system moves as expected according to xbox controller inputs
    - you'll want to look at driving forward/backwards and rotating
    - also make sure that each side can be driven independently
  - Make sure sensors show accurate data
    - look at the prints on the shuffleboard from your subsystem
    - Look at encoder position and velocity reporting
    - Look at pigeon Yaw reporting
    - Move the robot around and see that the sensor values make sense
      - move the robot on the floor for a know distance (use a tape measure) and compare encoder positions to known measurement
      - for velocity, keep the robot at a steady speed while moving it along a known distance; measure the time it took and compare this with encoder velocity measurement
      - rotate the robot and compare pigeon yaw to expected degrees
        - try rotating around in a circle multiple times and check the values.
  - More on Sensors
    - You should probably reset both the pigeon yaw and encoder positions in the constructor.
    - The pigeon Yaw is usually tracking rotating in an opposite direction then expected.
    - Consider how to handle clamping the pigeon Yaw between 0 and 360, as it is not limited to this range and we do want it to be
    - Detail in a comment in your code on how the pigeon values change with rotation of the robot

Requirements:
- [ ] Finished Subsystem code
  - Tank Drive capability with PercentVBus and stop
  - Methods to access sensor information
    - Encoder Position/Velocity for each side
    - Pigeon Yaw
      - Pigeon uses Phoenix V6 
  - Dashboard prints of sensor information
  - Accurate sensor information
    - Pigeon information limited between 0-360
- [ ] Command
  - A command to drive the system with xbox controller
- [ ] Robot code
  - Code in robot class that creates the system and runs the commnad
    - run command in `teleopInit` 
 
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
  - [ ] try inserting the note from different positions and directions to make sure the note is collected well
- [ ] test the limit switch detection
  - [ ] test the limit switch to make sure it works (use shuffleboard to view its state)
  - [ ] push in a note and see when the limit switch detects the note
  - [ ] pull the note out and see when the limit switch no longer detects the note
- [ ] final result
  - [ ] remove axis-based `XboxController` command
  - [ ] create a command (or more) to rotate the wheels in and out at constant speeds. Attach these commands to the `RB` and `LB` buttons of the intake
    - determine the constant speeds yourself, based on testing. Find a speed which is capable of collecting and releasing the note quickly, but not damage it
    - stop the commands when the note has entered or left the system
  - [ ] accurate information from the sensors is displayed on the shuffleboard
    - limit switch ball in indication

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
- [ ] final result
  - [ ] remove axis-based `XboxController` command
  - [ ] create a command to rotate all motors at a constant speed. Bind this to the `X` button, so that while it is held the commands run
    - determine the speed yourself, based on testing. Find a speed which is capable of firing the notes quickly and 1-2 meters away, but not damage them. 
  - [ ] accurate information from the sensors is displayed on the shuffleboard
    - Velocity from each NEO encoder
      
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
- [ ] final result
  - [ ] remove axis-based `XboxController` command
  - [ ] create a command (or more) to allow controlling the arm via the DPad of the `XboxController`. holding DPad up (0) will raise the arm at a constant speed; holding DPad down (180) will lower the arm at a constant speed.
    - determine the best speed yourself. It must not be too fast as to damage the arm, or too slow as to fail to raise the arm.
  - [ ] The Through-Bore Encoder has its 0 position when the arm is on the floor. Raising the arm will increase the angle value.
  - [ ] Software checks limit the motion of the arm from exceeding its mechanical limitations.
    - Use information from the Through-Bore Encoder to limit this motion.
  - [ ] accurate information from the sensors is displayed on the shuffleboard
    - Position and velocity from NEO encoder
    - Position from Through-Bore encoder  

### Phase 2 - Improved Control

In this phase we will improve the capability of each system by adding more capable control capabilities, that is, adding control loops to have more efficent and capable systems. This will mean different things for each system, but in general, we want to offload basic work from the robot operators to algorithms.

#### Drive

We want the capability to track the robot's position in the field. This will allow us to build autonomous actions based on the robot's absolute position, making it quite useful. You'll need to use the `DifferentialDriveOdometery` which works with the encoders and pigeon of the system

Guidelines:
- [ ] Add field odometery
  - [ ] Create and update field positining in subsystem code using encoders and pigeon
    - Use the `DifferentialDriveOdometery` class, construct it in the constructor with an initial position of 0
    - Update it in `periodic`
    - Note about `DifferentialDriveOdometery` using `Rotation2d` for angle information, you can create this from `Rotation2d.fromDegrees`.
  - [ ] Display of position via a `Field2D` object
    - Make sure to add the object to the dashboard (only once)
    - Make sure to update the robot position after odometery update
  - [ ] Allow reseting the positioning to make the odometery think we are at a different position. This can allow us to lie about our positining.
    - Look at `DifferentialDriveOdometery.resetPosition`
  - [ ] Test and verify position tracking is actually accurate
    - try wild motions with the robot in the air and on the ground. Make sure the tracking reflects accurate positining. Check with a Tape measure.
       
Requirements:
  - [ ] Subsystem has odometery capability
    - Odometery is accurate and updates smoothly
    - Odometery can be reset according to a wanted `Pose2d` so we could lie about our position
      - Post reset, odometery works well with the new position 

#### Intake

#### Shooter

We want the capability to control the shooter based on a specific velocity (usually measured in RPM). This can give us the capability to provide and use precision shooting based on initial note velocity.

Guidelines:
- [ ] Added Closed-Loop Velocity to control the velocity of the motors
  - [ ] Use integrated SparkMAX PIDF Velocity control mode
  - [ ] Control each motor individually
    - Consider: why do we want to do this
  - [ ] Tune the control loop with REV Hardware Client
    - You only need to tune for one motor, as they are quite similar. So tune once and use this tuning with all motors.
  - [ ] Create a command which uses this to rotate the shooter at a specific velocity
  - [ ] Test the command by running it with different speeds
    - make sure all motors are stabilized on the set point
    - make sure to tune until control is stable and quick
    - insert a note and watch how the note affect the control loop (does the speed loss get fixed quickly?)
  - [ ] Attach the command to a button
    - while the button is held, the command is running (command doesn't need `isFinished` then )
    - select a base velocity which we will be using for our testing for attaching to the button
  - [ ] Provide capability in the subsystem to check if all the motors have stabilized on a given RPM
    - this will allow us to check when we've reached our wanted velocity

Requirements:
- [ ] Subsystem has velocity closed-loop control
  - Using the SparkMAX PIDF
  - Tuned well
- [ ] Has command to use this capability
  - Command is attached to a button for testing 

#### Arm

### Phase 3 - Integration

### Phase 4 - Initial Autonomous

