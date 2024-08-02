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
- Subsystem creation
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
- Create Command
  - You'll need a command to run your system with an xbox controller.
  - Create this command and use Y axes from the two sticks to drive the left and right sides of the system
  - You may want to consider adding a deadband elimination, so that when the xbox isn't touched, even if it isn't at zero exactly we still stop the system
- Testing
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
- Finished Subsystem code
  - Tank Drive capability with PercentVBus and stop
  - Methods to access sensor information
    - Encoder Position/Velocity for each side
    - Pigeon Yaw
      - Pigeon uses Phoenix V6 
  - Dashboard prints of sensor information
  - Accurate sensor information
    - For the pigeon: a detailed explanation (in a comment) on what we can expect from the values. We want to know the limits of values, direction. e.g. if we rotate clockwise for 90 degrees, what will be the value?
- Command
  - A command to drive the system with xbox controller
- Robot code
  - Code in robot class that creates the system and runs the commnad
    - run command in `teleopInit` 
 
#### Intake  

Implement the subsystem. Include definitions for the motor controller and limit switch.

Create methods for basic PercentVBus rotation of the motor, as well as a method for accessing the limit switch state. Add dashboard display of the limit switch state.

Create a command to rotate the intake motor based on the `XboxController`. Use axes and not buttons.

Guidelines:
- Subsystem creation
  - Define all motor controllers and sensors used in the system
  - Construct these components in the constructor
  - Remember to configure the controller properly
    - at the very least reset to factory default
  - You will need to have the following set of method
    - a way to rotate the motor based on PercentVBus
      - you should have constant speeds for in and out (in and out doesn't have to be the same) 
    - a way to stop the motor rotation
    - a way to access limit switch information
  - Remember to add print of sensors information to the dashboard
- Create Command
  - You'll need a command to run your system with an xbox controller.
  - Create a command or more and attach them to buttons. At the very least: one button needs to pull note in and one out.
  - Configure it so that holding the buttons is required. This eliminates our need for a `isFinished` for the moment
  - don't use limit switch in `isFinished` for now
- Testing
  - Make sure the system moves as expected in both speed and direction
  - Check different speeds for note in and note out. Find optimal speeds and note them in your code.
  - Try inserting the note from different positions and directions to make sure the note is collected well
  - Test the limit switch to make sure it works (use shuffleboard to view its state)
  - Push in a note and see when the limit switch detects the note
  - Pull the note out and see when the limit switch no longer detects the note

Requirements:
- Finished Subsystem code
  - Rotate in and out capability with PercentVBus and stop
    - in and out done in a constant speed
    - speed should be determined and tested (not arbitrary) 
  - Methods to access sensor information
    - Limit switch
  - Dashboard prints of sensor information
  - Accurate sensor information
- Command
  - A command (or more) to rotate the intake in and out
    - the commands should not used limit switch for `isFinished` in this phase.
- Robot code
  - Code in robot class that creates the system and runs the commnad
    - attach commands to button such that: holding `RB` pulls note in, holding `LB` pushes note out


#### Shooter

Implement the subsystem. Include definitions for the motor controller and encoder sensors.

Create methods for basic PercentVBus rotation of the motors, as well as methods for accessing the velocity measured by each encoder. Add dashboard display of these values.

Create a command to rotate the motors based on the `XboxController`. Use axes and not buttons. So moving the Y axis up, rotates the motors.

Guidelines:
- Subsystem creation
  - Define all motor controllers and sensors used in the system
  - Construct these components in the constructor
  - Remember to configure the controller properly
    - at the very least reset to factory default
  - You will need to have the following set of method
    - a way to rotate the motor based on PercentVBus
      - you should have constant speeds for rotation
    - a way to stop the motor rotation
    - a way to access velocity information for each information
    - avoid using follow here, to allow independent control for each controller in later phases
  - Remember to add print of sensors information to the dashboard
- Create Command
  - You'll need a command to run your system with an xbox controller.
  - Create a command and attach it to buttons. At the very least: one button needs to shoot out.
  - Configure it so that holding the buttons is required. This eliminates our need for a `isFinished` for the moment
- Testing
  - Make sure the system moves as expected in both speed and direction
    - due to the configuration of the system, two motors will be inverted 
  - Make sure all the encoders show expected velocity measurements
    - The speeds should be similar between the motors, but not exactly the same
  - Try running the motors at different velocities
    - Insert a note and look at the ranges for shooting depending on the speed of the motor
    - Select an initial constant speed to use for testing for now
    - Check the minimum speed for the shooter to shoot at all (note this in code comments)
    - Make sure all motors are relatively rotating at the same speeds  

Requirements:
- Finished Subsystem code
  - Rotate out capability with PercentVBus and stop
    - use a constant speed for out for now
  - Methods to access sensor information
    - Velocity for each motor
  - Dashboard prints of sensor information
  - Accurate sensor information
- Command
  - A command to rotate the shooter out
    - the commands should not finish
- Robot code
  - Code in robot class that creates the system and runs the commnad
    - attach command to button such that: holding `X` shoots out
      
#### Arm

Implement the subsystem. Include definition for the motor controller and Through-Bore encoder.

Create methods for basic PercentVBus rotation of the motor to move the arm up and down, as well as methods for accessing the position measured by the Through-Bore and NEO encoder, and velocity for NEO encoder. Add dashboard display of these sensor values.

Create a command to rotate the motors based on the `XboxController`. Use axes and not buttons. So moving the Y axis up, raises the arm, and down lowers the arm. Use the speed parameter from the axis but be wary of high values as we don't want to damage the arm (you may wish to limit the values up to 0.5).

Guidelines:
- Subsystem creation
  - Define all motor controllers and sensors used in the system
  - Construct these components in the constructor
  - Remember to configure the controller properly
    - at the very least reset to factory default
  - You will need to have the following set of method
    - a way to rotate the motor based on PercentVBus
      - you should have constant speeds for up and down (in and out doesn't have to be the same) 
    - a way to stop the motor rotation
    - a way to access sensor information
      - Position and velocity from NEO Encoder
      - Position from Through-Bore encoder
  - Remember to add print of sensors information to the dashboard
- Create Command
  - You'll need a command to run your system with an xbox controller.
  - Create a command or more and attach them to buttons. At the very least: one button needs to raise arm and one to lower arm.
  - Configure it so that holding the buttons is required. This eliminates our need for a `isFinished` for the moment
- Testing
  - Make sure the system moves as expected in both speed and direction
  - Check and comment in code on how far the arm can move in each direction
    - Be careful as to not damage the system while doing this
  - Try moving the arm at different speeds, note the slowest speed for the arm to even move and the fastest speed in which the arm moves without damaging anything.
  - Check that the position information from both encoders updates consistently and reflects the actual position of the arm
    - Since the Through-Bore is an absolute encoder, you will want to check how its values are distributed over the motion arc of the arm
    - We would want the Through-Bore to report 0 when the arm is on the floor and for the angle to increase as the arm raises. This makes the values intuitive to the real position of the arm. Check what the encoder normally report and see if it needs changes. Figure out how to change this. 


Requirements:
- Finished Subsystem code
  - Rotate out capability with PercentVBus and stop
    - use a constant speed for up and down (not necessarily the same speed)
  - Methods to access sensor information
    - Position and velocity from NEO encoder
    - Position from Through-Bore encoder
  - Dashboard prints of sensor information
  - Accurate sensor information
    - The Through-Bore Encoder should have its 0 position when the arm is on the floor. Raising the arm will increase the angle value.
  - Software checks limit the motion of the arm from exceeding its mechanical limitations.
    - Use information from the Through-Bore Encoder to limit this motion.
- Command
  - A command (or several) to raise the arm and lower the arm
    - the commands should not finish
    - use a speed for the arm that allow for rapid motion, but does not damage the arm or the system.
- Robot code
  - Code in robot class that creates the system and runs the commnad
    - attach commands to buttons such that: holding `DPad Up` (0) raises the arm and `DPad Down` (180) lowers the arm

### Phase 2 - Improved Control

In this phase we will improve the capability of each system by adding more capable control capabilities, that is, adding control loops to have more efficent and capable systems. This will mean different things for each system, but in general, we want to offload basic work from the robot operators to algorithms.

#### Drive

We want the capability to track the robot's position in the field. This will allow us to build autonomous actions based on the robot's absolute position, making it quite useful. You'll need to use the `DifferentialDriveOdometery` which works with the encoders and pigeon of the system

Guidelines:
- Create and update field positining in subsystem code using encoders and pigeon
  - Use the `DifferentialDriveOdometery` class, construct it in the constructor with an initial position of 0
  - Update it in `periodic`
  - Note about `DifferentialDriveOdometery` using `Rotation2d` for angle information, you can create this from `Rotation2d.fromDegrees`.
- Display of position via a `Field2D` object
  - Make sure to add the object to the dashboard (only once)
  - Make sure to update the robot position after odometery update
- Allow reseting the positioning to make the odometery think we are at a different position. This can allow us to lie about our positining.
  - Look at `DifferentialDriveOdometery.resetPosition`
- Test and verify position tracking is actually accurate
  - try wild motions with the robot in the air and on the ground. Make sure the tracking reflects accurate positining. Check with a Tape measure.
       
Requirements:
  - Subsystem has odometery capability
    - Odometery is accurate and updates smoothly
    - Odometery can be reset according to a wanted `Pose2d` so we could lie about our position
      - Post reset, odometery works well with the new position 

#### Intake

We want to control the intake with conjuction with the limit switch. This will allow the robot operator to no longer have to guess and stop commands themselves. 

Guidelines:
- Update or create commands for intake in and intake out
 - update the `isFinished` for both commands so that the commands finish depending on the note state as shown from the limit switch.
 - For intake in: run until the limit switch shows that the note is in the system, then stop
 - For intake out: run until liimt switch shows the note is no longer in the system
- Testing
 - test these changes by inserting notes into the system and showing that the commands function properly
 - we expect to see that during intake in, the command stops when the note has entered
 - while for intake out, the command stops when the note has fully left the system
 - test this in and out multiple times, with the arm at multiple positions and the note being at different orientations to the system
   - this will make sure that if the note is on the floor in peculiar ways, we can still work with it well 

Requirements:
- Commands for intake update to use the limit switch state
 - For intake in command, run until limit switch detects the note is in the system
 - For intake out command, run until the limit switch detects the note is no longer in the system  

#### Shooter

We want the capability to control the shooter based on a specific velocity (usually measured in RPM). This can give us the capability to provide and use precision shooting based on initial note velocity.

Guidelines:
- Added Closed-Loop Velocity to control the velocity of the motors
  - Use integrated SparkMAX PIDF Velocity control mode
  - Control each motor individually
    - Consider: why do we want to do this
  - Tune the control loop with REV Hardware Client
    - You only need to tune for one motor, as they are quite similar. So tune once and use this tuning with all motors.
  - Create a command which uses this to rotate the shooter at a specific velocity
- Test the command by running it with different speeds
  - make sure all motors are stabilized on the set point
  - make sure to tune until control is stable and quick
  - insert a note and watch how the note affect the control loop (does the speed loss get fixed quickly?)
- Attach the command to a button
  - while the button is held, the command is running (command doesn't need `isFinished` then )
  - select a base velocity which we will be using for our testing for attaching to the button
  - check to see that the button runs the command well
- Provide capability in the subsystem to check if all the motors have stabilized on a given RPM
  - this will allow us to check when we've reached our wanted velocity
  - test this capability. Add a print to the dashboard of whether the system is stabalized and run the command. See that when the system is stabilized, the method reports so, and when it isn't the method reports so. 

Requirements:
- Subsystem has velocity closed-loop control
  - Using the SparkMAX PIDF
  - Tuned well
- Has command to use this capability
  - Command is attached to a button for testing 

#### Arm

### Phase 3 - Integration I

### Phase 4 - Initial Autonomous

