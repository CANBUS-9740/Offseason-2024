package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class Robot extends TimedRobot {

    private DriveSubsystem driveSubsystem;
    private ShooterSystem shooterSystem;
    private IntakeSystem intakeSystem;
    private ArmSystem armSystem;
    private XboxController xboxController;

    @Override
    public void robotInit() {
        driveSubsystem = new DriveSubsystem();
        shooterSystem = new ShooterSystem();
        intakeSystem = new IntakeSystem();
        armSystem = new ArmSystem();
        xboxController = new XboxController(RobotMap.XBOX_CONTROLLER_PORT);


        armSystem.setDefaultCommand(
                new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                                new IntakeSlowlyCommand(intakeSystem)
                        ),
                new ArmMoveToShooterCommand(armSystem)
                )
        );

        POVButton dPadUp = new POVButton(xboxController, RobotMap.D_PAD_TO_SHOOTER_ANGLE);
        POVButton dPadDown = new POVButton(xboxController, RobotMap.D_PAD_TO_FLOOR_ANGLE);

        dPadUp.onTrue(new ArmMoveToShooterCommand(armSystem));
        dPadDown.onTrue(new ArmMoveToFloorCommand(armSystem));

        new JoystickButton(xboxController, XboxController.Button.kX.value).whileTrue(new ShootOut(shooterSystem));
        new JoystickButton(xboxController, XboxController.Button.kY.value).whileTrue(new OuttakeCommand(intakeSystem));

        ParallelDeadlineGroup collectNote = new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> intakeSystem.isNoteInside()),
                new SequentialCommandGroup(
                        new ArmMoveToFloorCommand(armSystem),
                        new IntakeCommand(intakeSystem)
                )
        );

        new JoystickButton(xboxController,XboxController.Button.kA.value).onTrue(collectNote);
        new JoystickButton(xboxController, XboxController.Button.kB.value).onTrue(
                new InstantCommand(collectNote::cancel)
        );
    }

    @Override
    public void disabledInit() {


    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        DriveTeleopCommand driveTeleopCommand = new DriveTeleopCommand(driveSubsystem, xboxController);
        driveTeleopCommand.schedule();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationPeriodic() {

    }
}
