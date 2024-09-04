package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.DriveTeleopCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.ArmMoveToFloorCommand;
import frc.robot.commands.ArmMoveToShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.commands.ShootOut;
import frc.robot.commands.ShooterPID;
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

        POVButton dPadUp = new POVButton(xboxController, RobotMap.D_PAD_TO_SHOOTER_ANGLE);
        POVButton dPadDown = new POVButton(xboxController, RobotMap.D_PAD_TO_FLOOR_ANGLE);

        dPadUp.onTrue(new ArmMoveToShooterCommand(armSystem));
        dPadDown.onTrue(new ArmMoveToFloorCommand(armSystem));

        new JoystickButton(xboxController, XboxController.Button.kX.value).whileTrue(new ShootOut(shooterSystem));
        new JoystickButton(xboxController, XboxController.Button.kB.value).whileTrue(new ShooterPID(shooterSystem, RobotMap.SHOOTER_TARGET_RPM));
        new JoystickButton(xboxController, XboxController.Button.kY.value).whileTrue(new OuttakeCommand(intakeSystem));
        new JoystickButton(xboxController, XboxController.Button.kA.value).whileTrue(new IntakeCommand(intakeSystem));

        SequentialCommandGroup collectNote = new SequentialCommandGroup(
                new ArmMoveToFloorCommand(armSystem),
                new IntakeCommand(intakeSystem)
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

        ParallelDeadlineGroup moveToFloorAndIntake = new ParallelDeadlineGroup(
                new IntakeCommand(intakeSystem),
                new ArmMoveToFloorCommand(armSystem)
        );

        new JoystickButton(xboxController,XboxController.Button.kA.value).onTrue(moveToFloorAndIntake);
        new JoystickButton(xboxController, XboxController.Button.kB.value).onTrue(
                new InstantCommand(moveToFloorAndIntake::cancel)
        );
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
