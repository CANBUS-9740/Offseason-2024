package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
        xboxController = new XboxController(0);

        POVButton dPadUp = new POVButton(xboxController, 0);
        POVButton dPadDown = new POVButton(xboxController, 180);

        dPadUp.onTrue(new ArmMoveToShooterCommand(armSystem));
        dPadDown.onTrue(new ArmMoveToFloorCommand(armSystem));

        new JoystickButton(xboxController, XboxController.Button.kX.value).whileTrue(new ShootOut(shooterSystem));
        new JoystickButton(xboxController, XboxController.Button.kA.value).whileTrue(new ShooterPID(shooterSystem, 2000));
        new JoystickButton(xboxController, XboxController.Button.kY.value).whileTrue(new OuttakeCommand(intakeSystem));
        new JoystickButton(xboxController, XboxController.Button.kA.value).whileTrue(new IntakeCommand(intakeSystem));
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        DriveTeleopCommand driveTeleopCommand = new DriveTeleopCommand(driveSubsystem,xboxController);
        driveTeleopCommand.schedule();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void autonomousInit() {
        AutoBuilder.buildAuto("Simple Auto").schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testInit() {
        // Run SysId tests
        Commands.sequence(
                driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward),
                driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward),
                driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        ).schedule();
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
