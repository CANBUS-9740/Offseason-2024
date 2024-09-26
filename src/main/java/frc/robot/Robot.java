package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private XboxController driveController;
    private XboxController operatorController;
    private Command shootNoteSpeaker;

    @Override
    public void robotInit() {
        driveSubsystem = new DriveSubsystem();
        shooterSystem = new ShooterSystem();
        intakeSystem = new IntakeSystem();
        armSystem = new ArmSystem();
        driveController = new XboxController(0);
        operatorController = new XboxController(1);

        shootNoteSpeaker = new ParallelRaceGroup(
                new ShooterPID(shooterSystem, 5000),
                new SequentialCommandGroup(
                        new InstantCommand(() -> System.out.println("ShootNoteSpeaker: In Seq")),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()-> shooterSystem.reachedRPM(RobotMap.TARGET_RPM_SHOOTER) && armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                                new RunCommand(() -> SmartDashboard.putBoolean("ShootNoteSpeakerRpmReached", shooterSystem.reachedRPM(RobotMap.TARGET_RPM_SHOOTER))),
                                new RunCommand(() -> SmartDashboard.putBoolean("ShootNoteSpeakerArmReached", armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE))),
                                new ArmMoveToShooterCommand(armSystem),
                                new InstantCommand(() -> System.out.println("ShootNoteSpeaker: In ParDead"))
                        ),
                        new InstantCommand(() -> System.out.println("ShootNoteSpeaker: After ParDead")),
                        new ParallelRaceGroup(
                                new OuttakeCommand(intakeSystem),
                                Commands.waitSeconds(2)
                        )
                )
        );

        POVButton dPadUp = new POVButton(operatorController, 0);
        POVButton dPadDown = new POVButton(operatorController, 180);

        dPadUp.onTrue(new ArmMoveToShooterCommand(armSystem));
        dPadDown.onTrue(new ArmMoveToFloorCommand(armSystem));

        new JoystickButton(operatorController, XboxController.Button.kX.value).onTrue(shootNoteSpeaker);
        new JoystickButton(operatorController, XboxController.Button.kB.value).whileTrue(new ShooterPID(shooterSystem, 2000));
        new JoystickButton(operatorController, XboxController.Button.kY.value).whileTrue(new OuttakeCommand(intakeSystem));
        new JoystickButton(operatorController, XboxController.Button.kA.value).whileTrue(new IntakeCommand(intakeSystem));
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        DriveTeleopCommand driveTeleopCommand = new DriveTeleopCommand(driveSubsystem, driveController);
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

        SmartDashboard.putBoolean("ShootNote Scheduled", shootNoteSpeaker.isScheduled());
    }

    @Override
    public void simulationPeriodic() {

    }
}
