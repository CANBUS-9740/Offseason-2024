package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
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

    private XboxController driveController;
    private XboxController operatorController;

    private Command shootToAmp;
    private Command shootNoteSpeaker;
    private Command collectNote;

    @Override
    public void robotInit() {
        driveSubsystem = new DriveSubsystem();
        shooterSystem = new ShooterSystem();
        intakeSystem = new IntakeSystem();
        armSystem = new ArmSystem();

        driveController = new XboxController(0);
        operatorController = new XboxController(1);

        armSystem.setDefaultCommand(
                new ParallelCommandGroup(
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                                new IntakeSlowlyCommand(intakeSystem)
                        ),
                        new ArmMoveToShooterCommand(armSystem)
                )
        );

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

        collectNote = new ParallelDeadlineGroup(
                new WaitUntilCommand(() -> intakeSystem.isNoteInside()),
                new ParallelDeadlineGroup(
                        new IntakeCommand(intakeSystem),
                        new ArmMoveToFloorCommand(armSystem)
                )
        );

        shootToAmp = new ParallelRaceGroup(
                new ArmMoveToAmp(armSystem),
                new SequentialCommandGroup(
                        new WaitUntilCommand(()-> armSystem.reachedATargetAngle(RobotMap.ARM_AMP_RELEASE_ANGLE)),
                        new ParallelRaceGroup(
                                new OuttakeCommand(intakeSystem),
                                Commands.waitSeconds(1)
                        )
                )
        );

        POVButton dPadUp = new POVButton(operatorController, 0);
        POVButton dPadDown = new POVButton(operatorController, 180);

        dPadUp.onTrue(new ArmMoveToShooterCommand(armSystem));
        dPadDown.onTrue(new ArmMoveToFloorCommand(armSystem));

        new JoystickButton(operatorController, XboxController.Button.kX.value).onTrue(shootNoteSpeaker);
        new JoystickButton(operatorController, XboxController.Button.kY.value).whileTrue(new OuttakeCommand(intakeSystem));
        new JoystickButton(operatorController, XboxController.Button.kA.value).onTrue(collectNote);
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

        SmartDashboard.putBoolean("ShootNoteSpeaker Scheduled", shootNoteSpeaker.isScheduled());
        SmartDashboard.putBoolean("CollectNote Scheduled", collectNote.isScheduled());
        SmartDashboard.putBoolean("ShootNoteAmp Scheduled", shootToAmp.isScheduled());
    }

    @Override
    public void simulationPeriodic() {

    }
}
