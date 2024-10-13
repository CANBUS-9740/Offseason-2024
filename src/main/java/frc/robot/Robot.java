package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.AngleCorretion;
import frc.robot.commands.ArmMoveToAngle;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeSlowlyCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShooterPID;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.utils.ShuffleboardDashboard;

public class Robot extends TimedRobot {

    private DriveSubsystem driveSubsystem;
    private ShooterSystem shooterSystem;
    private IntakeSystem intakeSystem;
    private ArmSystem armSystem;

    private XboxController driveController;
    private XboxController operatorController;

    private Command shooterNoteAmp;
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
                        new ArmMoveToAngle(armSystem, RobotMap.ARM_SHOOTER_ANGLE)
                )
        );

        shootNoteSpeaker = new ParallelRaceGroup(
                new ShooterPID(shooterSystem, 5000),
                new SequentialCommandGroup(
                        new InstantCommand(() -> System.out.println("ShootNoteSpeaker: In Seq")),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> shooterSystem.reachedRPM(RobotMap.TARGET_RPM_SHOOTER) && armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                                new RunCommand(() -> SmartDashboard.putBoolean("ShootNoteSpeakerRpmReached", shooterSystem.reachedRPM(RobotMap.TARGET_RPM_SHOOTER))),
                                new RunCommand(() -> SmartDashboard.putBoolean("ShootNoteSpeakerArmReached", armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE))),
                                new ArmMoveToAngle(armSystem, RobotMap.ARM_SHOOTER_ANGLE),
                                new IntakeSlowlyCommand(intakeSystem),
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
                new IntakeCommand(intakeSystem),
                new ArmMoveToAngle(armSystem, RobotMap.ARM_FLOOR_ANGLE)
        );

        shooterNoteAmp = new ParallelRaceGroup(
                new ArmMoveToAngle(armSystem, RobotMap.ARM_AMP_ANGLE),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> armSystem.reachedATargetAngle(RobotMap.ARM_AMP_ANGLE)),
                        new ParallelRaceGroup(
                                new OuttakeCommand(intakeSystem),
                                Commands.waitSeconds(1)
                        )
                )
        );

        ParallelCommandGroup cancelAllCommands = new ParallelCommandGroup(
                new InstantCommand(() -> armSystem.getCurrentCommand().cancel()),
                new InstantCommand(() -> driveSubsystem.getCurrentCommand().cancel()),
                new InstantCommand(() -> intakeSystem.getCurrentCommand().cancel()),
                new InstantCommand(() -> shooterSystem.getCurrentCommand().cancel())
        );
        POVButton dPadUp = new POVButton(operatorController, 0);
        POVButton dPadDown = new POVButton(operatorController, 180);

        dPadUp.onTrue(new ArmMoveToAngle(armSystem, RobotMap.ARM_SHOOTER_ANGLE));
        dPadDown.onTrue(new ArmMoveToAngle(armSystem, RobotMap.ARM_FLOOR_ANGLE));

        new JoystickButton(operatorController, XboxController.Button.kX.value).onTrue(shootNoteSpeaker);
        new JoystickButton(operatorController, XboxController.Button.kB.value).onTrue(shooterNoteAmp);
        new JoystickButton(operatorController, XboxController.Button.kA.value).onTrue(collectNote);
        new JoystickButton(operatorController, XboxController.Button.kY.value).whileTrue(new OuttakeCommand(intakeSystem));
        new JoystickButton(operatorController, XboxController.Button.kStart.value).whileTrue(cancelAllCommands);

        ShuffleboardDashboard.initialize(armSystem, driveSubsystem, intakeSystem, shooterSystem);


        // todo: REMOVE LATER
        Pose2d robot = new Pose2d(4, 5, Rotation2d.fromDegrees(0));
        Pose2d target = new Pose2d(2, 3, Rotation2d.fromDegrees(0));

        Field2d field2d = new Field2d();
        SmartDashboard.putData("field2f222", field2d);

        driveSubsystem.setRobotPose(robot);
        Info info = driveSubsystem.getTargetInfo(target);
        SmartDashboard.putNumber("distance1", info.getDistance());
        SmartDashboard.putNumber("angle1", info.getAngleDegrees());

        field2d.setRobotPose(new Pose2d(robot.getX(), robot.getY(), Rotation2d.fromDegrees(info.getAngleDegrees())));
        field2d.getObject("target").setPose(target);
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
        new AngleCorretion(driveSubsystem, 90).schedule();
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
        ShuffleboardDashboard.update();

        SmartDashboard.putBoolean("ShootNoteSpeaker Scheduled", shootNoteSpeaker.isScheduled());
        SmartDashboard.putBoolean("CollectNote Scheduled", collectNote.isScheduled());
        SmartDashboard.putBoolean("ShootNoteAmp Scheduled", shooterNoteAmp.isScheduled());
    }

    @Override
    public void simulationPeriodic() {

    }
}
