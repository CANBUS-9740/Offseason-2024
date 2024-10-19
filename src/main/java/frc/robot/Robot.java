package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.utils.KeyPositions;
import frc.robot.utils.ShuffleboardAutoTab;
import frc.robot.utils.ShuffleboardDashboard;

public class Robot extends TimedRobot {

    private DriveSubsystem driveSubsystem;
    private ShooterSystem shooterSystem;
    private IntakeSystem intakeSystem;
    private ArmSystem armSystem;

    private VideoCamera frontCamera;
    private VideoCamera backCamera;

    private XboxController driveController;
    private XboxController operatorController;

    private Command shooterNoteAmp;
    private Command shootNoteSpeaker;
    private Command shootNoteSlow;
    private Command collectNote;

    @Override
    public void robotInit() {
        driveSubsystem = new DriveSubsystem();
        shooterSystem = new ShooterSystem();
        intakeSystem = new IntakeSystem();
        armSystem = new ArmSystem();

        driveController = new XboxController(0);
        operatorController = new XboxController(1);

        armSystem.setDefaultCommand(new ParallelCommandGroup(
                new ParallelDeadlineGroup(
                        new WaitUntilCommand(() -> armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                        new IntakeSlowlyCommand(intakeSystem)
                ),
                new ArmMoveToAngle(armSystem, RobotMap.ARM_SHOOTER_ANGLE)
        ));

        shootNoteSpeaker = new ParallelRaceGroup(
                new ShooterPID(shooterSystem, RobotMap.TARGET_RPM_SHOOTER_FAST),
                new SequentialCommandGroup(
                        new InstantCommand(() -> System.out.println("ShootNoteSpeaker: In Seq")),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> shooterSystem.reachedRPM(RobotMap.TARGET_RPM_SHOOTER_FAST) && armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                                new RunCommand(() -> SmartDashboard.putBoolean("ShootNoteSpeakerRpmReached", shooterSystem.reachedRPM(RobotMap.TARGET_RPM_SHOOTER_FAST))),
                                new RunCommand(() -> SmartDashboard.putBoolean("ShootNoteSpeakerArmReached", armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE))),
                                new ArmMoveToAngle(armSystem, RobotMap.ARM_SHOOTER_ANGLE),
                                new IntakeSlowlyCommand(intakeSystem),
                                new InstantCommand(() -> System.out.println("ShootNoteSpeaker: In ParDead"))
                        ),
                        new InstantCommand(() -> System.out.println("ShootNoteSpeaker: After ParDead")),
                        new ParallelRaceGroup(
                                new OuttakeCommand(intakeSystem),
                                Commands.waitSeconds(1)
                        )
                )
        );

        shootNoteSlow = new ParallelRaceGroup(
                new ShooterPID(shooterSystem, RobotMap.TARGET_RPM_SHOOTER_SLOW),
                new SequentialCommandGroup(
                        new InstantCommand(() -> System.out.println("ShootNoteSpeaker: In Seq")),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(() -> shooterSystem.reachedRPM(RobotMap.TARGET_RPM_SHOOTER_SLOW) && armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                                new RunCommand(() -> SmartDashboard.putBoolean("ShootNoteSpeakerRpmReached", shooterSystem.reachedRPM(RobotMap.TARGET_RPM_SHOOTER_SLOW))),
                                new RunCommand(() -> SmartDashboard.putBoolean("ShootNoteSpeakerArmReached", armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE))),
                                new ArmMoveToAngle(armSystem, RobotMap.ARM_SHOOTER_ANGLE),
                                new IntakeSlowlyCommand(intakeSystem),
                                new InstantCommand(() -> System.out.println("ShootNoteSpeaker: In ParDead"))
                        ),
                        new InstantCommand(() -> System.out.println("ShootNoteSpeaker: After ParDead")),
                        new ParallelRaceGroup(
                                new OuttakeCommand(intakeSystem),
                                Commands.waitSeconds(1)
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

        Command cancelAllCommands = Commands.runOnce(() -> {
        }, driveSubsystem, armSystem, intakeSystem, shooterSystem);

        new POVButton(operatorController, 0).onTrue(new ArmMoveToAngle(armSystem, RobotMap.ARM_SHOOTER_ANGLE));
        new POVButton(operatorController, 180).onTrue(new ArmMoveToAngle(armSystem, RobotMap.ARM_FLOOR_ANGLE));
        new JoystickButton(operatorController, XboxController.Button.kRightBumper.value).onTrue(shootNoteSpeaker);
        new JoystickButton(operatorController, XboxController.Button.kX.value).onTrue(shootNoteSlow);
        new JoystickButton(operatorController, XboxController.Button.kY.value).onTrue(shooterNoteAmp);
        new JoystickButton(operatorController, XboxController.Button.kA.value).onTrue(collectNote);
        new JoystickButton(operatorController, XboxController.Button.kY.value).whileTrue(new OuttakeCommand(intakeSystem));
        new JoystickButton(operatorController, XboxController.Button.kStart.value).onTrue(cancelAllCommands);

        setUpCameras();

        registerNamedCommands();

        ShuffleboardDashboard.setRobotDataSupplier(() -> new ShuffleboardDashboard.RobotData(RobotController.getBatteryVoltage()));
        ShuffleboardDashboard.initialize(frontCamera, backCamera);

        ShuffleboardAutoTab.initialize();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void teleopInit() {
        driveSubsystem.setDefaultCommand(new DriveTeleopCommand(driveSubsystem, driveController));
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        driveSubsystem.removeDefaultCommand();
    }

    @Override
    public void autonomousInit() {
        ShuffleboardAutoTab.getAutonomousCommand().schedule();
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

    private void enumerateVideoModes(VideoCamera camera) {
        for (final var videoMode : camera.enumerateVideoModes()) {
            System.out.println("===== VIDEO MODE: " + videoMode.width + "x" + videoMode.height + " " + videoMode.fps + " " + videoMode.pixelFormat.toString());
        }
    }

    private void setUpCameras() {
        frontCamera = CameraServer.startAutomaticCapture("Front Camera", 0);
        backCamera = new HttpCamera("Back Camera", "http://10.97.40.11:5800", HttpCamera.HttpCameraKind.kMJPGStreamer); // limelight stream
        frontCamera.setVideoMode(PixelFormat.kMJPEG, 160, 120, 15);
        backCamera.setVideoMode(PixelFormat.kMJPEG, 640, 480, 30);
        System.out.println("========== FRONT CAMERA");
        enumerateVideoModes(frontCamera);
        System.out.println("========== BACK CAMERA");
        enumerateVideoModes(backCamera);
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("PickUpNote", Commands.sequence(
                Commands.race(
                        Commands.waitSeconds(4.5),
                        Commands.deadline(
                                new IntakeCommand(intakeSystem),
                                new ArmMoveToAngle(armSystem, RobotMap.ARM_FLOOR_ANGLE),
                                new DriveSpeedCommand(0.5, driveSubsystem)
                        )
                ),
                Commands.deadline(
                        Commands.waitUntil(() -> armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                        new ArmMoveToAngle(armSystem, RobotMap.ARM_SHOOTER_ANGLE)
                )
        ));
        NamedCommands.registerCommand("ShootNote", shootNoteSpeaker);
        NamedCommands.registerCommand("MoveForwardABit", Commands.deadline(
                Commands.waitSeconds(1.5),
                new DriveSpeedCommand(0.25, driveSubsystem)
        ));
        NamedCommands.registerCommand("MoveBackABit", Commands.deadline(
                Commands.waitSeconds(1.5),
                new DriveSpeedCommand(-0.25, driveSubsystem)
        ));
        NamedCommands.registerCommand("MoveForwardABunch", Commands.deadline(
                Commands.waitSeconds(2.0),
                new DriveSpeedCommand(0.5, driveSubsystem)
        ));
        NamedCommands.registerCommand("MoveBackABunch", Commands.deadline(
                Commands.waitSeconds(2.0),
                new DriveSpeedCommand(-0.5, driveSubsystem)
        ));
        NamedCommands.registerCommand("TurnTo0", Commands.runOnce(() -> {
            new TurnToAngleCommand(Rotation2d.fromDegrees(0), true, driveSubsystem).schedule();
        }));
        NamedCommands.registerCommand("TurnTo180", Commands.runOnce(() -> {
            new TurnToAngleCommand(Rotation2d.fromDegrees(180), true, driveSubsystem).schedule();
        }));
        NamedCommands.registerCommand("TurnToFourthCenterNote", Commands.runOnce(() -> {
            new TurnToAngleCommand(driveSubsystem.getRobotPose().getTranslation().minus(KeyPositions.getFourthCenterNote().getTranslation()).getAngle(), true, driveSubsystem).schedule();
        }));
        NamedCommands.registerCommand("PathfindToLocalTopNote", Commands.runOnce(() -> {
            driveSubsystem.pathfindTo(KeyPositions.getLocalTopNote()).schedule();
        }));
        NamedCommands.registerCommand("PathfindToLocalMiddleNote", Commands.runOnce(() -> {
            driveSubsystem.pathfindTo(KeyPositions.getLocalMiddleNote()).schedule();
        }));
        NamedCommands.registerCommand("PathfindToLocalTopThird", Commands.runOnce(() -> {
            driveSubsystem.pathfindTo(KeyPositions.getLocalTopThird()).schedule();
        }));
        NamedCommands.registerCommand("PathfindToLocalBottomThird", Commands.runOnce(() -> {
            driveSubsystem.pathfindTo(KeyPositions.getLocalBottomThird()).schedule();
        }));
        NamedCommands.registerCommand("PathfindToHalfLocalBottomThirdHalfFourthCenterNote", Commands.runOnce(() -> {
            driveSubsystem.pathfindTo(new Pose2d(KeyPositions.getLocalBottomThird().getTranslation().plus(KeyPositions.getFourthCenterNote().getTranslation()).div(2), new Rotation2d())).schedule();
        }));
    }
}
