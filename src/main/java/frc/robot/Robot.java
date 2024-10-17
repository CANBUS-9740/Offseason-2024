package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoCamera;
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
        new JoystickButton(operatorController, XboxController.Button.kB.value).whileTrue(new OuttakeCommand(intakeSystem));
        new JoystickButton(operatorController, XboxController.Button.kStart.value).onTrue(cancelAllCommands);

        setUpCameras();
        ShuffleboardDashboard.setRobotDataSupplier(() -> new ShuffleboardDashboard.RobotData(RobotController.getBatteryVoltage()));
        ShuffleboardDashboard.initialize(frontCamera, backCamera);
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
        driveSubsystem.setDefaultCommand(null);
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
}
