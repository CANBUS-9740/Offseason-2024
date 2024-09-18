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
    private ParallelCommandGroup cancelAllCommands;
    private ParallelCommandGroup shootNote;

    @Override
    public void robotInit() {
        driveSubsystem = new DriveSubsystem();
        shooterSystem = new ShooterSystem();
        intakeSystem = new IntakeSystem();
        armSystem = new ArmSystem();
        driveController = new XboxController(0);
        operatorController = new XboxController(1);

        JoystickButton Ybutton = new JoystickButton(operatorController, XboxController.Button.kY.value);
        //shooterSystem.getCurrentCommand().cancel();

        ShooterPID shooterCommand = new ShooterPID(shooterSystem, 5000);
        ArmMoveToShooterCommand armCommand = new ArmMoveToShooterCommand(armSystem);

        shootNote = new ParallelCommandGroup(
                shooterCommand,
                new InstantCommand(() -> System.out.println("After shooter")),
                new SequentialCommandGroup(
                        new InstantCommand(() -> System.out.println("In Seq")),
                        new ParallelDeadlineGroup(
                                new WaitUntilCommand(()-> shooterSystem.reachedRPM(5000) && armSystem.reachedATargetAngle(RobotMap.ARM_SHOOTER_ANGLE)),
                                armCommand,
                                new InstantCommand(() -> System.out.println("In ParDead"))
                        ),
                        new InstantCommand(() -> System.out.println("After ParDead")),
                        new OuttakeCommand(intakeSystem),
                        Commands.waitSeconds(2),
                        new InstantCommand(shooterCommand::cancel),
                        new InstantCommand(armCommand::cancel)
                )
        );

        POVButton dPadUp = new POVButton(operatorController, 0);
        POVButton dPadDown = new POVButton(operatorController, 180);

        dPadUp.onTrue(new ArmMoveToShooterCommand(armSystem));
        dPadDown.onTrue(new ArmMoveToFloorCommand(armSystem));

        new JoystickButton(operatorController, XboxController.Button.kX.value).onTrue(shootNote);
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
        ShooterPID shooterPID = new ShooterPID(shooterSystem,5000);
        shooterPID.schedule();

        ShootOut shootOut =  new ShootOut(shooterSystem);
        shootOut.schedule();

        OuttakeCommand outtakeCommand = new OuttakeCommand(intakeSystem);
        outtakeCommand.schedule();

        IntakeCommand intakeCommand = new IntakeCommand(intakeSystem);
        intakeCommand.schedule();

        ArmMoveToShooterCommand armMoveToShooterCommand = new ArmMoveToShooterCommand(armSystem);
        armMoveToShooterCommand.schedule();

        ArmMoveToFloorCommand armMoveToFloorCommand = new ArmMoveToFloorCommand(armSystem);
        armMoveToFloorCommand.schedule();

        DriveTeleopCommand driveTeleopCommand = new DriveTeleopCommand(driveSubsystem, driveController);
        driveTeleopCommand.schedule();

        shootNote.schedule();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putBoolean("ShootNote Scheduled", shootNote.isScheduled());
    }

    @Override
    public void simulationPeriodic() {

    }
}
