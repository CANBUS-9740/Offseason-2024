package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveSubsystem;


public class TurnToAngleCommand extends Command {
    private final Rotation2d rotation2d;

    private final PIDController pidController = new PIDController(RobotMap.ROTATION_P, RobotMap.ROTATION_I, RobotMap.ROTATION_D);

    private final DriveSubsystem driveSubsystem;

    public TurnToAngleCommand(Rotation2d rotation2d, boolean flip, DriveSubsystem driveSubsystem) {
        final var shouldFlip = flip && DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent();
        this.rotation2d = shouldFlip ? new Rotation2d(-rotation2d.getCos(), rotation2d.getSin()) : rotation2d;
        this.driveSubsystem = driveSubsystem;

        pidController.setTolerance(RobotMap.ROTATION_POSITION_TOLERANCE, RobotMap.ROTATION_VELOCITY_TOLERANCE);
        pidController.enableContinuousInput(-180.0, 180.0);
        pidController.setSetpoint(rotation2d.getDegrees());

        SmartDashboard.putData("RotationPIDController", pidController);

        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Turning to " + rotation2d.getDegrees() + " degrees.");
        pidController.reset();
    }

    @Override
    public void execute() {
        final var output = MathUtil.clamp(pidController.calculate(driveSubsystem.getRobotPose().getRotation().getDegrees()), -1.0, 1.0);
        SmartDashboard.putNumber("TurnToAngleOutput", output);

        driveSubsystem.arcadeDrive(0.0, output);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("TurnToAngleOutput", 0);
        driveSubsystem.stop();
    }
}
