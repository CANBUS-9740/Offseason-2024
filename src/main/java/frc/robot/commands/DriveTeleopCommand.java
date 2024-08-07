package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class DriveTeleopCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final XboxController xboxController;
    private final DifferentialDriveOdometry odometry;
    private final Field2d field2d;


    public DriveTeleopCommand(DriveSubsystem driveSubsystem, XboxController xboxController) {
        this.driveSubsystem = driveSubsystem;
        this.xboxController = xboxController;
        this.field2d = new Field2d();
        addRequirements(this.driveSubsystem);
        this.odometry = new DifferentialDriveOdometry(new Rotation2d(this.driveSubsystem.getAngleDegrees()), this.driveSubsystem.getLeftDistancePassedMeters(), this.driveSubsystem.getRightDistancePassedMeters());
    }
    @Override
    public void initialize() {
        driveSubsystem.initialize();
    }

    @Override
    public void execute() {
        field2d.setRobotPose(this.odometry.getPoseMeters().getX(),this.odometry.getPoseMeters().getY(), this.odometry.getPoseMeters().getRotation() );
        driveSubsystem.powerLeftMotors(xboxController.getLeftY());
        driveSubsystem.powerRightMotors(xboxController.getRightY());
        this.odometry.update(new Rotation2d(this.driveSubsystem.getAngleDegrees()), this.driveSubsystem.getLeftDistancePassedMeters(), this.driveSubsystem.getRightDistancePassedMeters());
        SmartDashboard.putNumber("angleOfBot", driveSubsystem.getAngleDegrees());
        SmartDashboard.putNumber("LeftDistance", driveSubsystem.getLeftDistancePassedMeters());
        SmartDashboard.putNumber("RightDistance", driveSubsystem.getRightDistancePassedMeters());
        SmartDashboard.putNumber("X:", this.odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y:", this.odometry.getPoseMeters().getY());
        SmartDashboard.putData("field: ", field2d);
        SmartDashboard.putNumber("Angle:", this.odometry.getPoseMeters().getRotation().getDegrees());

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
