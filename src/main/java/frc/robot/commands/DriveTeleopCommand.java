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
    private final Field2d field2d;


    public DriveTeleopCommand(DriveSubsystem driveSubsystem, XboxController xboxController) {
        this.driveSubsystem = driveSubsystem;
        this.xboxController = xboxController;
        this.field2d = new Field2d();
        addRequirements(this.driveSubsystem);
    }
    @Override
    public void initialize() {
        driveSubsystem.initialize();
    }

    @Override
    public void execute() {
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
