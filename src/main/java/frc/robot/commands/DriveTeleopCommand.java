package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class DriveTeleopCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final XboxController xboxController;

    public DriveTeleopCommand(DriveSubsystem driveSubsystem, XboxController xboxController) {
        this.driveSubsystem = driveSubsystem;
        this.xboxController = xboxController;
        addRequirements(this.driveSubsystem);
    }
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        driveSubsystem.powerLeftMotors(xboxController.getLeftY());
        driveSubsystem.powerRightMotors(xboxController.getRightY());
        SmartDashboard.putNumber("angleOfBot", driveSubsystem.getPigeonAngle());
        SmartDashboard.putNumber("LeftDistance", driveSubsystem.getHowMuchLeftMoved());
        SmartDashboard.putNumber("RightDistance", driveSubsystem.getHowMuchRightMoved());
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
