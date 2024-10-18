package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class DriveSpeedCommand extends Command {
    private final double metersPerSecond;

    private final DriveSubsystem driveSubsystem;

    public DriveSpeedCommand(double metersPerSecond, DriveSubsystem driveSubsystem) {
        this.metersPerSecond = metersPerSecond;
        this.driveSubsystem = driveSubsystem;

        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.driveSpeedLeft(metersPerSecond);
        driveSubsystem.driveSpeedRight(metersPerSecond);
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
