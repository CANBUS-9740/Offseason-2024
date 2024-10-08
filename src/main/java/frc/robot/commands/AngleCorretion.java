package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AngleCorretion extends Command {
    private DriveSubsystem driveSubsystem;
    private PIDController pid;
    private double targetAngle;
    private double lastSign = 1;

    public AngleCorretion(DriveSubsystem driveSubsystem, double targetAngle) {
        pid = new PIDController(0.1, 0, 0);
        pid.enableContinuousInput(0, 360);
        this.driveSubsystem = driveSubsystem;
        this.targetAngle = targetAngle;

        addRequirements(driveSubsystem);
    }

    public void initialize() {
        pid.reset();
    }

    public void execute() {
        SmartDashboard.putData("pid", pid);
        double power = pid.calculate(driveSubsystem.getAngleDegrees(), targetAngle);
        System.out.println("Drive angle: " + driveSubsystem.getAngleDegrees());
        System.out.println("Target angle: " + targetAngle);
        System.out.println("PID Power: " + power);
        if (lastSign != Math.signum(power)) {
            lastSign = Math.signum(power);
            System.out.println("Changed Drive angle: " + driveSubsystem.getAngleDegrees());
            System.out.println("Changed Target angle: " + targetAngle);
        }
        driveSubsystem.arcadeDrive(0, power);
    }

    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    public boolean isFinished() {
        return driveSubsystem.didGetToAngleDegrees(targetAngle);
    }


}
