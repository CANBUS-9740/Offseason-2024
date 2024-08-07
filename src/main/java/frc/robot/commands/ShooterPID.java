package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class ShooterPID extends Command {
    private ShooterSystem shooterSystem;
    private final double targetRPM;
    public ShooterPID(ShooterSystem shooterSystem, double targetRPM){
        this.shooterSystem = shooterSystem;
        this.targetRPM = targetRPM;
    }
    public void initialize() {
        shooterSystem.rotatePID();
    }

    public void execute() {}

    public void end(boolean interrupted) {
        shooterSystem.stop();
    }


    public boolean isFinished() {
        return false;
    }
}
