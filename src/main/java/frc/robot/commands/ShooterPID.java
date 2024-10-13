package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class ShooterPID extends Command {
    private final ShooterSystem shooterSystem;
    private final double targetRPM;

    public ShooterPID(ShooterSystem shooterSystem, double targetRPM){
        this.shooterSystem = shooterSystem;
        this.targetRPM = targetRPM;
        addRequirements(shooterSystem);
    }

    public void initialize() {
        shooterSystem.rotatePID(targetRPM);
    }

    public void execute() {

    }

    public void end(boolean interrupted) {
        shooterSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
