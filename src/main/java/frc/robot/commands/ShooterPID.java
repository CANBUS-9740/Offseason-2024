package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class ShooterPID extends Command {
    private final ShooterSystem shooterSystem;
    private final double targetRPMRight;
    private final double targetRPMLeft;

    public ShooterPID(ShooterSystem shooterSystem, double targetRPMRight, double targetRPMLeft){
        this.shooterSystem = shooterSystem;
        this.targetRPMRight = targetRPMRight;
        this.targetRPMLeft = targetRPMLeft;

        addRequirements(shooterSystem);
    }

    public void initialize() {
        shooterSystem.rotatePID(targetRPMRight, targetRPMLeft);
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
