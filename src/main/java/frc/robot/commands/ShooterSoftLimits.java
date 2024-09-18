package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class ShooterSoftLimits extends Command {
    private final ShooterSystem shooterSystem;
    private int stallLimit;
    private int freeLimit;
    private int limitRPM;

    public ShooterSoftLimits(ShooterSystem shooterSystem, int stallLimit, int freeLimit, int limitRPM){
        this.shooterSystem = shooterSystem;
        this.stallLimit = stallLimit;
        this.freeLimit = freeLimit;
        this.limitRPM = limitRPM;

        addRequirements(shooterSystem);
    }
    public void initialize() {
        shooterSystem.setNewSoftLimits(stallLimit, stallLimit, limitRPM);
        shooterSystem.rotatePID(RobotMap.SHOOTER_TARGET_RPM);
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
