package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class ShooterPID extends Command {
    private final ShooterSystem shooterSystem;

    public ShooterPID(ShooterSystem shooterSystem){
        this.shooterSystem = shooterSystem;

        addRequirements(shooterSystem);
    }

    public void initialize() {
        shooterSystem.rotatePID(RobotMap.SHOOTER_TARGET_RPM);
    }

    public void execute() {}

    public void end(boolean interrupted) {
        shooterSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
