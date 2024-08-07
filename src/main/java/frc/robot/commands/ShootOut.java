package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class ShootOut extends Command {
    private final ShooterSystem shooterSystem;

    public ShootOut(ShooterSystem shooterSystem) {
        this.shooterSystem = shooterSystem;

        addRequirements(shooterSystem);
    }


    public void execute() {

        shooterSystem.rotate();

    }


    public void end(boolean interrupted) {
        shooterSystem.stop();
    }


    public boolean isFinished() {
        return false;
    }
}
