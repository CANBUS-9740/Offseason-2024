package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class ShooterTeleop extends Command {
    private final ShooterSystem shooterSystem;
    private final XboxController xboxController;


    public ShooterTeleop(ShooterSystem shooterSystem, XboxController xboxController){
        this.shooterSystem = shooterSystem;
        this.xboxController = xboxController;

        addRequirements();
    }

    public void initialize() {}


    public void execute() {
        if(xboxController.getRightY() < 0) {
            shooterSystem.rotate();
        }
    }


    public void end(boolean interrupted) {
        shooterSystem.stop();
    }


    public boolean isFinished() {
        return false;
    }
}
