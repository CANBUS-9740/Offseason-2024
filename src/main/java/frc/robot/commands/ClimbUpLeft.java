package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystemLeft;

public class ClimbUpLeft extends Command {
    private final ClimbSystemLeft climbSystemLeft;

    public ClimbUpLeft(ClimbSystemLeft climbSystemLeft){
        this.climbSystemLeft = climbSystemLeft;

        addRequirements(climbSystemLeft);
    }
    public void initialize() {
    }

    public void execute() {
        climbSystemLeft.move(RobotMap.CLIMB_MOVE_POWER);
    }

    public void end(boolean interrupted) {
        climbSystemLeft.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
