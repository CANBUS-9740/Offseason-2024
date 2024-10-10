package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystemRight;

public class ClimbDownRight extends Command {
    private final ClimbSystemRight climbSystemRight;

    public ClimbDownRight(ClimbSystemRight climbSystemRight){
        this.climbSystemRight = climbSystemRight;

        addRequirements(climbSystemRight);

    }
    public void initialize() {
    }

    public void execute() {
        climbSystemRight.move(-RobotMap.CLIMB_MOVE_POWER);
    }

    public void end(boolean interrupted) {
        climbSystemRight.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
