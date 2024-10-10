package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystemLeft;
import frc.robot.subsystems.ClimbSystemRight;

public class ClimbResetRight extends Command {
    private final ClimbSystemRight climbSystemRight;


    public ClimbResetRight(ClimbSystemRight climbSystemRight){
        this.climbSystemRight = climbSystemRight;

        addRequirements(climbSystemRight);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climbSystemRight.move(-RobotMap.CLIMB_MOVE_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        climbSystemRight.resetRight();
        climbSystemRight.stop();
    }

    @Override
    public boolean isFinished() {
        return climbSystemRight.isRightClimberDown();

    }
}
