package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystemLeft;
import frc.robot.subsystems.ClimbSystemRight;

public class ClimbResetLeft extends Command {
    private final ClimbSystemLeft climbSystemLeft;


    public ClimbResetLeft(ClimbSystemLeft climbSystemLeft){
        this.climbSystemLeft = climbSystemLeft;

        addRequirements(climbSystemLeft);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climbSystemLeft.move(-RobotMap.CLIMB_MOVE_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        climbSystemLeft.resetLeft();
        climbSystemLeft.stop();
    }

    @Override
    public boolean isFinished() {
        return climbSystemLeft.isLeftClimberDown();

    }
}
