package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystemLeft;

public class ClimbStayInPositionLeft extends Command {
    private final ClimbSystemLeft climbSystemLeft;
    private double startHeight;


    public ClimbStayInPositionLeft(ClimbSystemLeft climbSystemLeft){
        this.climbSystemLeft = climbSystemLeft;

        addRequirements(climbSystemLeft);
    }

    @Override
    public void initialize() {
        startHeight = climbSystemLeft.getHeight();
        climbSystemLeft.stayInPositionLeft(startHeight);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        climbSystemLeft.stop();
    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
