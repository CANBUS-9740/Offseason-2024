package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystemLeft;
import frc.robot.subsystems.ClimbSystemRight;

public class ClimbStayInPositionRight extends Command {
    private final ClimbSystemRight climbSystemRight;
    private double startHeight;


    public ClimbStayInPositionRight(ClimbSystemRight climbSystemRight){
        this.climbSystemRight = climbSystemRight;

        addRequirements(climbSystemRight);
    }

    @Override
    public void initialize() {
        startHeight = climbSystemRight.getHeight();
        climbSystemRight.stayInPositionRight(startHeight);

    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        climbSystemRight.stop();
    }

    @Override
    public boolean isFinished() {
        return false ;

    }
}
