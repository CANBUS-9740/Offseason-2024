package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveTeleopCommand;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
    private DriveSubsystem driveSubsystem;
    private DriveTeleopCommand driveTeleopCommand;
    private XboxController xboxController;

    @Override
    public void robotInit() {
        driveSubsystem = new DriveSubsystem();
        driveSubsystem.initialize();

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void teleopInit() {
        xboxController = new XboxController(RobotMap.DRIVE_CONTROLLER_PORT);
        driveTeleopCommand = new DriveTeleopCommand(driveSubsystem,xboxController);
    }

    @Override
    public void teleopPeriodic() {
        driveTeleopCommand.schedule();
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void simulationPeriodic() {

    }
}
