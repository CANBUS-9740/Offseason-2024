package frc.robot.utils;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class ShuffleboardAutoTab {
    private static SendableChooser<Command> autoChooser;

    private ShuffleboardAutoTab() {
        // Required private constructor
    }

    public static void initialize() {
        autoChooser = AutoBuilder.buildAutoChooser();

        final var tab = Shuffleboard.getTab("Auto");

        tab.add("Auto Chooser", autoChooser)
                .withPosition(0, 0)
                .withSize(4, 4);
    }

    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
