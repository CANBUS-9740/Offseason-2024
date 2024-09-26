package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

public class ShuffleboardUtils {
    private ShuffleboardUtils() {
        // Required private constructor
    }

    public static ShuffleboardTab getArmIntakeShooterTab() {
        return Shuffleboard.getTab("Arm, Intake & Shooter");
    }

    public static ShuffleboardLayout getArmIntakeShooterSubsystemsLayout() {
        return getArmIntakeShooterTab().getLayout("Subsystems", BuiltInLayouts.kGrid)
                .withPosition(4, 0)
                .withSize(4, 2)
                .withProperties(Map.of("Number of columns", 3, "Number of rows", 1));
    }
}
