package frc.robot.utils;

import edu.wpi.first.wpilibj.shuffleboard.*;

import java.util.Map;

public class ShuffleboardUtils {
    private ShuffleboardUtils() {
        // Required private constructor
    }

    public static ShuffleboardTab getArmIntakeShooterTab() {
        return Shuffleboard.getTab("Arm, Intake & Shooter");
    }

    public static SimpleWidget addDrivetrainWheelSpeedWidget(ShuffleboardContainer container, String title) {
        return container.add(title, 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -5, "max", 5));
    }

    public static SimpleWidget addRobotAngleWidget(ShuffleboardContainer container) {
        return container.add("Robot Angle", 0.0)
                .withWidget(BuiltInWidgets.kGyro)
                .withProperties(Map.of("Starting angle", 90));
    }

    public static SimpleWidget addArmAngleWidget(ShuffleboardContainer container) {
        return container.add("Arm Angle", 0.0)
                .withWidget(BuiltInWidgets.kGyro)
                .withProperties(Map.of("Counter clockwise", true));
    }

    public static SimpleWidget addIntakeMotorSpeedWidget(ShuffleboardContainer container) {
        return container.add("Intake Speed", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", -2000, "max", 2000));
    }

    public static SimpleWidget addShooterSpeedWidget(ShuffleboardContainer container, String title) {
        return container.add(title, 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 2000));
    }

    public static ShuffleboardLayout getArmIntakeShooterSubsystemsLayout() {
        return getArmIntakeShooterTab().getLayout("Subsystems", BuiltInLayouts.kGrid)
                .withPosition(7, 0)
                .withSize(6, 2)
                .withProperties(Map.of("Number of columns", 3, "Number of rows", 1));
    }
}
