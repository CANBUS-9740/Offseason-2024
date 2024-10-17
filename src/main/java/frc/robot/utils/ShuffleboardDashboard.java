package frc.robot.utils;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.util.Map;
import java.util.function.Supplier;

public class ShuffleboardDashboard {

    private ShuffleboardDashboard() {
        // Required private constructor
    }

    private static final Field2d field2d = new Field2d();

    private static GenericEntry shooterTopLeftSpeedEntry;
    private static GenericEntry shooterTopRightSpeedEntry;
    private static GenericEntry shooterBottomLeftSpeedEntry;
    private static GenericEntry shooterBottomRightSpeedEntry;
    private static GenericEntry shooterAtSpeed;
    private static GenericEntry intakeMotorSpeedEntry;
    private static GenericEntry robotVoltageEntry;
    private static GenericEntry noteInsideEntry;

    private static Supplier<DrivetrainData> drivetrainDataSupplier;
    private static Supplier<ArmData> armDataSupplier;
    private static Supplier<IntakeData> intakeDataSupplier;
    private static Supplier<ShooterData> shooterDataSupplier;
    private static Supplier<RobotData> robotDataSupplier;

    public static void setDrivetrainDataSupplier(Supplier<DrivetrainData> drivetrainDataSupplier) {
        ShuffleboardDashboard.drivetrainDataSupplier = drivetrainDataSupplier;
    }

    public static void setArmDataSupplier(Supplier<ArmData> armDataSupplier) {
        ShuffleboardDashboard.armDataSupplier = armDataSupplier;
    }

    public static void setIntakeDataSupplier(Supplier<IntakeData> intakeDataSupplier) {
        ShuffleboardDashboard.intakeDataSupplier = intakeDataSupplier;
    }

    public static void setShooterDataSupplier(Supplier<ShooterData> shooterDataSupplier) {
        ShuffleboardDashboard.shooterDataSupplier = shooterDataSupplier;
    }

    public static void setRobotDataSupplier(Supplier<RobotData> robotDataSupplier) {
        ShuffleboardDashboard.robotDataSupplier = robotDataSupplier;
    }

    public static void initialize(VideoSource frontCamera, VideoSource backCamera) {
        final var tab = Shuffleboard.getTab("Dashboard");

        ShuffleboardLayout shooterSpeedsLayout = tab.getLayout("Shooter Speeds", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 2))
                .withPosition(0, 0)
                .withSize(3, 3);

        shooterTopLeftSpeedEntry = ShuffleboardUtils.addShooterSpeedWidget(shooterSpeedsLayout, "Top Left")
                .withPosition(0, 0)
                .getEntry();
        shooterTopRightSpeedEntry = ShuffleboardUtils.addShooterSpeedWidget(shooterSpeedsLayout, "Top Right")
                .withPosition(1, 0)
                .getEntry();
        shooterBottomLeftSpeedEntry = ShuffleboardUtils.addShooterSpeedWidget(shooterSpeedsLayout, "Bottom Left")
                .withPosition(0, 1)
                .getEntry();
        shooterBottomRightSpeedEntry = ShuffleboardUtils.addShooterSpeedWidget(shooterSpeedsLayout, "Bottom Right")
                .withPosition(1, 1)
                .getEntry();

        shooterAtSpeed = tab.add("Shooter At Speed", false)
                .withPosition(3, 0)
                .withSize(1, 3)
                .getEntry();

        tab.add("Field", field2d)
                .withPosition(0, 3)
                .withSize(4, 3);

        tab.add("Front Camera", frontCamera)
                .withPosition(4, 0)
                .withSize(5, 4)
                .withProperties(Map.of("Show crosshair", false, "Show controls", false));
        tab.add("Back Camera", backCamera)
                .withPosition(9, 0)
                .withSize(4, 4)
                .withProperties(Map.of("Show crosshair", false, "Show controls", false));

        noteInsideEntry = tab.add("Note Inside", false)
                .withPosition(4, 4)
                .withSize(9, 1)
                .getEntry();

        intakeMotorSpeedEntry = ShuffleboardUtils.addIntakeMotorSpeedWidget(tab)
                .withPosition(9, 5)
                .withSize(4, 1)
                .getEntry();

        robotVoltageEntry = tab.add("Robot Voltage", 0.0)
                .withWidget(BuiltInWidgets.kVoltageView)
                .withPosition(4, 5)
                .withSize(5, 1)
                .withProperties(Map.of("Min", 8, "Max", 13, "Number of tick marks", 11))
                .getEntry();
    }

    public static void update() {
        if (drivetrainDataSupplier != null)
            updateDrivetrainEntries();
        if (armDataSupplier != null)
            updateArmEntries();
        if (intakeDataSupplier != null)
            updateIntakeEntries();
        if (shooterDataSupplier != null)
            updateShooterEntries();
        if (robotDataSupplier != null)
            updateRobotEntries();
    }

    private static void updateDrivetrainEntries() {
        final var drivetrainData = drivetrainDataSupplier.get();

        field2d.setRobotPose(drivetrainData.robotPose);
    }

    @SuppressWarnings("unused")
    private static void updateArmEntries() {
        final var armData = armDataSupplier.get();
    }

    private static void updateIntakeEntries() {
        final var intakeData = intakeDataSupplier.get();

        noteInsideEntry.setBoolean(intakeData.noteInside);
        intakeMotorSpeedEntry.setDouble(intakeData.intakeMotorSpeedRpm);
    }

    private static void updateShooterEntries() {
        final var shooterData = shooterDataSupplier.get();

        shooterTopLeftSpeedEntry.setDouble(shooterData.topLeftSpeedRpm);
        shooterTopRightSpeedEntry.setDouble(shooterData.topRightSpeedRpm);
        shooterBottomLeftSpeedEntry.setDouble(shooterData.bottomLeftSpeedRpm);
        shooterBottomRightSpeedEntry.setDouble(shooterData.bottomRightSpeedRpm);
        shooterAtSpeed.setBoolean(shooterData.isAtSpeed);
    }

    private static void updateRobotEntries() {
        final var robotData = robotDataSupplier.get();

        robotVoltageEntry.setDouble(robotData.voltage);
    }

    public static class DrivetrainData {
        public final Pose2d robotPose;
        public final DifferentialDriveWheelSpeeds wheelSpeeds;

        public DrivetrainData(Pose2d robotPose, DifferentialDriveWheelSpeeds wheelSpeeds) {
            this.robotPose = robotPose;
            this.wheelSpeeds = wheelSpeeds;
        }
    }

    public static class ArmData {
        public final double armAngleDegrees;

        public ArmData(double armAngleDegrees) {
            this.armAngleDegrees = armAngleDegrees;
        }
    }

    public static class IntakeData {
        public final boolean noteInside;
        public final double intakeMotorSpeedRpm;

        public IntakeData(boolean noteInside, double intakeMotorSpeedRpm) {
            this.noteInside = noteInside;
            this.intakeMotorSpeedRpm = intakeMotorSpeedRpm;
        }
    }

    public static class ShooterData {
        public final double topLeftSpeedRpm;
        public final double topRightSpeedRpm;
        public final double bottomLeftSpeedRpm;
        public final double bottomRightSpeedRpm;
        public final boolean isAtSpeed;

        public ShooterData(double topLeftSpeedRpm, double topRightSpeedRpm, double bottomLeftSpeedRpm, double bottomRightSpeedRpm, boolean isAtSpeed) {
            this.topLeftSpeedRpm = topLeftSpeedRpm;
            this.topRightSpeedRpm = topRightSpeedRpm;
            this.bottomLeftSpeedRpm = bottomLeftSpeedRpm;
            this.bottomRightSpeedRpm = bottomRightSpeedRpm;
            this.isAtSpeed = isAtSpeed;
        }
    }

    public static class RobotData {
        public final double voltage;

        public RobotData(double voltage) {
            this.voltage = voltage;
        }
    }
}
