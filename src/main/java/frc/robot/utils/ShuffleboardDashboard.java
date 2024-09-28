package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

import java.util.Map;
import java.util.function.Supplier;

public class ShuffleboardDashboard {
    private ShuffleboardDashboard() {
        // Required private constructor
    }

    private static final Field2d field2d = new Field2d();

    private static GenericEntry robotAngleEntry;
    private static GenericEntry leftWheelSpeedEntry;
    private static GenericEntry rightWheelSpeedEntry;
    private static GenericEntry armAngleEntry;
    private static GenericEntry noteInsideEntry;
    private static GenericEntry intakeMotorSpeedEntry;
    private static GenericEntry shooterTopLeftSpeedEntry;
    private static GenericEntry shooterTopRightSpeedEntry;
    private static GenericEntry shooterBottomLeftSpeedEntry;
    private static GenericEntry shooterBottomRightSpeedEntry;

    private static Supplier<DrivetrainData> drivetrainDataSupplier;
    private static Supplier<ArmData> armDataSupplier;
    private static Supplier<IntakeData> intakeDataSupplier;
    private static Supplier<ShooterData> shooterDataSupplier;

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

    public static void initialize(ArmSystem armSystem, DriveSubsystem driveSubsystem, IntakeSystem intakeSystem, ShooterSystem shooterSystem) {
        final var tab = Shuffleboard.getTab("Dashboard");

        tab.add("Field", field2d)
                .withPosition(0, 0)
                .withSize(7, 4);

        robotAngleEntry = ShuffleboardUtils.addRobotAngleWidget(tab)
                .withPosition(0, 4)
                .withSize(2, 2)
                .getEntry();

        ShuffleboardLayout wheelSpeedsLayout = tab.getLayout("Wheel Speeds", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 1))
                .withPosition(2, 4)
                .withSize(5, 2);

        leftWheelSpeedEntry = ShuffleboardUtils.addDrivetrainWheelSpeedWidget(wheelSpeedsLayout, "Left Wheel")
                .withPosition(0, 0)
                .getEntry();
        rightWheelSpeedEntry = ShuffleboardUtils.addDrivetrainWheelSpeedWidget(wheelSpeedsLayout, "Right Wheel")
                .withPosition(1, 0)
                .getEntry();

        ShuffleboardLayout subsystemsLayout = tab.getLayout("Subsystem States", BuiltInLayouts.kList)
                .withProperties(Map.of("Label position", "TOP"))
                .withPosition(7, 0)
                .withSize(2, 6);

        subsystemsLayout.add("Arm Subsystem", armSystem);
        subsystemsLayout.add("Drivetrain Subsystem", driveSubsystem);
        subsystemsLayout.add("Intake Subsystem", intakeSystem);
        subsystemsLayout.add("Shooter Subsystem", shooterSystem);

        armAngleEntry = ShuffleboardUtils.addArmAngleWidget(tab)
                .withPosition(9, 0)
                .withSize(2, 2)
                .getEntry();

        noteInsideEntry = tab.add("Note Inside", false)
                .withPosition(11, 0)
                .withSize(2, 2)
                .getEntry();

        intakeMotorSpeedEntry = ShuffleboardUtils.addIntakeMotorSpeedWidget(tab)
                .withPosition(9, 2)
                .withSize(4, 1)
                .getEntry();

        ShuffleboardLayout shooterSpeedsLayout = tab.getLayout("Shooter Speeds", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 2))
                .withPosition(9, 3)
                .withSize(4, 3);

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
    }

    private static void updateDrivetrainEntries() {
        final var drivetrainData = drivetrainDataSupplier.get();

        field2d.setRobotPose(drivetrainData.robotPose);
        robotAngleEntry.setDouble(drivetrainData.robotPose.getRotation().getDegrees());
        leftWheelSpeedEntry.setDouble(drivetrainData.wheelSpeeds.leftMetersPerSecond);
        rightWheelSpeedEntry.setDouble(drivetrainData.wheelSpeeds.rightMetersPerSecond);
    }

    private static void updateArmEntries() {
        final var armData = armDataSupplier.get();

        armAngleEntry.setDouble(armData.armAngleDegrees);
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

        public ShooterData(double topLeftSpeedRpm, double topRightSpeedRpm, double bottomLeftSpeedRpm, double bottomRightSpeedRpm) {
            this.topLeftSpeedRpm = topLeftSpeedRpm;
            this.topRightSpeedRpm = topRightSpeedRpm;
            this.bottomLeftSpeedRpm = bottomLeftSpeedRpm;
            this.bottomRightSpeedRpm = bottomRightSpeedRpm;
        }
    }
}
