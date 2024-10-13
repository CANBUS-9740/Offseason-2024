package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.ShuffleboardDashboard;
import frc.robot.utils.ShuffleboardUtils;

import java.util.Map;

public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonSRX rightMaster;
    private final WPI_VictorSPX leftFollower;
    private final WPI_VictorSPX rightFollower;
    private final WPI_TalonSRX leftMaster;
    private final Field2d field2d;
    private final DifferentialDriveOdometry differentialDriveOdometry;
    private final DifferentialDrive differentialDrive;
    private final Pigeon2 pigeon2;

    // Shuffleboard

    private GenericEntry xEntry;
    private GenericEntry yEntry;
    private GenericEntry angleEntry;
    private GenericEntry leftSpeedEntry;
    private GenericEntry rightSpeedEntry;

    public DriveSubsystem() {
        rightFollower = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_BACK_MOTOR_ID);
        leftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_BACK_MOTOR_ID);
        rightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FRONT_MOTOR_ID);
        leftFollower = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_FRONT_MOTOR_ID);
        pigeon2 = new Pigeon2(RobotMap.PIGEON_ID);
        rightFollower.configFactoryDefault();
        leftMaster.configFactoryDefault();
        rightMaster.configFactoryDefault();
        leftFollower.configFactoryDefault();
        pigeon2.getConfigurator().apply(new Pigeon2Configuration());

        rightMaster.setInverted(true);
        rightFollower.setInverted(true);
        leftMaster.setSensorPhase(true);

        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);

        rightFollower.follow(rightMaster);
        leftFollower.follow(leftMaster);

        this.field2d = new Field2d();
        SmartDashboard.putData("Field" ,field2d);

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

        differentialDriveOdometry = new DifferentialDriveOdometry(
                Rotation2d.fromDegrees(getAngleDegrees()),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters()
        );

        initialize();
        setUpShuffleboard();

        ShuffleboardDashboard.setDrivetrainDataSupplier(() -> new ShuffleboardDashboard.DrivetrainData(
                differentialDriveOdometry.getPoseMeters(),
                new DifferentialDriveWheelSpeeds(getLeftSpeedMetersPerSecond(), getRightSpeedMetersPerSecond())
        ));
    }

    public Field2d getField2d() {
        return field2d;
    }

    public double getLeftDistancePassedMeters() {
        return leftMaster.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getRightDistancePassedMeters() {
        return rightMaster.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getLeftSpeedMetersPerSecond() {
        return leftMaster.getSelectedSensorVelocity() / RobotMap.TALON_ENCODER_PPR / RobotMap.TALON_ENCODER_TIMEFRAME_SECONDS * RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getRightSpeedMetersPerSecond() {
        return rightMaster.getSelectedSensorVelocity() / RobotMap.TALON_ENCODER_PPR / RobotMap.TALON_ENCODER_TIMEFRAME_SECONDS * RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_METERS;
    }

    public double getAngleDegrees() {
        return (360 - pigeon2.getAngle()) % 360;// the value returned will be from 0 - 360 depending on its location
        // 0  - degree after initializing, 359 - one degree to the right, 1 - one degree to the left// 90 will be 90 degrees to the left
    }

    public void arcadeDrive(double linearSpeed, double rotationSpeed){
        differentialDrive.arcadeDrive(linearSpeed, rotationSpeed);
    }

    public void stop() {
        rightFollower.stopMotor();
        leftFollower.stopMotor();
    }

    private void setUpShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        tab.add("Field", field2d)
                .withPosition(0, 0)
                .withSize(10, 6);

        ShuffleboardLayout listLayout = tab.getLayout("Information", BuiltInLayouts.kList)
                .withProperties(Map.of("Label position", "TOP"))
                .withPosition(10, 0)
                .withSize(3, 6);

        listLayout.add("Drive Subsystem State", this);

        ShuffleboardLayout positionLayout = listLayout.getLayout("Position", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 1));

        angleEntry = ShuffleboardUtils.addRobotAngleWidget(listLayout)
                .getEntry();

        xEntry = positionLayout.add("X", 0.0)
                .withPosition(0, 0)
                .getEntry();

        yEntry = positionLayout.add("Y", 0.0)
                .withPosition(1, 0)
                .getEntry();

        ShuffleboardLayout speedsLayout = listLayout.getLayout("Wheel Speeds", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 1));

        leftSpeedEntry = ShuffleboardUtils.addDrivetrainWheelSpeedWidget(speedsLayout, "Left Wheel")
                .withPosition(0, 0)
                .getEntry();
        rightSpeedEntry = ShuffleboardUtils.addDrivetrainWheelSpeedWidget(speedsLayout, "Right Wheel")
                .withPosition(1, 0)
                .getEntry();
    }

    private void initialize() {
        pigeon2.reset();
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    private void updateShuffleboard() {
        Pose2d pose2d = differentialDriveOdometry.getPoseMeters();

        xEntry.setDouble(pose2d.getX());
        yEntry.setDouble(pose2d.getY());
        angleEntry.setDouble(getAngleDegrees());

        leftSpeedEntry.setDouble(getLeftSpeedMetersPerSecond());
        rightSpeedEntry.setDouble(getRightSpeedMetersPerSecond());
    }

    private void updateOdometry() {
        differentialDriveOdometry.update(
                new Rotation2d(Math.toRadians(getAngleDegrees())),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters()
        );
    }

    @Override
    public void periodic() {
        updateOdometry();
        updateShuffleboard();

        SmartDashboard.putNumber("DriveLeftCurrent", leftMaster.getStatorCurrent());
        SmartDashboard.putNumber("DriveRightCurrent", rightMaster.getStatorCurrent());

        field2d.setRobotPose(differentialDriveOdometry.getPoseMeters());
    }
}

