package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;
import frc.robot.utils.ShuffleboardDashboard;
import frc.robot.utils.ShuffleboardUtils;

import java.util.Map;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftMaster;
    private final WPI_VictorSPX leftFollower;
    private final WPI_TalonSRX rightMaster;
    private final WPI_VictorSPX rightFollower;

    private final Pigeon2 pigeon2;

    private final Field2d field2d;
    private final DifferentialDrivePoseEstimator differentialDriveOdometry;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotMap.DRIVE_TRACK_WIDTH);
    private final DifferentialDrive differentialDrive;

    // vision
    private NetworkTable table;
    private double[] botPoseArray;
    private NetworkTableEntry networkTableEntryOfBotPose;
    private NetworkTableEntry networkTableEntryOfExistanceAprilTag;
    private boolean existanceOfAprilTag;
    private Timer timerOfAprilTagDetection;

    private final SysIdRoutine routine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            voltage -> {
                                driveVoltsLeft(voltage.in(Volts));
                                driveVoltsRight(voltage.in(Volts));
                            },
                            log -> {
                                log.motor("drive-left")
                                        .voltage(Volts.of(getLeftOutputVoltageVolts()))
                                        .linearPosition(Meters.of(getLeftDistancePassedMeters()))
                                        .linearVelocity(MetersPerSecond.of(getLeftSpeedMetersPerSecond()));
                                log.motor("drive-right")
                                        .voltage(Volts.of(getRightOutputVoltageVolts()))
                                        .linearPosition(Meters.of(getRightDistancePassedMeters()))
                                        .linearVelocity(MetersPerSecond.of(getRightSpeedMetersPerSecond()));
                            },
                            this
                    )
            );


    // Shuffleboard
    private GenericEntry xEntry;
    private GenericEntry yEntry;
    private GenericEntry angleEntry;
    private GenericEntry leftSpeedEntry;
    private GenericEntry rightSpeedEntry;
    private GenericEntry leftPositionEntry;
    private GenericEntry rightPositionEntry;

    public DriveSubsystem() {
        leftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_BACK_MOTOR_ID);
        leftFollower = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_FRONT_MOTOR_ID);
        rightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FRONT_MOTOR_ID);
        rightFollower = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_BACK_MOTOR_ID);

        pigeon2 = new Pigeon2(RobotMap.PIGEON_ID);

        leftFollower.configFactoryDefault();
        rightMaster.configFactoryDefault();
        leftMaster.configFactoryDefault();
        rightFollower.configFactoryDefault();

        pigeon2.getConfigurator().apply(new Pigeon2Configuration());

        rightMaster.setInverted(true);
        rightFollower.setInverted(true);
        leftMaster.setSensorPhase(true);
        rightMaster.setSensorPhase(true);

        rightMaster.setNeutralMode(NeutralMode.Brake);
        rightFollower.setNeutralMode(NeutralMode.Brake);
        leftMaster.setNeutralMode(NeutralMode.Brake);
        leftFollower.setNeutralMode(NeutralMode.Brake);

        rightFollower.follow(rightMaster);
        leftFollower.follow(leftMaster);


        field2d = new Field2d();

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

        differentialDriveOdometry = new DifferentialDrivePoseEstimator(
                kinematics,
                pigeon2.getRotation2d(),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters(),
                RobotMap.STARTING_DEFAULT_LOCATION
        );

        configureMotorPID(leftMaster);
        configureMotorPID(rightMaster);

        PathPlannerLogging.setLogTargetPoseCallback(pose -> field2d.getObject("PathPlannerTarget").setPose(pose));
        PathPlannerLogging.setLogActivePathCallback(poses -> field2d.getObject("PathPlannerPath").setPoses(poses));

        AutoBuilder.configureRamsete(
                this::getRobotPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::driveChassisSpeeds,
                new ReplanningConfig(),
                () -> DriverStation.getAlliance().filter(value -> value == DriverStation.Alliance.Red).isPresent(),
                this
        );

        initialize();
        setUpShuffleboard();

        ShuffleboardDashboard.setDrivetrainDataSupplier(() -> new ShuffleboardDashboard.DrivetrainData(
                getRobotPose(),
                new DifferentialDriveWheelSpeeds(getLeftSpeedMetersPerSecond(), getRightSpeedMetersPerSecond())
        ));

        timerOfAprilTagDetection = new Timer();
        timerOfAprilTagDetection.start();

        table = NetworkTableInstance.getDefault().getTable("limelight");
        networkTableEntryOfBotPose = table.getEntry("botpose_wpiblue");
        networkTableEntryOfExistanceAprilTag = table.getEntry("tv");

        botPoseArray = networkTableEntryOfBotPose.getDoubleArray(new double[6]);
        existanceOfAprilTag = networkTableEntryOfExistanceAprilTag.getInteger(0) == 1;
    }

    private static void configureMotorPID(WPI_TalonSRX motor) {
        motor.config_kP(0, RobotMap.DRIVE_TALON_PID_P);
        motor.config_kI(0, RobotMap.DRIVE_TALON_PID_I);
        motor.config_kD(0, RobotMap.DRIVE_TALON_PID_D);
        motor.config_kF(0, RobotMap.DRIVE_TALON_PID_F);
        motor.config_IntegralZone(0, RobotMap.DRIVE_TALON_PID_IZONE);
    }

    public Field2d getField2d() {
        return field2d;
    }

    public Pose2d getRobotPose() {
        return differentialDriveOdometry.getEstimatedPosition();
    }

    public double getLeftOutputVoltageVolts() {
        return leftMaster.getMotorOutputVoltage();
    }

    public double getRightOutputVoltageVolts() {
        return rightMaster.getMotorOutputVoltage();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSpeedMetersPerSecond(), getRightSpeedMetersPerSecond());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    private void resetPose(Pose2d pose) {
        differentialDriveOdometry.resetPosition(
                Rotation2d.fromDegrees(getAngleDegrees()),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters(),
                pose
        );
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

    public void drivePowerLeft(double power) {
        leftMaster.set(power);
    }

    public void drivePowerRight(double power) {
        rightMaster.set(power);
    }

    public void driveVoltsLeft(double voltage) {
        drivePowerLeft(voltage / RobotController.getBatteryVoltage());
    }

    public void driveVoltsRight(double voltage) {
        drivePowerRight(voltage / RobotController.getBatteryVoltage());
    }

    public void driveSpeedLeft(double metersPerSecond) {
        SmartDashboard.putNumber("LeftTargetSpeed", metersPerSecond);
        leftMaster.set(ControlMode.Velocity, metersPerSecond / RobotMap.TALON_ENCODER_VELOCITY_TO_METERS_PER_SECOND);
    }

    public void driveSpeedRight(double metersPerSecond) {
        SmartDashboard.putNumber("RightTargetSpeed", metersPerSecond);
        rightMaster.set(ControlMode.Velocity, metersPerSecond / RobotMap.TALON_ENCODER_VELOCITY_TO_METERS_PER_SECOND);
    }

    public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        driveSpeedLeft(wheelSpeeds.leftMetersPerSecond);
        driveSpeedRight(wheelSpeeds.rightMetersPerSecond);
    }

    public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        driveWheelSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
    }

    public void arcadeDrive(double linearSpeed, double rotationSpeed) {
        differentialDrive.arcadeDrive(linearSpeed, rotationSpeed);
    }

    public void stop() {
        rightMaster.stopMotor();
        leftMaster.stopMotor();
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

        leftSpeedEntry = ShuffleboardUtils.addDrivetrainWheelSpeedWidget(speedsLayout, "Left Speed")
                .withPosition(0, 0)
                .getEntry();
        rightSpeedEntry = ShuffleboardUtils.addDrivetrainWheelSpeedWidget(speedsLayout, "Right Speed")
                .withPosition(1, 0)
                .getEntry();

        ShuffleboardLayout positionsLayout = listLayout.getLayout("Wheel Positions", BuiltInLayouts.kGrid)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 1));

        leftPositionEntry = positionsLayout.add("Left Position", 0.0)
                .withPosition(0, 0)
                .getEntry();
        rightPositionEntry = positionsLayout.add("Right Position", 0.0)
                .withPosition(1, 0)
                .getEntry();
    }

    private void initialize() {
        pigeon2.reset();
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    private void updateShuffleboard() {
        Pose2d pose2d = differentialDriveOdometry.getEstimatedPosition();

        xEntry.setDouble(pose2d.getX());
        yEntry.setDouble(pose2d.getY());
        angleEntry.setDouble(getAngleDegrees());

        leftSpeedEntry.setDouble(getLeftSpeedMetersPerSecond());
        rightSpeedEntry.setDouble(getRightSpeedMetersPerSecond());
        leftPositionEntry.setDouble(getLeftDistancePassedMeters());
        rightPositionEntry.setDouble(getRightDistancePassedMeters());
    }

    private void updateOdometry() {
        //DifferentialDrivePoseEstimator !!!1
        if (existanceOfAprilTag && timerOfAprilTagDetection.get() > 1.5) {
            differentialDriveOdometry.resetPosition(
                    pigeon2.getRotation2d(),
                    getLeftDistancePassedMeters(),
                    getRightDistancePassedMeters(),
                    new Pose2d(
                            getXFromCamera(),
                            getYFromCamera(),
                            new Rotation2d(Units.degreesToRadians(getYawFromCamera()))));
            timerOfAprilTagDetection.reset();
        }
        differentialDriveOdometry.update(
                new Rotation2d(Math.toRadians(getAngleDegrees())),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters()
        );
    }

    private double getXFromCamera() {
        return botPoseArray[0];
    }

    private double getYFromCamera() {
        return botPoseArray[1];
    }

    private double getZFromCamera() {
        return botPoseArray[2];
    }

    private double getRollFromCamera() {
        return botPoseArray[3];
    }

    private double getPitchFromCamera() {
        return botPoseArray[4];
    }

    private double getYawFromCamera() {
        return botPoseArray[5];
    }

    // SysId Stuff

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    // PathPlanner

    public Command pathfindTo(Pose2d pose2d) {
        return AutoBuilder.pathfindToPose(pose2d, RobotMap.PATH_CONSTRAINTS, 0.0, 1.0);
    }

    @Override
    public void periodic() {
        updateOdometry();
        updateShuffleboard();

        SmartDashboard.putNumber("DriveLeftCurrent", leftMaster.getStatorCurrent());
        SmartDashboard.putNumber("DriveRightCurrent", rightMaster.getStatorCurrent());
        SmartDashboard.putNumber("DriveLeftVoltage", leftMaster.getMotorOutputVoltage());
        SmartDashboard.putNumber("DriveRightVoltage", rightMaster.getMotorOutputVoltage());

        SmartDashboard.putNumber("Position2dX", differentialDriveOdometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Position2dY", differentialDriveOdometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Position2dDegree", differentialDriveOdometry.getEstimatedPosition().getRotation().getDegrees());

        botPoseArray = networkTableEntryOfBotPose.getDoubleArray(new double[6]);
        existanceOfAprilTag = networkTableEntryOfExistanceAprilTag.getInteger(0) == 1;

        SmartDashboard.putBoolean("IsTag: ", existanceOfAprilTag);
        SmartDashboard.putNumber("yaw", getYawFromCamera());

        field2d.setRobotPose(getRobotPose());
    }
}

