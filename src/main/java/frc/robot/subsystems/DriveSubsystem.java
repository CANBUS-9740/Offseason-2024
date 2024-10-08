package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_VictorSPX leftFrontMotor;
    private final WPI_TalonSRX rightFrontMotor;
    private final WPI_TalonSRX leftBackMotor;
    private final WPI_VictorSPX rightBackMotor;
    private final Pigeon2 pigeon2;

    private final DifferentialDrive differentialDrive;
    private final DifferentialDriveOdometry differentialDriveOdometry;
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(RobotMap.DRIVE_TRACK_WIDTH);

    private final Field2d field2d;

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

    public DriveSubsystem() {
        rightBackMotor = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_BACK_MOTOR_ID);
        leftBackMotor = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_BACK_MOTOR_ID);
        rightFrontMotor = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FRONT_MOTOR_ID);
        leftFrontMotor = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_FRONT_MOTOR_ID);

        pigeon2 = new Pigeon2(RobotMap.PIGEON_ID);

        rightBackMotor.configFactoryDefault();
        leftBackMotor.configFactoryDefault();
        rightFrontMotor.configFactoryDefault();
        leftFrontMotor.configFactoryDefault();

        pigeon2.getConfigurator().apply(new Pigeon2Configuration());

        rightFrontMotor.setInverted(true);
        rightBackMotor.setInverted(true);
        rightFrontMotor.setSensorPhase(true);
        leftBackMotor.setSensorPhase(true);

        rightBackMotor.follow(rightFrontMotor);
        leftFrontMotor.follow(leftBackMotor);

        configureMotorPID(leftFrontMotor);
        configureMotorPID(rightBackMotor);

        this.field2d = new Field2d();
        SmartDashboard.putData("field2d", field2d);

        differentialDrive = new DifferentialDrive(rightFrontMotor, leftBackMotor);

        differentialDriveOdometry = new DifferentialDriveOdometry(
                new Rotation2d(getAngleDegrees()),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters()
        );

        PathPlannerLogging.setLogActivePathCallback(poses -> field2d.getObject("PathPlannerPoses").setPoses(poses));

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
    }

    private static void configureMotorPID(WPI_VictorSPX motor) {
        motor.config_kP(0, RobotMap.DRIVE_TALON_PID_P);
        motor.config_kI(0, RobotMap.DRIVE_TALON_PID_I);
        motor.config_kD(0, RobotMap.DRIVE_TALON_PID_D);
        motor.config_kF(0, RobotMap.DRIVE_TALON_PID_F);
    }

    public Field2d getField2d() {
        return field2d;
    }

    public Pose2d getRobotPose() {
        return differentialDriveOdometry.getPoseMeters();
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
        return rightFrontMotor.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_RADIUS * 2 * Math.PI;
    }

    public double getRightDistancePassedMeters() {
        return leftBackMotor.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_RADIUS * 2 * Math.PI;
    }

    public double getLeftSpeedMetersPerSecond() {
        return rightFrontMotor.getSelectedSensorVelocity() * RobotMap.TALON_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    public double getRightSpeedMetersPerSecond() {
        return leftBackMotor.getSelectedSensorVelocity() * RobotMap.TALON_ENCODER_VELOCITY_TO_METERS_PER_SECOND;
    }

    public double getLeftOutputVoltageVolts() {
        return rightFrontMotor.getMotorOutputVoltage();
    }

    public double getRightOutputVoltageVolts() {
        return leftBackMotor.getMotorOutputVoltage();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSpeedMetersPerSecond(), getRightSpeedMetersPerSecond());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    private void initialize() {
        pigeon2.reset();
        rightFrontMotor.setSelectedSensorPosition(0);
        leftBackMotor.setSelectedSensorPosition(0);//inits
    }

    public double getAngleDegrees() {
        return (360 - pigeon2.getAngle()) % 360;// the value returned will be from 0-360 depending on its location
        // 0 - degree after initializing, 359 - one degree to the right, 1 - one degree to the left// 90 will be 90 degrees to the left
    }

    public void drivePowerLeft(double power) {
        rightFrontMotor.set(power);
    }

    public void drivePowerRight(double power) {
        leftBackMotor.set(power);
    }

    public void driveVoltsLeft(double voltage) {
        drivePowerLeft(voltage / RobotController.getBatteryVoltage());
    }

    public void driveVoltsRight(double voltage) {
        drivePowerRight(voltage / RobotController.getBatteryVoltage());
    }

    public void driveSpeedLeft(double metersPerSecond) {
        rightFrontMotor.set(ControlMode.Velocity, metersPerSecond / RobotMap.TALON_ENCODER_VELOCITY_TO_METERS_PER_SECOND);
    }

    public void driveSpeedRight(double metersPerSecond) {
        leftBackMotor.set(ControlMode.Velocity, metersPerSecond / RobotMap.TALON_ENCODER_VELOCITY_TO_METERS_PER_SECOND);
    }

    public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        driveSpeedLeft(wheelSpeeds.leftMetersPerSecond);
        driveSpeedRight(wheelSpeeds.rightMetersPerSecond);
    }

    public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        driveWheelSpeeds(kinematics.toWheelSpeeds(chassisSpeeds));
    }

    public void stop() {
        leftBackMotor.stopMotor();
        leftFrontMotor.stopMotor();
        rightBackMotor.stopMotor();
        rightFrontMotor.stopMotor();
    }

    private void updateOdometry() {
        differentialDriveOdometry.update(
                new Rotation2d(Math.toRadians(getAngleDegrees())),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters()
        );
    }

    public void arcadeDrive(double linearSpeed, double rotationSpeed) {
        differentialDrive.arcadeDrive(linearSpeed, rotationSpeed);
    }

    // SysId Stuff

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public void periodic() {
        updateOdometry();

        SmartDashboard.putNumber("angleOfBot", getAngleDegrees());
        SmartDashboard.putNumber("pigeon", pigeon2.getAngle());
        SmartDashboard.putNumber("DriveLeftDistance", getLeftDistancePassedMeters());
        SmartDashboard.putNumber("DriveRightDistance", getRightDistancePassedMeters());
        SmartDashboard.putNumber("X:", differentialDriveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y:", differentialDriveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Angle:", differentialDriveOdometry.getPoseMeters().getRotation().getDegrees());

        field2d.setRobotPose(differentialDriveOdometry.getPoseMeters());
    }
}

