package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX rightFrontMotor;
    private final WPI_VictorSPX leftFrontMotor;
    private final WPI_VictorSPX rightBackMotor;
    private final WPI_TalonSRX leftBackMotor;
    private final Field2d field2d;
    private final DifferentialDriveOdometry differentialDriveOdometry;
    private final DifferentialDrive differentialDrive;
    private final Pigeon2 pigeon2;

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
        leftFrontMotor.setSensorPhase(true);

        rightBackMotor.follow(rightFrontMotor);
        leftFrontMotor.follow(leftBackMotor);

        this.field2d = new Field2d();
        SmartDashboard.putData("field2d" ,field2d);

        differentialDrive = new DifferentialDrive(rightFrontMotor, leftBackMotor);

        differentialDriveOdometry = new DifferentialDriveOdometry(
                new Rotation2d(getAngleDegrees()),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters()
        );

        initialize();
    }

    public Field2d getField2d() {
        return field2d;
    }

    public double getLeftDistancePassedMeters() {
        return rightFrontMotor.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_RADIUS * 2 * Math.PI;
    }

    public double getRightDistancePassedMeters() {
        return leftBackMotor.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_RADIUS * 2 * Math.PI;
    }

    private void initialize() {
        pigeon2.reset();
        rightFrontMotor.setSelectedSensorPosition(0);
        leftBackMotor.setSelectedSensorPosition(0);//inits
    }

    public double getAngleDegrees() {
        return (360 - pigeon2.getAngle()) % 360;// the value returned will be from 0 - 360 depending on its location
        // 0  - degree after initializing, 359 - one degree to the right, 1 - one degree to the left// 90 will be 90 degrees to the left
    }

    public void arcadeDrive(double linearSpeed, double rotationSpeed){
        differentialDrive.arcadeDrive(linearSpeed, rotationSpeed);
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

    public void periodic() {
        updateOdometry();

        SmartDashboard.putNumber("angleOfBot", getAngleDegrees());
        SmartDashboard.putNumber("pigeon" , pigeon2.getAngle());
        SmartDashboard.putNumber("DriveLeftDistance", getLeftDistancePassedMeters());
        SmartDashboard.putNumber("DriveRightDistance", getRightDistancePassedMeters());
        SmartDashboard.putNumber("X:", differentialDriveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y:", differentialDriveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Angle:", differentialDriveOdometry.getPoseMeters().getRotation().getDegrees());

        field2d.setRobotPose(differentialDriveOdometry.getPoseMeters());
    }
}

