package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftFrontMotor;
    private final WPI_VictorSPX rightFrontMotor;
    private final WPI_VictorSPX leftBackMotor;
    private final WPI_TalonSRX rightBackMotor;
    private final Field2d field2d;
    private final DifferentialDriveOdometry odometry;
    private Pigeon2 pigeon2;

    public DriveSubsystem() {
        leftBackMotor = new WPI_VictorSPX(RobotMap.DRIVE_LEFT_BACK_MOTOR_ID);
        rightBackMotor = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_BACK_MOTOR_ID);
        leftFrontMotor = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_FRONT_MOTOR_ID);
        rightFrontMotor = new WPI_VictorSPX(RobotMap.DRIVE_RIGHT_FRONT_MOTOR_ID);
        pigeon2 = new Pigeon2(RobotMap.PIGEON_ID);
        leftBackMotor.configFactoryDefault();
        rightBackMotor.configFactoryDefault();
        leftFrontMotor.configFactoryDefault();
        rightFrontMotor.configFactoryDefault();
        pigeon2.getConfigurator().apply(new Pigeon2Configuration());

        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);
        leftFrontMotor.setSensorPhase(true);

        this.field2d = new Field2d();

        this.odometry = new DifferentialDriveOdometry(new Rotation2d(getAngleDegrees()), getLeftDistancePassedMeters(), getRightDistancePassedMeters());
        field2d.setRobotPose(this.odometry.getPoseMeters().getX(),this.odometry.getPoseMeters().getY(), this.odometry.getPoseMeters().getRotation() );

        initialize();


    }
    public double getLeftDistancePassedMeters(){
        return leftFrontMotor.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_RADIUS*2*Math.PI;
    }
    public double getRightDistancePassedMeters(){
        return rightBackMotor.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_RADIUS*2*Math.PI;
    }
    public void initialize(){
        pigeon2.reset();
        leftFrontMotor.setSelectedSensorPosition(0);
        rightBackMotor.setSelectedSensorPosition(0);//inits
    }
    public double getAngleDegrees(){
        return (360 - pigeon2.getAngle()) % 360;// the value returned will be from 0 - 360 depending on its location
        // 0  - degree after initializing, 359 - one degree to the right, 1 - one degree to the left// 90 will be 90 degrees to the left
    }
    public void powerLeftMotors(double power){
        leftFrontMotor.set(power);
        leftBackMotor.set(power);
    }
    public void powerRightMotors(double power){
        rightFrontMotor.set(power);
        rightBackMotor.set(power);
    }
    public void stop(){
        rightBackMotor.stopMotor();
        rightFrontMotor.stopMotor();
        leftBackMotor.stopMotor();
        leftFrontMotor.stopMotor();
    }

    public void setOdometryPose2d (Pose2d pose2d){
        odometry.update(pose2d.getRotation(), getRightDistancePassedMeters(), getLeftDistancePassedMeters());

    }

    public void periodic(){
        odometry.update(new Rotation2d(getAngleDegrees()), getLeftDistancePassedMeters(), getRightDistancePassedMeters());
        SmartDashboard.putNumber("angleOfBot", getAngleDegrees());
        SmartDashboard.putNumber("DriveLeftDistance", getLeftDistancePassedMeters());
        SmartDashboard.putNumber("DriveRightDistance", getRightDistancePassedMeters());
        SmartDashboard.putNumber("X:", this.odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y:", this.odometry.getPoseMeters().getY());
        SmartDashboard.putData("field: ", field2d);
        SmartDashboard.putNumber("Angle:", this.odometry.getPoseMeters().getRotation().getDegrees());
        field2d.setRobotPose(this.odometry.getPoseMeters().getX(),this.odometry.getPoseMeters().getY(), this.odometry.getPoseMeters().getRotation() );
        field2d.setRobotPose(odometry.getPoseMeters());

    }
}

