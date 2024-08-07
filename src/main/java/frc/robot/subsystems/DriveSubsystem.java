package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
    private WPI_TalonSRX leftFrontMotor;
    private WPI_VictorSPX rightFrontMotor;
    private WPI_VictorSPX leftBackMotor;
    private WPI_TalonSRX rightBackMotor;
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
        // 0  - initial degree, 359 - one degree to the right, 1 - one degree to the left// 90 will be 90 degrees to the left
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
}

