package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.ShuffleboardUtils;

import java.util.Map;

public class ShooterSystem extends SubsystemBase {
    private final CANSparkMax motorLT;
    private final CANSparkMax motorRT;
    private final CANSparkMax motorLB;
    private final CANSparkMax motorRB;
    private final RelativeEncoder encoderLT;
    private final RelativeEncoder encoderRT;
    private final RelativeEncoder encoderLB;
    private final RelativeEncoder encoderRB;
    private final SparkPIDController pid;
    public static final double SHOOTER_ROTATE_SPEED = 0.5;
    public static final double SHOOTER_RPM_KP = 0.0001;
    public static final double SHOOTER_RPM_KI = 0.000001;
    public static final double SHOOTER_RPM_KD = 0;

    // Shuffleboard

    private GenericEntry leftTopSpeed;
    private GenericEntry rightTopSpeed;
    private GenericEntry leftBottomSpeed;
    private GenericEntry rightBottomSpeed;

    public ShooterSystem() {
        motorLT = new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_TOP, CANSparkLowLevel.MotorType.kBrushless);
        motorRT = new CANSparkMax(RobotMap.SHOOTER_MOTOR_RIGHT_TOP, CANSparkLowLevel.MotorType.kBrushless);
        motorLB = new CANSparkMax(RobotMap.SHOOTER_MOTOR_LEFT_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);
        motorRB = new CANSparkMax(RobotMap.SHOOTER_MOTOR_RIGHT_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);

        motorRT.restoreFactoryDefaults();
        motorLT.restoreFactoryDefaults();
        motorLB.restoreFactoryDefaults();
        motorRB.restoreFactoryDefaults();

        motorRT.setInverted(true);
        motorRB.setInverted(true);

        encoderLT = motorLT.getEncoder();
        encoderLB = motorLB.getEncoder();
        encoderRT = motorRT.getEncoder();
        encoderRB = motorRB.getEncoder();

        pid = motorLB.getPIDController();

        pid.setP(SHOOTER_RPM_KP, 0);
        pid.setI(SHOOTER_RPM_KI, 0);
        pid.setD(SHOOTER_RPM_KD, 0);

        motorRB.follow(motorLB);
        motorRT.follow(motorLB);
        motorLT.follow(motorLB);

        setUpShuffleboard();
    }

    public void stop() {
        motorLT.stopMotor();
        motorLB.stopMotor();
        motorRB.stopMotor();
        motorRT.stopMotor();
    }

    public void rotate() {
        motorLT.set(SHOOTER_ROTATE_SPEED);
        motorRB.set(SHOOTER_ROTATE_SPEED);
        motorLB.set(SHOOTER_ROTATE_SPEED);
        motorRT.set(SHOOTER_ROTATE_SPEED);
    }


    public double getLeftBottomVelocityRpm() {
        return encoderLB.getVelocity();
    }

    public double getLeftTopVelocityRpm() {
        return encoderLT.getVelocity();
    }

    public double getRightTopVelocityRpm() {
        return encoderRT.getVelocity();
    }

    public double getRightBottomVelocityRpm() {
        return encoderRB.getVelocity();
    }

    public void rotatePID(double targetRPM) {
        pid.setReference(targetRPM, CANSparkBase.ControlType.kVelocity);
    }

    private void setUpShuffleboard() {
        ShuffleboardTab tab = ShuffleboardUtils.getArmIntakeShooterTab();

        ShuffleboardLayout subsystemsLayout = ShuffleboardUtils.getArmIntakeShooterSubsystemsLayout();
        subsystemsLayout.add("Shooter", this)
                .withPosition(2, 0);

        ShuffleboardLayout speedsLayout = tab.getLayout("Shooter Motor Speeds RPM", BuiltInLayouts.kGrid)
                .withPosition(4, 2)
                .withSize(4, 3)
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 2));

        leftTopSpeed = speedsLayout.add("Left Top", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 5000))
                .withPosition(0, 0)
                .getEntry();
        rightTopSpeed = speedsLayout.add("Right Top", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 5000))
                .withPosition(1, 0)
                .getEntry();
        leftBottomSpeed = speedsLayout.add("Left Bottom", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 5000))
                .withPosition(0, 1)
                .getEntry();
        rightBottomSpeed = speedsLayout.add("Right Bottom", 0.0)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("min", 0, "max", 5000))
                .withPosition(1, 1)
                .getEntry();
    }

    private void updateShuffleboard() {
        leftTopSpeed.setDouble(getLeftTopVelocityRpm());
        rightTopSpeed.setDouble(getRightTopVelocityRpm());
        leftBottomSpeed.setDouble(getLeftBottomVelocityRpm());
        rightBottomSpeed.setDouble(getRightBottomVelocityRpm());
    }

    @Override
    public void periodic() {
        updateShuffleboard();

        SmartDashboard.putNumber("ShooterLeftTopMotor", getLeftTopVelocityRpm());
        SmartDashboard.putNumber("ShooterLeftBottomMotor", getLeftBottomVelocityRpm());
        SmartDashboard.putNumber("ShooterRightTopMotor", getRightTopVelocityRpm());
        SmartDashboard.putNumber("ShooterRightBottomMotor", getRightBottomVelocityRpm());
    }
}
