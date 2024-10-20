package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.utils.ShuffleboardDashboard;
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
    public final double SHOOTER_ROTATE_SPEED = 0.5;
    public final double SHOOTER_RPM_KP = 0.0001;
    public final double SHOOTER_RPM_KI = 0.000001;
    public final double SHOOTER_RPM_KD = 0;

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

        motorRT.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motorLT.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motorLB.setIdleMode(CANSparkBase.IdleMode.kBrake);
        motorRB.setIdleMode(CANSparkBase.IdleMode.kBrake);

        motorRT.setInverted(true);
        motorRB.setInverted(true);
        motorLB.setSmartCurrentLimit(60, 20);
        motorRT.setSmartCurrentLimit(60, 20);
        motorRB.setSmartCurrentLimit(60, 20);
        motorLT.setSmartCurrentLimit(60, 20);
        // a 50, 20; i 20, 5

        encoderLT = motorLT.getEncoder();
        encoderLB = motorLB.getEncoder();
        encoderRT = motorRT.getEncoder();
        encoderRB = motorRB.getEncoder();

        pid = motorLB.getPIDController();

        pid.setP(SHOOTER_RPM_KP, 0);
        pid.setI(SHOOTER_RPM_KI, 0);
        pid.setD(SHOOTER_RPM_KD, 0);

        motorRB.follow(motorLB, true);
        motorRT.follow(motorLB, true);
        motorLT.follow(motorLB);

        setUpShuffleboard();

        ShuffleboardDashboard.setShooterDataSupplier(() -> new ShuffleboardDashboard.ShooterData(
                getLeftTopVelocityRpm(),
                getRightTopVelocityRpm(),
                getLeftBottomVelocityRpm(),
                getRightBottomVelocityRpm(),
                reachedRPM(RobotMap.TARGET_RPM_SHOOTER_FAST)
        ));
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
                .withProperties(Map.of("Number of columns", 2, "Number of rows", 2))
                .withPosition(3, 1)
                .withSize(4, 3);

        leftTopSpeed = ShuffleboardUtils.addShooterSpeedWidget(speedsLayout, "Left Top")
                .withPosition(0, 0)
                .getEntry();
        rightTopSpeed = ShuffleboardUtils.addShooterSpeedWidget(speedsLayout, "Right Top")
                .withPosition(1, 0)
                .getEntry();
        leftBottomSpeed = ShuffleboardUtils.addShooterSpeedWidget(speedsLayout, "Left Bottom")
                .withPosition(0, 1)
                .getEntry();
        rightBottomSpeed = ShuffleboardUtils.addShooterSpeedWidget(speedsLayout, "Right Bottom")
                .withPosition(1, 1)
                .getEntry();
    }

    private void updateShuffleboard() {
        leftTopSpeed.setDouble(getLeftTopVelocityRpm());
        rightTopSpeed.setDouble(getRightTopVelocityRpm());
        leftBottomSpeed.setDouble(getLeftBottomVelocityRpm());
        rightBottomSpeed.setDouble(getRightBottomVelocityRpm());
    }

    public boolean reachedRPM(double rpm) {
        return getRightBottomVelocityRpm() > (rpm - RobotMap.TARGET_RPM_WIGGLE_ROOM_SHOOTER);
    }

    @Override
    public void periodic() {
        updateShuffleboard();
    }
}
