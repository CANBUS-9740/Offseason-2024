package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftFrontMotor;
    private final WPI_VictorSPX rightFrontMotor;
    private final WPI_VictorSPX leftBackMotor;
    private final WPI_TalonSRX rightBackMotor;
    private final Field2d twodFieldXY;
    private final DifferentialDriveOdometry differentialDriveOdometry;
    private final DifferentialDrive differentialDrive;
    private final Pigeon2 pigeon2;
    private NetworkTable table;
    private double[] botPoseArray;
    private NetworkTableEntry networkTableEntryOfBotPose;
    private NetworkTableEntry networkTableEntryOfExistanceAprilTag;
    private boolean existanceOfAprilTag;


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

        table = NetworkTableInstance.getDefault().getTable("limelight");
        networkTableEntryOfBotPose = table.getEntry("botpose_wpiblue");
        networkTableEntryOfExistanceAprilTag = table.getEntry("tv");

        twodFieldXY = new Field2d();

        botPoseArray = networkTableEntryOfBotPose.getDoubleArray(new double[6]);
        existanceOfAprilTag = networkTableEntryOfExistanceAprilTag.getInteger(0) == 1;

        leftFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);
        leftFrontMotor.setSensorPhase(true);

        SmartDashboard.putData("field2d", twodFieldXY);

        differentialDrive = new DifferentialDrive(leftFrontMotor, rightBackMotor);

        differentialDriveOdometry = new DifferentialDriveOdometry(
                pigeon2.getRotation2d(),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters()
        );

        initialize();
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

    public Field2d getTwodFieldXY() {
        return twodFieldXY;
    }

    public double getLeftDistancePassedMeters() {
        return leftFrontMotor.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_RADIUS * 2 * Math.PI;
    }

    public double getRightDistancePassedMeters() {
        return rightBackMotor.getSelectedSensorPosition() / RobotMap.TALON_ENCODER_PPR * RobotMap.DRIVE_WHEEL_RADIUS * 2 * Math.PI;
    }

    private void initialize() {
        pigeon2.reset();
        leftFrontMotor.setSelectedSensorPosition(0);
        rightBackMotor.setSelectedSensorPosition(0);//inits
    }

    public double getAngleDegrees() {
        return (360 - pigeon2.getAngle()) % 360;// the value returned will be from 0 - 360 depending on its location
        // 0  - degree after initializing, 359 - one degree to the right, 1 - one degree to the left// 90 will be 90 degrees to the left
    }

    public void arcadeDrive(double linearSpeed, double rotationSpeed) {
        differentialDrive.arcadeDrive(linearSpeed, rotationSpeed);
    }

    public void stop() {
        rightBackMotor.stopMotor();
        rightFrontMotor.stopMotor();
        leftBackMotor.stopMotor();
        leftFrontMotor.stopMotor();
    }


    private void updateOdometry() {
        if (existanceOfAprilTag) {
            differentialDriveOdometry.resetPosition(
                    pigeon2.getRotation2d(),
                    getLeftDistancePassedMeters(),
                    getRightDistancePassedMeters(),
                    new Pose2d(
                            getXFromCamera(),
                            getYFromCamera(),
                            new Rotation2d(Units.degreesToRadians(getYawFromCamera()))));

        }

        differentialDriveOdometry.update(
                new Rotation2d(Math.toRadians(getAngleDegrees())),
                getLeftDistancePassedMeters(),
                getRightDistancePassedMeters()
                //TODO: change the condition
        );
    }

    public double getCurrentXLocation(){
        return differentialDriveOdometry.getPoseMeters().getX();
    }
    public double getCurrentYLocation(){
        return differentialDriveOdometry.getPoseMeters().getY();
    }
    public double getCurrentAngleLocation(){
        return differentialDriveOdometry.getPoseMeters().getRotation().getDegrees();
    }


    public void periodic() {
        twodFieldXY.setRobotPose(differentialDriveOdometry.getPoseMeters());
        botPoseArray = networkTableEntryOfBotPose.getDoubleArray(new double[6]);
        existanceOfAprilTag = networkTableEntryOfExistanceAprilTag.getInteger(0) == 1;
        updateOdometry();
        SmartDashboard.putBoolean("IsTag: ", existanceOfAprilTag);
        SmartDashboard.putNumber("yaw", getYawFromCamera());

    }
}

