package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

import static edu.wpi.first.units.Units.Centimeters;

public class RobotMap {
    private RobotMap() {
        // Required empty constructor
    }

    public static final double DRIVE_WHEEL_RADIUS = Units.inchesToMeters(3);
    public static final double DRIVE_WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * DRIVE_WHEEL_RADIUS;

    public static final Measure<Distance> DRIVE_TRACK_WIDTH = Centimeters.of(60);

    public static final int DRIVE_RIGHT_FRONT_MOTOR_ID = 4;
    public static final int DRIVE_RIGHT_BACK_MOTOR_ID = 5;
    public static final int DRIVE_LEFT_FRONT_MOTOR_ID = 2;
    public static final int DRIVE_LEFT_BACK_MOTOR_ID = 3;
    public static final int PIGEON_ID = 7;

    public static final double TALON_ENCODER_PPR = 4096;
    public static final double TALON_ENCODER_TIMEFRAME_SECONDS = 0.1;

    public static final double TALON_ENCODER_VELOCITY_TO_METERS_PER_SECOND =
            1.0
                    / TALON_ENCODER_TIMEFRAME_SECONDS
                    / TALON_ENCODER_PPR
                    * DRIVE_WHEEL_CIRCUMFERENCE_METERS;

    public static final double DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO = 8.45;

    public static final double DRIVE_TALON_PID_P = 1.2;
    public static final double DRIVE_TALON_PID_I = 0.01;
    public static final double DRIVE_TALON_PID_D = 0.15;
    public static final double DRIVE_TALON_PID_F = 0.0;
    public static final double DRIVE_TALON_PID_IZONE = 0.1;

    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(3.0, 1.0, Units.rotationsToRadians(2), Units.rotationsToRadians(1));

    public static final int ARM_MOTOR_PORT = 8;
    public static final int ARM_ENCODER_PORT = 0;
    @SuppressWarnings("PointlessArithmeticExpression")
    public static final double ARM_GEAR_RATIO = 100.0 / 1.0;
    public static final int ARM_NEO_ENCODER_CPR = 42;
    public static final double ARM_SHOOTER_ANGLE = 194;
    public static final double ARM_FLOOR_ANGLE = 12;
    public static final double ARM_MIN_ANGLE = 8;
    public static final double ARM_MAX_ANGLE = 195;
    public static final double ABSOLUTE_ENCODER_ZERO_OFFSET = 214.0 / 360.0;
    public static final double ARM_PID_P = 0.01;
    public static final double ARM_PID_I = 0.005;
    public static final double ARM_PID_D = 0.0;
    public static final int ARM_PID_I_ZONE = 10;
    public static final double ARM_PID_K_GRAVITY = 0.03;
    public static final int NEAR_ANGLE_TOLERANCE = 5;
    public static final double ARM_AMP_ANGLE = 128;
    public static final double ARM_AMP_RELEASE_ANGLE = 130;


    public static final int SHOOTER_MOTOR_LEFT_TOP = 13;
    public static final int SHOOTER_MOTOR_LEFT_BOTTOM = 12;
    public static final int SHOOTER_MOTOR_RIGHT_TOP = 15;
    public static final int SHOOTER_MOTOR_RIGHT_BOTTOM = 14;
    public static final int TARGET_RPM_SHOOTER_FAST = 5500;
    public static final int TARGET_RPM_SHOOTER_SLOW = 2000;
    public static final int TARGET_RPM_WIGGLE_ROOM_SHOOTER = 50;

    public static final int INTAKE_MOTOR = 9;
    public static final int INTAKE_LIMIT_SWITCH = 5;

    public static final double INTAKE_MOTOR_TO_WHEEL_RATIO = 3;

    public static final Pose2d STARTING_DEFAULT_LOCATION = new Pose2d(0,0,new Rotation2d(0));
}
