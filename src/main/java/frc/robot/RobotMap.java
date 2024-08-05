package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class RobotMap {
    public static final double WHEEL_RADIUS = Units.inchesToMeters(3);

    public static final int LEFT_FRONT_MOTOR_ID = 4;
    public static final int LEFT_BACK_MOTOR_ID = 5;
    public static final int RIGHT_FRONT_MOTOR_ID = 2;
    public static final int RIGHT_BACK_MOTOR_ID = 3;
    public static final int PIGEON_ID = 7;
    public static final int DRIVE_CONTROLLER_PORT = 0;

    public static final double TALON_ENCODER_PPR = 4096;

    public static final double DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO = 8.45;
}
