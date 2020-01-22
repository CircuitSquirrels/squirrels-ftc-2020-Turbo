package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Ashley on 11/29/2017.
 */

public class Constants {

    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 4 / 1.04;
    public static final double DRIVE_WHEEL_LATERAL_RATIO = 72 / 73.2 * 72 / 80.87;

    public static final double MM_PER_IN = 25.4f;
    private static double rotationScaleIncrease = 3600 / 3635.9;
    public static final double WHEELBASE_WIDTH_IN = 15.5 / rotationScaleIncrease;
    public static final double WHEELBASE_LENGTH_IN = 7.5 / rotationScaleIncrease;
    public static final double WHEELBASE_WIDTH_MM = WHEELBASE_WIDTH_IN  * MM_PER_IN;
    public static final double WHEELBASE_LENGTH_MM = WHEELBASE_LENGTH_IN  * MM_PER_IN;
    public static final double DRIVE_WHEEL_RADIUS_MM = DRIVE_WHEEL_DIAMETER_INCHES /2.0 * MM_PER_IN;
    public static final double DRIVE_WHEEL_MM_PER_ROT = DRIVE_WHEEL_RADIUS_MM * 2 *  Math.PI;

    public static final double DRIVE_WHEEL_STEPS_PER_ROT = 28*19.2;

    public static final double LEFT_CLAW_VERTICAL = 0.12;
    public static final double LEFT_CLAW_OPEN = 0.59-.1;
    public static final double LEFT_CLAW_CLOSED = 0.85;
    public static final double RIGHT_CLAW_VERTICAL = 0.88;
    public static final double RIGHT_CLAW_OPEN = 0.39+.1;
    public static final double RIGHT_CLAW_CLOSED = 0.15;

    // Arm Lift Encoders
    public static final double LIFT_TICKS_PER_INCH = 335;
    public static final double LIFT_FOUNDATION_HEIGHT_TICKS = 360;
    public static final double LIFT_KNOB_HEIGHT_TICKS = 330 * 2;


    
    // Simplify initializing mecanum navigation.
    public static MecanumNavigation.DriveTrainMecanum getDriveTrainMecanum() {
        return new MecanumNavigation.DriveTrainMecanum(
                Constants.WHEELBASE_LENGTH_IN, Constants.WHEELBASE_WIDTH_IN,
                Constants.DRIVE_WHEEL_DIAMETER_INCHES, Constants.DRIVE_WHEEL_STEPS_PER_ROT,
                Constants.DRIVE_WHEEL_LATERAL_RATIO);
    }
}
