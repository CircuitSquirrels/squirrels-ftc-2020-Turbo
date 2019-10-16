package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Ashley on 11/29/2017.
 */

public class Constants {

    public static final double DRIVE_WHEEL_DIAMETER_INCHES  =  6/1.057;
    public static final double DRIVE_WHEEL_LATERAL_RATIO = 0.89 * 0.963 / 0.95884 * 0.96312;

    public static final double MM_PER_IN = 25.4f;
    private static double rotationScaleIncrease = 1.0975 / 1.088844444; //1.0084722;
    public static final double WHEELBASE_WIDTH_IN = 11.5 / rotationScaleIncrease;
    public static final double WHEELBASE_LENGTH_IN = 16 / rotationScaleIncrease;
    public static final double WHEELBASE_WIDTH_MM  = WHEELBASE_WIDTH_IN  * MM_PER_IN;
    public static final double WHEELBASE_LENGTH_MM  = WHEELBASE_LENGTH_IN  * MM_PER_IN;
    public static final double DRIVE_WHEEL_RADIUS_MM = DRIVE_WHEEL_DIAMETER_INCHES /2.0 * MM_PER_IN;
    public static final double DRIVE_WHEEL_MM_PER_ROT = DRIVE_WHEEL_RADIUS_MM * 2 *  Math.PI;

    public static final int DRIVE_WHEEL_STEPS_PER_ROT     = 28*20*2;
    
    // Simplify initializing mecanum navigation.
    public static MecanumNavigation.DriveTrainMecanum getDriveTrainMecanum() {
        return new MecanumNavigation.DriveTrainMecanum(
                Constants.WHEELBASE_LENGTH_IN, Constants.WHEELBASE_WIDTH_IN,
                Constants.DRIVE_WHEEL_DIAMETER_INCHES, Constants.DRIVE_WHEEL_STEPS_PER_ROT,
                Constants.DRIVE_WHEEL_LATERAL_RATIO);
    }

}
