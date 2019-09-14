package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Ashley on 11/29/2017.
 */

public class Constants {

    // Arm geometry
    static final int armFoldedOffset = 0; //60;
    static final int wristFoldedOffset = 0; //1095;
    public static final double ARM_BOTTOM_ANGLE_DEGREES = -60 + armFoldedOffset;
    public static final double ARM_BOTTOM_HARDSTOP = -60 +armFoldedOffset; //fix
    public static final int ARM_BOTTOM_TICKS = 0 + armFoldedOffset;
    public static final int ARM_LEVEL_TICKS = 2140 + armFoldedOffset;
    public static final int ARM_VERTICAL_TICKS = 5858 + armFoldedOffset;
    public static final double ARM_TICKS_PER_DEGREE = (ARM_VERTICAL_TICKS - ARM_LEVEL_TICKS) / 90;

    public static final int WRIST_MAX_TICKS = 267 + wristFoldedOffset;
    public static final int WRIST_MIN_TICKS = -1095 + wristFoldedOffset;
    public static final int WRIST_START_TICKS = 0 + wristFoldedOffset;
    public static final int WRIST_STRAIGHT_TICKS = -320 + wristFoldedOffset; //get value
    public static final double WRIST_MAX_ANGLE = 59 + wristFoldedOffset; //Above start
    public static final double WRIST_TICKS_PER_DEGREE = (WRIST_MAX_TICKS - WRIST_START_TICKS) / WRIST_MAX_ANGLE;

    public static final int LIFTER_MAX_TICKS = 4100;
    public static final int LIFTER_MIN_TICKS = 0;
    public static final double LIFTER_MAX_ANGEL = 0;

    public static final double DRIVE_WHEEL_DIAMETER_INCHES  =  6/1.057;
    public static final double DRIVE_WHEEL_LATERAL_RATIO = 0.89 * 0.963 / 0.95884 * 0.96312;

    public static final double MM_PER_IN = 25.4f;
    private static double rotationScaleIncrease = 1.0975 / 1.088844444; //1.0084722;
    public static final double WHEELBASE_WIDTH_IN = 15 / rotationScaleIncrease;
    public static final double WHEELBASE_LENGTH_IN = 11.75 / rotationScaleIncrease;
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



    // CONSTANTS specific to our purple testing MiniRobot
    public static class MiniRobot {
        public static final double DRIVE_WHEEL_DIAMETER_INCHES  =  4;
        public static final double DRIVE_WHEEL_LATERAL_RATIO = 0.89;
        private static double rotationScaleIncrease = 1.0975;
        public static final double WHEELBASE_WIDTH_IN = 15.268 / rotationScaleIncrease;
        public static final double WHEELBASE_LENGTH_IN = 13.5 / rotationScaleIncrease;
        public static final int DRIVE_WHEEL_STEPS_PER_ROT     = 28*40;

        // Simplify initializing mecanum navigation.
        public static MecanumNavigation.DriveTrainMecanum getDriveTrainMecanum() {
            return new MecanumNavigation.DriveTrainMecanum(
                    Constants.MiniRobot.WHEELBASE_LENGTH_IN, Constants.MiniRobot.WHEELBASE_WIDTH_IN,
                    Constants.MiniRobot.DRIVE_WHEEL_DIAMETER_INCHES, Constants.MiniRobot.DRIVE_WHEEL_STEPS_PER_ROT,
                    Constants.MiniRobot.DRIVE_WHEEL_LATERAL_RATIO);
        }
    }

}
