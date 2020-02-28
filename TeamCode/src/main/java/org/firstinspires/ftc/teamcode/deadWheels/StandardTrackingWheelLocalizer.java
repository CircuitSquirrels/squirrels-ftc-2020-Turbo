package org.firstinspires.ftc.teamcode.deadWheels;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.RobotHardware.MotorName;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Frame2D;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */

public class StandardTrackingWheelLocalizer extends ModifiedThreeTrackingWheelLocalizer implements Localizer
{
//    public static double TICKS_PER_REV = 0;
//    public static double WHEEL_RADIUS = 2; // in
//    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
//
//    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
//    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    private RobotHardware opmode;

    public StandardTrackingWheelLocalizer(RobotHardware opmode) {
        super(Arrays.asList(
                toPose2dFromNav2d(OdometryConfig.getLeftWheelPosition()), // left
                toPose2dFromNav2d(OdometryConfig.getRightWheelPosition()), // right
                toPose2dFromNav2d(OdometryConfig.getCenterWheelPosition()) // center
        ));

        this.opmode = opmode;
    }

    public static double encoderTicksToInches(int ticks) {
        return OdometryConfig.inchesFromTicks(ticks);
    }

    public static Pose2d toPose2dFromNav2d(Navigation2D navigation2D) {
        return new Pose2d(navigation2D.x, navigation2D.y, navigation2D.theta);
    }

    public static Navigation2D toNav2dFromPose2d(Pose2d pose2d) {
        return new Navigation2D(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(opmode.getEncoderValue(MotorName.LEFT_WHEEL)),
                encoderTicksToInches(opmode.getEncoderValue(MotorName.RIGHT_WHEEL)),
                encoderTicksToInches(opmode.getEncoderValue(MotorName.CENTER_WHEEL))
        );
    }

    @Override
    public void update(RobotHardware robotHardware) {
        super.update();
    }


    private double previousHeading = 0;
    private double headingChange = 0;
    private double headingCompensation = 0;
    private double heading_deg = 0;
    // Unwrap Angle to be +- infinity
    @Override
    public Navigation2D getCurrentPosition() {
        Navigation2D currentPosition = toNav2dFromPose2d(getPoseEstimate());
        heading_deg = Math.toDegrees(currentPosition.theta);

        headingChange = heading_deg - previousHeading;
        previousHeading = Math.toDegrees(currentPosition.theta);
        headingCompensation = headingChange > 180 ? headingCompensation - 360 : (headingChange < -180 ? headingCompensation + 360 : headingCompensation);

        currentPosition.addInPlace(0,0,Math.toRadians(headingCompensation));
        return currentPosition;
    }

    @Override
    public void setCurrentPosition(Navigation2D currentPosition) {
        this.setPoseEstimate(toPose2dFromNav2d(currentPosition));
    }

    @Override
    public MecanumNavigation.Frame2D getRobotFrame() {
        return new Frame2D(getCurrentPosition());
    }
}
