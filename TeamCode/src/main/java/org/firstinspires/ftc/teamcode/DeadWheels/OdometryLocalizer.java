package org.firstinspires.ftc.teamcode.DeadWheels;

import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.*;

public class OdometryLocalizer {

    private OdometryConfig odometryConfig;

    // Positive center rotations are to the left
    private OdometryTicks deltaEncoderPosition = new OdometryTicks(0,0,0);
    private OdometryTicks encoderPosition = new OdometryTicks(0,0,0);
    private Navigation2D absolutePosition = new Navigation2D(0,0,0);

    public OdometryLocalizer(OdometryConfig odometryConfig) {
        this.odometryConfig = odometryConfig;
    }

    public void update(OdometryTicks newTicks) {
        Navigation2D rotatingFrameMotion;
        Navigation2D deltaPositionInRobotFrame;

        deltaEncoderPosition = newTicks.subtractAndReturn(encoderPosition);
        encoderPosition = newTicks;

        rotatingFrameMotion = calculateRotatingFrameMotion(deltaEncoderPosition);
        deltaPositionInRobotFrame = calculateRelativeRobotMotionFromRotating(rotatingFrameMotion);
        absolutePosition = calculateNewAbsolutePositionFromDelta(deltaPositionInRobotFrame);
    }

    public Navigation2D calculateRotatingFrameMotion(OdometryTicks deltaTicks) {
        double forward_in = odometryConfig.inchesFromTicks((deltaTicks.right + deltaTicks.left) / 2.0);
        double rotation_degrees_CCW = odometryConfig.inchesFromTicks((deltaTicks.right - deltaTicks.left) / 2.0) /
                (odometryConfig.outerWheelDistance * Math.PI) * 360.0;
        double strafeCorrection_in = rotation_degrees_CCW * odometryConfig.strafeErrorPerDegrees;
        double strafe_in = odometryConfig.inchesFromTicks(deltaTicks.center) - strafeCorrection_in;

        return new Navigation2D(forward_in, strafe_in, rotation_degrees_CCW * Math.PI / 180.0);
    }

    public Navigation2D calculateRelativeRobotMotionFromRotating(Navigation2D rotatingFrameMotion) {
        // Arc length
        double translationDistance = Math.sqrt(Math.pow(rotatingFrameMotion.x, 2.0) + Math.pow(rotatingFrameMotion.y, 2.0));
        // Direction of travel relative to forward
        double translationAngle = Math.atan2(rotatingFrameMotion.y, rotatingFrameMotion.x);

        double deltaX, deltaY;
        // Start by assuming all motion is in the X (forward) direction
        // If theta does not equal 0 then calculate path radius
        if(rotatingFrameMotion.theta != 0) {
            double pathCircumference = Math.abs(translationDistance * (2.0 * Math.PI) / (rotatingFrameMotion.theta));
            double pathRadius = (pathCircumference / (Math.PI * 2.0));

            deltaX = Math.abs(pathRadius * Math.sin(rotatingFrameMotion.theta));
            double thetaSign = rotatingFrameMotion.theta / Math.abs(rotatingFrameMotion.theta);
            deltaY = Math.abs(pathRadius - pathRadius * Math.cos(rotatingFrameMotion.theta)) * thetaSign;
        } else {
            deltaX = translationDistance;
            deltaY = 0.0;
        }

        Navigation2D robotFrameOffset = new  Navigation2D(deltaX,deltaY,rotatingFrameMotion.theta);
        robotFrameOffset.rotateDegrees(Math.toDegrees(translationAngle));
        robotFrameOffset.addInPlace(0,0,-translationAngle);

        return robotFrameOffset;
    }

    public Navigation2D calculateNewAbsolutePositionFromDelta(Navigation2D deltaPosition) {
        Frame2D oldRobotPosition = new Frame2D(absolutePosition.copy());
        deltaPosition.referenceFrame = oldRobotPosition;
        return deltaPosition.getNav2DInWorldFrame();
    }

    public void setCurrentPosition(Navigation2D currentPosition) {
        this.absolutePosition = currentPosition;
    }

    public void setEncoderPosition(OdometryTicks encoderPosition) {
        this.encoderPosition = encoderPosition;

    }

    public boolean isInitialized() {
        return encoderPosition == null;
    }

    public Navigation2D getCurrentPosition() {
        return absolutePosition;
    }

    public OdometryTicks getEncoderPosition() {
        return encoderPosition;
    }

    public Frame2D getRobotFrame() {
        return new Frame2D(absolutePosition);
    }
}