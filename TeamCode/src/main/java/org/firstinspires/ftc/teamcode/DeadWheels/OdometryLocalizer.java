package org.firstinspires.ftc.teamcode.DeadWheels;

import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.*;

public class OdometryLocalizer {

    private OdometryConfig odometryConfig;

    // Positive center rotations are to the left
    private OdometryTicks deltaEncoderPosition = new OdometryTicks(0,0,0);
    private OdometryTicks encoderPosition = new OdometryTicks(0,0,0);
    private Navigation2D absolutePosition = new Navigation2D(0,0,0);

    OdometryLocalizer(OdometryConfig odometryConfig) {
        this.odometryConfig = odometryConfig;
    }

    public void setCurrentPosition(Navigation2D currentPosition) {
        absolutePosition = currentPosition;
    }

    public void setEncoderPosition(OdometryTicks encoderPosition) {
        this.encoderPosition = encoderPosition;
    }

    public boolean isInitialized() {
        return encoderPosition == null;
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
        double rotation_degrees_CCW = odometryConfig.inchesFromTicks((deltaTicks.right - deltaTicks.left) / 2.0) / (odometryConfig.wheelDiameter * Math.PI) * 360.0;
        double strafeCorrection_in = rotation_degrees_CCW * odometryConfig.strafeErrorPerDegrees;
        double strafe_in = odometryConfig.inchesFromTicks(deltaTicks.center) - strafeCorrection_in;

        return new Navigation2D(forward_in, strafe_in, rotation_degrees_CCW);
    }

    public Navigation2D calculateRelativeRobotMotionFromRotating(Navigation2D rotatingFrameMotion) {
        // Arc length
        double translationDistance = Math.sqrt(Math.pow(rotatingFrameMotion.x, 2.0) + Math.pow(rotatingFrameMotion.y, 2.0));
        // Direction of travel relative to forward
        double translationAngle = Math.atan2(rotatingFrameMotion.x, rotatingFrameMotion.y);

        double pathCircumference = Math.abs(translationDistance * (2.0 * Math.PI) / (rotatingFrameMotion.theta));
        double pathRadius = (pathCircumference / (Math.PI * 2.0));

        double deltaX = Math.abs(pathRadius * Math.sin(rotatingFrameMotion.theta));
        double thetaSign = rotatingFrameMotion.theta / Math.abs(rotatingFrameMotion.theta);
        double deltaY = Math.abs(pathRadius - pathRadius * Math.cos(rotatingFrameMotion.theta)) * thetaSign;

        // probably wrong
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
}
