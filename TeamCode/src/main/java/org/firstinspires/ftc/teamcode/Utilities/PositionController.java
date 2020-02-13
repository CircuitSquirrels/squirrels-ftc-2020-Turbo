package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.*;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.miniPID.MiniPID;

public class PositionController {

    private RobotHardware opMode;
    private MecanumNavigation mecanumNavigation;
    private MiniPID pidX;
    private MiniPID pidY;
    private MiniPID pidTheta;

    public PositionController(RobotHardware opMode, MecanumNavigation mecanumNavigation) {
        this.opMode = opMode;
        this.mecanumNavigation = mecanumNavigation;
        initializePID();
    }

    public void initializePID() {
        double p = 0.1;
        double maxPower = 1.0;
        double rampRateTranslation = 1;
        double rampRateRotation = 1;
        pidX = new MiniPID(p, 0, 0);
        pidY = new MiniPID(p,0, 0);
        pidTheta = new MiniPID(p, 0, 0);

        pidX.setOutputLimits(maxPower);
        pidY.setOutputLimits(maxPower);
        pidTheta.setOutputLimits(maxPower);

        pidX.setOutputRampRate(rampRateTranslation);
        pidY.setOutputRampRate(rampRateTranslation);
        pidTheta.setOutputRampRate(rampRateRotation);
    }

    public void setTarget(Navigation2D target) {
        pidX.setSetpoint(target.x);
        pidY.setSetpoint(target.y);
        pidTheta.setSetpoint(target.theta);
    }

    public Navigation2D getOutput(Navigation2D actualPosition) {
        return new Navigation2D(pidX.getOutput(actualPosition.x), pidY.getOutput(actualPosition.y), pidTheta.getOutput(actualPosition.theta));
    }

    public boolean driveTo(Navigation2D targetPosition, double rate) {
        Navigation2D positionError = targetPosition.substractAndReturn(mecanumNavigation.currentPosition);

        if(hasArrived(positionError, 0.5, Math.toRadians(5))) return true;

        Navigation2D absPower;
        Navigation2D robotFramePower;

        setTarget(targetPosition);
        absPower = getOutput(mecanumNavigation.currentPosition);
        robotFramePower = toRobotFrame(absPower);
        opMode.setDriveForSimpleMecanum(robotFramePower.y, -robotFramePower.x, robotFramePower.theta, 0);
        return false;
    }

    public boolean hasArrived(Navigation2D positionError, double translationTolerance, double rotationTolerance) {
        return Math.abs(positionError.x) < translationTolerance && Math.abs(positionError.y) < translationTolerance && Math.abs(positionError.theta) < rotationTolerance;
    }

    public Navigation2D toRobotFrame(Navigation2D pointInFieldFrame) {
        Navigation2D pointInRobotFrame = pointInFieldFrame.copy();
        pointInRobotFrame.subtractInPlace(new Navigation2D(mecanumNavigation.currentPosition.x, mecanumNavigation.currentPosition.y, 0));
        pointInRobotFrame.rotateDegrees(-mecanumNavigation.currentPosition.theta);
        return pointInRobotFrame;
    }
}
