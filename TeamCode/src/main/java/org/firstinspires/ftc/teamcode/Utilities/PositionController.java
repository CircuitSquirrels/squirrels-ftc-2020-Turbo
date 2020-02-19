package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.DeadWheels.Localizer;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.*;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.miniPID.MiniPID;

public class PositionController {

    private RobotHardware opMode;
    private Localizer localizer;
    private MiniPID pidX;
    private MiniPID pidY;
    private MiniPID pidTheta;

    public Navigation2D absFramePower; // Power in field coordinates
    public Navigation2D robotFramePower; // Power in robot coordinates
    public Navigation2D target; // Public target

    double rampRateTranslation_powerPerSecond;
    double rampRateRotation_powerPerSecond;

    public PositionController(RobotHardware opMode, Localizer localizer) {
        this.opMode = opMode;
        this.localizer = localizer;
        initializePID();
    }

    public void initializePID() {
        // Linear Translation Parameters
        double p_linear = 0.1;
        double i_linear = 0.0;
        double d_linear = 0.1;
        double maxPower_linear = 1.0;
        rampRateTranslation_powerPerSecond = 3.0;

        // Rotation Parameters
        // Note that distances are in radians, so 90 degrees = 1.57
        double p_rotation = 0.7;
        double i_rotation = 0.0;
        double d_rotation = 0.3;
        double maxPower_rotation = 1.0;
        rampRateRotation_powerPerSecond = 3.0;

        // Setup
        pidX = new MiniPID(p_linear, i_linear, d_linear);
        pidY = new MiniPID(p_linear, i_linear, d_linear);
        pidTheta = new MiniPID(p_rotation, i_rotation, d_rotation);

        pidX.setOutputLimits(maxPower_linear);
        pidY.setOutputLimits(maxPower_linear);
        pidTheta.setOutputLimits(maxPower_rotation);

        // Just the initial setting, updateRampRate calibrates this in getOutput().
        pidX.setOutputRampRate(rampRateTranslation_powerPerSecond * .02);
        pidY.setOutputRampRate(rampRateTranslation_powerPerSecond * .02);
        pidTheta.setOutputRampRate(rampRateRotation_powerPerSecond * .02);
    }

    /**
     * the pid rate is per cycle, so that is being scaled
     */
    public void updateRampRate() {
        double rampRateTranslation = rampRateTranslation_powerPerSecond * Math.max(opMode.getAveragePeriodSec(), 0.01);
        double rampRateRotation = rampRateRotation_powerPerSecond * Math.max(opMode.getAveragePeriodSec(), 0.01);

        pidX.setOutputRampRate(rampRateTranslation);
        pidY.setOutputRampRate(rampRateTranslation);
        pidTheta.setOutputRampRate(rampRateRotation);
    }

    public void setTarget(Navigation2D target) {
        this.target = target; // For debugging.
        pidX.setSetpoint(target.x);
        pidY.setSetpoint(target.y);
        pidTheta.setSetpoint(target.theta);
    }

    public Navigation2D getOutput(Navigation2D actualPosition) {
        updateRampRate(); // Keeps rates scaled to average loop period
        return new Navigation2D(
                pidX.getOutput(actualPosition.x),
                pidY.getOutput(actualPosition.y),
                pidTheta.getOutput(actualPosition.theta));
    }

    public boolean driveTo(Navigation2D targetPosition, double rate) {
        Navigation2D positionError = targetPosition.substractAndReturn(localizer.getCurrentPosition());
        if(hasArrived(positionError, 0.5, Math.toRadians(5))) return true;
        setTarget(targetPosition);
        this.absFramePower = getOutput(localizer.getCurrentPosition());
        this.robotFramePower = toRobotFrameOrientation(absFramePower);
        opMode.setDriveForSimpleMecanum(-robotFramePower.y, -robotFramePower.x, -robotFramePower.theta, 0);
        return false;
    }

    public boolean hasArrived(Navigation2D positionError, double translationTolerance, double rotationTolerance) {
        return Math.abs(positionError.x) < translationTolerance && Math.abs(positionError.y) < translationTolerance && Math.abs(positionError.theta) < rotationTolerance;
    }

    public Navigation2D toRobotFrameOrientation(Navigation2D pointInFieldFrame) {
        Navigation2D pointInRobotFrame = pointInFieldFrame.copy();
        Navigation2D currentPosition = localizer.getCurrentPosition();
//        pointInRobotFrame.subtractInPlace(new Navigation2D(currentPosition.x, currentPosition.y, 0));
        pointInRobotFrame.rotateDegrees(Math.toDegrees(-currentPosition.theta));
        pointInRobotFrame.subtractInPlace(new Navigation2D(0, 0, -currentPosition.theta));
        return pointInRobotFrame;
    }
}
