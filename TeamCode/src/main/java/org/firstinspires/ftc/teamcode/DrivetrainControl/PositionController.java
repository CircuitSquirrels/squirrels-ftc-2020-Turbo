package org.firstinspires.ftc.teamcode.DrivetrainControl;

import org.firstinspires.ftc.teamcode.DeadWheels.Localizer;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.*;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.miniPID.MiniPID;
import org.firstinspires.ftc.teamcode.miniPID.MiniPID.MiniPIDConfiguration;

public class PositionController {

    private RobotHardware opMode;
    private Localizer localizer;
    private MiniPID pidX;
    private MiniPID pidY;
    private MiniPID pidTheta;

    public Navigation2D absFramePower = new Navigation2D(0,0,0);// Power in field coordinates
    public Navigation2D robotFramePower = new Navigation2D(0,0,0); // Power in robot coordinates
    public Mecanum.Command movementCommand = new Mecanum.Command(0,0,0);
    public Navigation2D target; // Public target

    double rampRateTranslation_powerPerSecond;
    double rampRateRotation_powerPerSecond;

    public PositionController(RobotHardware opMode, Localizer localizer) {
        this.opMode = opMode;
        this.localizer = localizer;
        initializePID();
    }


    // Could activate and deactivate I when very near target.
    // Ex: I not active until target is overshot, then allowed to accumulate.
    // Everytime setpoint is overshot, error is purged. (reset? It blanks I and D)
    // I is only active and accumulating when within a 'basket' ?
    public void initializePID() {
        // Linear Translation Parameters
        MiniPIDConfiguration translationPIDConfig = new MiniPIDConfiguration
                (0.05, 0.0, 0.0, 0.8, 0.3, 2.0, 0.5);

        // Rotation Parameters
        // Note that distances are in radians, so 90 degrees = 1.57
        MiniPIDConfiguration rotationPIDConfig = new MiniPIDConfiguration
                (0.05, 0.0, 0.0, 0.7, 0.3, 2.0, Math.toRadians(5.0));

        PositionControllerConfiguration basicConfig = new PositionControllerConfiguration(translationPIDConfig,rotationPIDConfig);
        setPIDParameters(basicConfig);
    }

    public void setPIDParameters(PositionControllerConfiguration config) {

        // Setup
        pidX = new MiniPID(config.translationPIDConfig.P, config.translationPIDConfig.I, config.translationPIDConfig.D);
        pidY = new MiniPID(config.translationPIDConfig.P, config.translationPIDConfig.I, config.translationPIDConfig.D);
        pidTheta = new MiniPID(config.rotationPIDConfig.P, config.rotationPIDConfig.I, config.rotationPIDConfig.D);

        pidX.setOutputLimits(config.translationPIDConfig.maxOutput);
        pidY.setOutputLimits(config.translationPIDConfig.maxOutput);
        pidTheta.setOutputLimits(config.rotationPIDConfig.maxOutput);

        pidX.setMaxIOutput(config.translationPIDConfig.maxIOutput);
        pidY.setMaxIOutput(config.translationPIDConfig.maxIOutput);
        pidTheta.setMaxIOutput(config.rotationPIDConfig.maxIOutput);

        pidX.positionTolerance = config.translationPIDConfig.positionTolerance;
        pidY.positionTolerance = config.translationPIDConfig.positionTolerance;
        pidTheta.positionTolerance = config.rotationPIDConfig.positionTolerance;

        this.rampRateTranslation_powerPerSecond = config.translationPIDConfig.rampRate;
        this.rampRateRotation_powerPerSecond = config.rotationPIDConfig.rampRate;
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
        if(hasArrived(positionError, pidX.positionTolerance, pidTheta.positionTolerance)) return true;
        setTarget(targetPosition);
        this.absFramePower = getOutput(localizer.getCurrentPosition());
        this.robotFramePower = toRobotFrameOrientation(absFramePower);
        movementCommand = new Mecanum.Command(robotFramePower.x,robotFramePower.y,robotFramePower.theta);
        movementCommand.normalizeJointly();
        opMode.setDriveForMecanumCommand(movementCommand);
//        opMode.setDriveForSimpleMecanum(-robotFramePower.y, -robotFramePower.x, -robotFramePower.theta, 0);
        return false;
    }

    public boolean hasArrived(Navigation2D positionError, double translationTolerance, double rotationTolerance) {
        return Math.abs(positionError.x) < translationTolerance && Math.abs(positionError.y) < translationTolerance && Math.abs(positionError.theta) < rotationTolerance;
    }

    public Navigation2D toRobotFrameOrientation(Navigation2D pointInFieldFrame) {
        Navigation2D pointInRobotFrame = pointInFieldFrame.copy();
        Navigation2D currentPosition = localizer.getCurrentPosition();
        pointInRobotFrame.rotateDegrees(Math.toDegrees(-currentPosition.theta));
        pointInRobotFrame.subtractInPlace(new Navigation2D(0, 0, -currentPosition.theta));
        return pointInRobotFrame;
    }

    public String getErrorSum() {
        String format_error = "%6.2f";
        return String.format(format_error,pidX.getErrorSum()) + ",  " + String.format(format_error,pidY.getErrorSum()) + ", " + String.format(format_error,pidTheta.getErrorSum()) + "";
    }

    static public class PositionControllerConfiguration {
        private MiniPIDConfiguration translationPIDConfig;
        private MiniPIDConfiguration rotationPIDConfig;

        public PositionControllerConfiguration(MiniPIDConfiguration translationPIDConfig, MiniPIDConfiguration rotationPIDConfig) {
            this.translationPIDConfig = translationPIDConfig;
            this.rotationPIDConfig = rotationPIDConfig;

        }
    }
}
