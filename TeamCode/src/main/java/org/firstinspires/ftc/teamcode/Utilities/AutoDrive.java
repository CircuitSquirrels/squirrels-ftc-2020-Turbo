package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;

/**
 * Created by Ashley on 12/18/2017.
 */

public class AutoDrive {

    private RobotHardware opMode;
    private MecanumNavigation mecanumNavigation;
    public MecanumNavigation.Navigation2D lastTargetPosition = new MecanumNavigation.Navigation2D(0,0,0);


    public AutoDrive(RobotHardware opMode, MecanumNavigation mecanumNavigation) {
        this.opMode = opMode;
        this.mecanumNavigation = mecanumNavigation;
    }

    /**
     * Drive to position. Simple calculation, drives in single rotational arc.
     * In general, not the most efficient path.
     * @param targetPosition
     * @param rate
     * @return boolean, true if currentPosition is near targetPosition.
     */
    public boolean driveToPosition(MecanumNavigation.Navigation2D targetPosition, double rate) {
        lastTargetPosition = targetPosition;
        double distanceThresholdInches = 1;
        double angleThresholdRadians = 2 * (2*Math.PI/180);
        rate = Range.clip(rate,0,1);
        MecanumNavigation.Navigation2D currentPosition =
                (MecanumNavigation.Navigation2D)mecanumNavigation.currentPosition.clone();
        MecanumNavigation.Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);

        // Not Close enough to target, keep moving
        if ( Math.abs(deltaPosition.x) > distanceThresholdInches ||
                Math.abs(deltaPosition.y) > distanceThresholdInches ||
                Math.abs(deltaPosition.theta) > angleThresholdRadians) {

            Mecanum.Wheels wheels = mecanumNavigation.deltaWheelsFromPosition(targetPosition);
            wheels.scaleWheelPower(rate);
            opMode.setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Close enough
            opMode.setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }
    }

    public boolean rotateThenDriveToPosition(MecanumNavigation.Navigation2D targetPosition, double rate) {
        lastTargetPosition = targetPosition;
        double distanceThresholdInches = 0.5;
        double angleThresholdRadians = 2.0 * (Math.PI/180.0);
        rate = Range.clip(rate,0,1);
        MecanumNavigation.Navigation2D currentPosition = mecanumNavigation.currentPosition.copy();
        MecanumNavigation.Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);
        double deltaDistance = Math.sqrt( Math.pow(deltaPosition.x,2) + Math.pow(deltaPosition.y,2));

        double rateScale;
        // Not Close enough to target, keep moving
        if ( Math.abs(deltaPosition.theta) > angleThresholdRadians) {

            MecanumNavigation.Navigation2D rotationTarget = currentPosition.copy();
            rotationTarget.theta = targetPosition.theta; // Only rotate to the target at first.
            Mecanum.Wheels wheels = mecanumNavigation.deltaWheelsFromPosition(rotationTarget);
            rateScale = rampDown(Math.abs(deltaPosition.theta)*(180/Math.PI), 50, 0.8, 0.1);
            wheels = wheels.scaleWheelPower(rateScale * rate);
            opMode.setDriveForMecanumWheels(wheels);
            return false;
            // After rotating, begin correcting translation.
        } else if (Math.abs(deltaPosition.x) > distanceThresholdInches ||
                   Math.abs(deltaPosition.y) > distanceThresholdInches) {
            Mecanum.Wheels wheels = mecanumNavigation.deltaWheelsFromPosition(targetPosition);
            rateScale = rampDown(deltaDistance, 10, 1, 0.05);
            wheels = wheels.scaleWheelPower(rateScale * rate);
            opMode.setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Close enough
            opMode.setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }

    }
    public boolean driveToPositionTranslateOnly(MecanumNavigation.Navigation2D targetPosition, double rate) {
        return driveToPositionTranslateOnly(targetPosition, rate, 0.5);
    }

    public boolean driveToPositionTranslateOnly(MecanumNavigation.Navigation2D targetPosition, double rate, double distanceThresholdInches) {
        lastTargetPosition = targetPosition;
        double angleThresholdRadians = 2.0 * (Math.PI/180.0);
        rate = Range.clip(rate,0,1);
        MecanumNavigation.Navigation2D currentPosition = mecanumNavigation.currentPosition.copy();
        MecanumNavigation.Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);
        double deltaDistance = Math.sqrt( Math.pow(deltaPosition.x,2) + Math.pow(deltaPosition.y,2));

        double rateScale;
        // Not Close enough to target, keep moving
        if (Math.abs(deltaPosition.x) > distanceThresholdInches ||
                Math.abs(deltaPosition.y) > distanceThresholdInches ||
                Math.abs(deltaPosition.theta) > angleThresholdRadians) {
            MecanumNavigation.Navigation2D translationTarget = (MecanumNavigation.Navigation2D)currentPosition.clone();
            translationTarget.x = targetPosition.x;
            translationTarget.y = targetPosition.y;
            Mecanum.Wheels wheels = mecanumNavigation.deltaWheelsFromPosition(targetPosition);
            rateScale = rampDown(deltaDistance, 10, 1, 0.05);
            wheels = wheels.scaleWheelPower(rateScale * rate);
            opMode.setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Close enough
            opMode.setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }

    }

    static public double rampDown(double errSignal, double signalRampDownThreshold, double maxRatio, double minRatio) {
        // Error Check: Swap values to ensure max >= min
        if (maxRatio < minRatio) {
            double temp = maxRatio;
            maxRatio = minRatio;
            minRatio = temp;
        }

        double output;
        double standardOutput = 1.0;

        if (errSignal > signalRampDownThreshold) {
            output = standardOutput;
        } else {
            output = standardOutput * (minRatio + (maxRatio - minRatio) * errSignal/signalRampDownThreshold);
        }
        return Range.clip(Math.abs(output),0,1);
    }

    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power, int rampDistanceTicks) {
        power = Range.clip(Math.abs(power), 0, 1);
        int poweredDistance = 5;
        int arrivedDistance = 50;

        double maxRampPower = 1.0;
        double minRampPower = 0.0;
        int errorSignal = opMode.getEncoderValue(motorName) - targetTicks;
        double direction = errorSignal > 0 ? -1.0: 1.0;
        double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampDistanceTicks, maxRampPower, minRampPower);

        if (Math.abs(errorSignal) >= poweredDistance) {
            opMode.setPower(motorName, direction * power * rampDownRatio);
        } else {
            opMode.setPower(motorName, 0);
        }

        if(Math.abs(errorSignal) <= arrivedDistance) {
            return true;
        }else {
            return false;
        }
    }

    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power) {
        int rampDistanceTicks = 400;
        return driveMotorToPos (motorName,targetTicks,power,rampDistanceTicks);
    }

    // multi waypoint state properties
    String previousStateName_multiWaypoint = "thisIsEmpty";
    int currentDriveWaypoint = 0;


    public boolean multiWaypointState(String stateName_multiWaypoint, double speed, ArrayList<MecanumNavigation.Navigation2D> waypointList) {
        // Determine if this is a new call to the function, and state variables should be reset.
        // This functions as the initialization of the state variables.
        if (stateName_multiWaypoint.equals(previousStateName_multiWaypoint)) {
            currentDriveWaypoint = 0;
            previousStateName_multiWaypoint = stateName_multiWaypoint;
        }

        boolean arrived;
        boolean finalArrived = false;
        int lastIndex = waypointList.size() - 1;

        if (currentDriveWaypoint <= lastIndex){
            arrived = rotateThenDriveToPosition(waypointList.get(currentDriveWaypoint),speed);
            if(arrived && currentDriveWaypoint != lastIndex) {
                currentDriveWaypoint++;
            }
            if (arrived && currentDriveWaypoint == lastIndex) {
                finalArrived = true;
            }
        }

        if (finalArrived) {
            opMode.stopAllMotors();
        }
        opMode.telemetry.addData("Multi Waypoint Drive","");
        opMode.telemetry.addData("Current Waypoint: ", currentDriveWaypoint);
        opMode.telemetry.addData("Mini State Name", stateName_multiWaypoint);
        opMode.telemetry.addData("Target", waypointList.get(currentDriveWaypoint).toString());
        return finalArrived;
    }
}
