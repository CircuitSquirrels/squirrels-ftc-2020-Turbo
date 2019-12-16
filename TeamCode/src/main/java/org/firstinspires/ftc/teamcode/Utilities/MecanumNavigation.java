package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.RobotHardware;

import java.text.DecimalFormat;

/**
 * Created by Ashley on 12/8/2017.
 */

public class MecanumNavigation {

    private RobotHardware opMode;
    public DriveTrainMecanum driveTrainMecanum;
    public Navigation2D currentPosition;
    public Navigation2D previousPosition;
    public WheelTicks wheelTicks;


    public MecanumNavigation(RobotHardware opMode, DriveTrainMecanum driveTrainMecanum) {
        this.opMode = opMode;
        this.driveTrainMecanum = driveTrainMecanum;
    }

    public void initialize(Navigation2D initialNavPosition, WheelTicks initialWheelTicks) {
        currentPosition = initialNavPosition;
        wheelTicks = initialWheelTicks;
    }

    public void initialize(Navigation2D initialNavPosition) {
        currentPosition = initialNavPosition;
        wheelTicks = getCurrentWheelTicks();
    }

    public void setCurrentPosition(Navigation2D currentPosition) {
        this.currentPosition = currentPosition;
    }

    public void update(WheelTicks newWheelTicks) {
        WheelTicks deltaWheelTicks = newWheelTicks.getDeltaFromPrevious(wheelTicks);
        this.wheelTicks = (WheelTicks)newWheelTicks.clone();

        Navigation2D deltaPosition = deltaPositionFromDeltaWheelTicks(deltaWheelTicks);
        this.previousPosition = (Navigation2D)currentPosition.clone();
        this.currentPosition.addRelativeDeltaToAbsolute(deltaPosition);
    }

    public void update() {
        update(new MecanumNavigation.WheelTicks(
                opMode.getEncoderValue(RobotHardware.MotorName.DRIVE_FRONT_LEFT),
                opMode.getEncoderValue(RobotHardware.MotorName.DRIVE_FRONT_RIGHT),
                opMode.getEncoderValue(RobotHardware.MotorName.DRIVE_BACK_LEFT),
                opMode.getEncoderValue(RobotHardware.MotorName.DRIVE_BACK_RIGHT)));
    }

    public void displayPosition() {
        opMode.telemetry.addData("X: ", opMode.df.format(currentPosition.x));
        opMode.telemetry.addData("Y: ", opMode.df.format(currentPosition.y));
        opMode.telemetry.addData("Theta: ", opMode.df.format(currentPosition.theta * 180 / Math.PI));
    }

    public Navigation2D deltaPositionFromDeltaWheelTicks(WheelTicks deltaWheelTicks) {
        double wheelRadius = driveTrainMecanum.wheelDiameter/2;
        double wheelbaseK = driveTrainMecanum.getK();

        double frontLeftRadians = driveTrainMecanum.ticksToRadians(deltaWheelTicks.frontLeft);
        double frontRightRadians = driveTrainMecanum.ticksToRadians(deltaWheelTicks.frontRight);
        double backLeftRadians = driveTrainMecanum.ticksToRadians(deltaWheelTicks.backLeft);
        double backRightRadians = driveTrainMecanum.ticksToRadians(deltaWheelTicks.backRight);

        double R_4 = wheelRadius / 4;

        double deltaX = (frontLeftRadians + frontRightRadians + backLeftRadians + backRightRadians)
                * R_4;
        double deltaY = (-frontLeftRadians + frontRightRadians + backLeftRadians - backRightRadians)
                * R_4;
        deltaY *= driveTrainMecanum.lateralScaling;
        double deltaTheta = (-frontLeftRadians + frontRightRadians - backLeftRadians + backRightRadians)
                * R_4 / wheelbaseK;

        return new Navigation2D(deltaX,deltaY,deltaTheta);
    }

    /**
     * Return a wheels object to move from the current position to the target position.
     * @param targetPosition Navigation2D
     * @return Wheels power object, scaled to 1.
     */
    public Mecanum.Wheels deltaWheelsFromPosition(Navigation2D targetPosition) {
        double deltaX = targetPosition.x - currentPosition.x;
        double deltaY = targetPosition.y - currentPosition.y;
        double deltaTheta = targetPosition.theta - currentPosition.theta;

        double bodyX = deltaX * Math.cos(currentPosition.theta) + deltaY * Math.sin(currentPosition.theta);
        double bodyY = -deltaX * Math.sin(currentPosition.theta) + deltaY * Math.cos(currentPosition.theta);
        double bodyTheta = deltaTheta;

        return deltaWheelsFromBodyRelativeMotion(new Navigation2D(bodyX, bodyY, bodyTheta));
    }


    /**
     * Return a wheels object to move in the 3 specified body relative directions.
     * Note that the direction of X and Y are always relative to the robot body,
     * even as it rotates.
     * @param bodyRelativeMovement Nav2d, a body relative movement, non-euclidean.
     * @return Wheels power object, scaled to 1.
     */
    public Mecanum.Wheels deltaWheelsFromBodyRelativeMotion(Navigation2D bodyRelativeMovement) {
        double deltaX = bodyRelativeMovement.x;
        double deltaY = bodyRelativeMovement.y;
        deltaY /= driveTrainMecanum.lateralScaling;
        double deltaTheta = bodyRelativeMovement.theta;

        double K = driveTrainMecanum.getK();

        double R_inv = 1.0 / (driveTrainMecanum.wheelDiameter/2);

        // remove time by multiplying by the elapsed time
        double frontLeft = driveTrainMecanum.radiansToTicks(R_inv * (deltaX - deltaY - K * deltaTheta));
        double backLeft = driveTrainMecanum.radiansToTicks(R_inv * (deltaX + deltaY - K * deltaTheta));
        double backRight = driveTrainMecanum.radiansToTicks(R_inv * (deltaX - deltaY + K * deltaTheta));
        double frontRight = driveTrainMecanum.radiansToTicks(R_inv * (deltaX + deltaY + K * deltaTheta));

        Mecanum.Wheels wheels =  new Mecanum.Wheels(frontLeft, frontRight, backLeft, backRight);
        wheels.coupledScaleToOne();
        return wheels;
    }


    /**
     * 2d position plus angular orientation.
     * x is in inches, forward
     * y is in inches, left
     * theta is in radians, CCW
     */
    public static class Navigation2D implements Cloneable{
        public double x = 0;
        public double y = 0;
        // Rotation degrees CCW
        public double theta = 0;

        /**
         *
         * @param x inches
         * @param y inches
         * @param theta radians CCW
         */
        public Navigation2D(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }

        public void rotate (double degrees) {
            double radians = degrees * Math.PI / 180;
            this.x = x * Math.cos(radians) - y * Math.sin(radians);
            this.y = x * Math.sin(radians) + y * Math.cos(radians);
            this.theta += radians;
        }

        public void reflectInX() {
            this.y = -y;
            this.theta = -theta; // Allows negative angles.
        }

        public Navigation2D rotateCopy (double degrees) {
            Navigation2D nav2D_copy = (Navigation2D) this.clone();
            nav2D_copy.rotate(degrees);
            return nav2D_copy;
        }

        // Simple clone.
        public Navigation2D(Navigation2D navigation2D) {
            this(navigation2D.x, navigation2D.y, navigation2D.theta);
        }

        public Navigation2D copy() {
            return (Navigation2D) this.clone();
        }

        public void addInPlace(Navigation2D other) {
            this.x += other.x;
            this.y += other.y;
            this.theta += other.theta;
        }

        public void addInPlace(double x, double y, double theta) {
            this.x += x;
            this.y += y;
            this.theta += theta;
        }

        public Navigation2D addAndReturn(Navigation2D other) {
            return new Navigation2D(this.x + other.x, this.y + other.y, this.theta + other.theta);
        }

        public Navigation2D addAndReturn(double x, double y, double theta) {
            return new Navigation2D(this.x + x, this.y + y, this.theta + theta);
        }

        public void subtractInPlace(Navigation2D other) {
            this.x -= other.x;
            this.y -= other.y;
            this.theta -= other.theta;
        }

        /**
         * Returns this object minus argument.
         * @param other Navigation2D object
         * @return this minus argument
         */
        public Navigation2D minusEquals (Navigation2D other) {
            return new Navigation2D(this.x - other.x,
                                    this.y - other.y,
                                 radianRound(this.theta - other.theta));
        }

        // Distance formula
        public double distanceTo(Navigation2D other) {
            return Math.sqrt( Math.pow(other.x - this.x,2) + Math.pow(other.y - this.y,2));
        }

        public double angleDegreesTo(Navigation2D other) {
            return (other.theta - this.theta) * 180 / Math.PI;
        }

        /**
         * When "this" Navigation2D instance refers to an absolute position,
         * and the argument,
         * @param deltaRelativeNav
         * refers to a delta movement in the robot relative coordinate frame,
         * this method transforms the relative movement into the absolute coordinate frame
         * and adds it to the absolute position.
         */
        public void addRelativeDeltaToAbsolute(Navigation2D deltaRelativeNav) {
            double PHASE_ROTATION = 0;

            double absoluteX = this.x;
            double absoluteY = this.y;
            double absoluteTheta = this.theta;

            absoluteX +=
                    + deltaRelativeNav.x * Math.cos(absoluteTheta + PHASE_ROTATION)
                    - deltaRelativeNav.y * Math.sin(absoluteTheta + PHASE_ROTATION);
            absoluteY +=
                    + deltaRelativeNav.x * Math.sin(absoluteTheta + PHASE_ROTATION)
                    + deltaRelativeNav.y * Math.cos(absoluteTheta + PHASE_ROTATION);
            absoluteTheta += deltaRelativeNav.theta;

            this.x = absoluteX;
            this.y = absoluteY;
            this.theta = absoluteTheta;
        }

        @Override
        public Object clone()
        {
            try {
                return super.clone();
            } catch (CloneNotSupportedException e) {
                e.printStackTrace();
            }
            return null;
        }

        public String toString() {
//            DecimalFormat df = new DecimalFormat(" 00.00;-00.00");
//            DecimalFormat df_deg = new DecimalFormat(" 00.0;-00.0");
//            return df.format(x) + ",  " + df.format(y) + ", " + df_deg.format(theta*180/Math.PI) + " deg";
            String format_xy = "%6.2f";
            String format_deg = "%6.1f";
            return String.format(format_xy,x) + ",  " + String.format(format_xy,y) + ", " + String.format(format_deg,theta*180/Math.PI) + " deg";
        }
    }


    /**
     * Defines navigation relevant qualities of a Mecanum drive train.
     */
    public static class DriveTrainMecanum{
        public double wheelbaseLength;
        public double wheelbaseWidth;
        public double wheelDiameter;
        public double encoderTicksPerRotation;
        public double lateralScaling = 1.0;

        public DriveTrainMecanum(double wheelbaseLength, double wheelbaseWidth,
                                 double wheelDiameter, int encoderTicksPerRotation) {
            this.wheelbaseLength = wheelbaseLength;
            this.wheelbaseWidth = wheelbaseWidth;
            this.wheelDiameter = wheelDiameter;
            this.encoderTicksPerRotation = encoderTicksPerRotation;
        }

        public DriveTrainMecanum(double wheelbaseLength, double wheelbaseWidth,
                                 double wheelDiameter, double encoderTicksPerRotation, double lateralScaling) {
            this.wheelbaseLength = wheelbaseLength;
            this.wheelbaseWidth = wheelbaseWidth;
            this.wheelDiameter = wheelDiameter;
            this.encoderTicksPerRotation = encoderTicksPerRotation;
            this.lateralScaling = lateralScaling;
        }

        public double getK() {
            return Math.abs(wheelbaseLength/2) + Math.abs(wheelbaseWidth/2);
        }

        public double ticksToRadians(int deltaTicks) {
            return  2 * Math.PI * deltaTicks / encoderTicksPerRotation;
        }

        public double radiansToTicks(double radians) {
            return radians / (2 * Math.PI) * encoderTicksPerRotation;
        }
    }

    /**
     * Track encoder values of each Mecanum wheel.
     */
    public static class WheelTicks implements Cloneable
    {
        public int frontLeft;
        public int frontRight;
        public int backLeft;
        public int backRight;

        public WheelTicks(int frontLeft, int frontRight, int backLeft, int backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }

        public WheelTicks getDeltaFromPrevious(WheelTicks previous) {
            return new WheelTicks(this.frontLeft - previous.frontLeft,
                                this.frontRight -  previous.frontRight,
                                this.backLeft - previous.backLeft,
                                this.backRight - previous.backRight);
        }

        @Override
        public Object clone()
        {
            try {
                return super.clone();
            } catch (CloneNotSupportedException e) {
                e.printStackTrace();
            }
            return null;
        }
    }

    // Quality of life function for enclosing class.
    public WheelTicks getCurrentWheelTicks() {
        return new MecanumNavigation.WheelTicks(
                opMode.getEncoderValue(RobotHardware.MotorName.DRIVE_FRONT_LEFT),
                opMode.getEncoderValue(RobotHardware.MotorName.DRIVE_FRONT_RIGHT),
                opMode.getEncoderValue(RobotHardware.MotorName.DRIVE_BACK_LEFT),
                opMode.getEncoderValue(RobotHardware.MotorName.DRIVE_BACK_RIGHT));
    }


    /**
     * Round an angle in radians so it falls within [-PI, PI]
     * @param radians
     * @return
     */
    static public double radianRound(double radians) {
        while (radians > Math.PI) {
            radians -= 2*Math.PI;
        }
        while (radians < - Math.PI) {
            radians += 2*Math.PI;
        }
        return radians;
    }


    public double boundDegrees(double angleDegrees) {
        while (Math.abs(angleDegrees) > 180) {
            if (angleDegrees < -180) {
                angleDegrees += 360;
            } else {
                angleDegrees -= 360;
            }
        }
        return angleDegrees;
    }
}
