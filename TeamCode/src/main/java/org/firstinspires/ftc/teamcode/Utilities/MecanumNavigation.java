package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.RobotHardware;

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
        private String label = "";

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


        public Navigation2D(double x, double y, double theta, Frame2D referenceFrame) {
            this(x,y,theta);
            this.referenceFrame = referenceFrame;
        }


        public void rotateDegrees(double degrees) {
            double radians, xNew, yNew, thetaNew;
            radians = degrees * Math.PI / 180;
            xNew = x * Math.cos(radians) - y * Math.sin(radians);
            yNew = x * Math.sin(radians) + y * Math.cos(radians);
            thetaNew = this.theta + radians;
            this.x = xNew;
            this.y = yNew;
            this.theta = thetaNew;
        }

        public void reflectInX() {
            this.y = -y;
            this.theta = -theta; // Allows negative angles.
        }

        public Navigation2D rotateCopy (double degrees) {
            Navigation2D nav2D_copy = (Navigation2D) this.clone();
            nav2D_copy.rotateDegrees(degrees);
            return nav2D_copy;
        }

        // Simple clone.
        public Navigation2D(Navigation2D navigation2D) {
            this(navigation2D.x, navigation2D.y, navigation2D.theta);
            this.referenceFrame = navigation2D.referenceFrame;
        }

        public Navigation2D(Navigation2D navigation2D, Frame2D referenceFrame) {
            this(navigation2D.x, navigation2D.y, navigation2D.theta);
            this.referenceFrame = referenceFrame; // DEBUG: Not copied, reference may stay connected
        }

        public Navigation2D copy() {
            if(this.label == "") {
                return this.copyAndLabel("");
            } else {
                return this.copyAndLabel(label.concat("_copy"));
            }
        }

        public Navigation2D copyAndLabel(String newLabel) {
            Navigation2D copied_n2d = new Navigation2D(this.x,this.y,this.theta);
            copied_n2d.label = newLabel;
            if(this.referenceFrame == null) {
                copied_n2d.referenceFrame = null;
            } else {
                copied_n2d.referenceFrame = this.referenceFrame.copy();
            }
            return copied_n2d;
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

        public Navigation2D substractAndReturn(double x, double y, double theta) {
            return new Navigation2D(this.x - x, this.y - y, this.theta - theta);
        }

        public Navigation2D substractAndReturn(Navigation2D other) {
            return new Navigation2D(this.x - other.x, this.y - other.y, this.theta - other.theta);
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

        public void setLabel(String label) {
            this.label = label;
        }

        public String getLabel() {
            return label;
        }

        @Override
        public Object clone()
        {
            try {
                Navigation2D temp_n2d = (Navigation2D) super.clone();
                temp_n2d.setLabel(temp_n2d.label.concat("_copy"));
                return temp_n2d;
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

        // Default value of reference frame is null, which indicates absolute, or world frame.
        public Frame2D referenceFrame = null;

        // Methods for getting parent

        public Navigation2D getNav2DInParentFrame() {
            Navigation2D pointInParentFrame = this.copy();
            // If reference frame is null, we are in the world frame with no parent, and can just return a copy.
            if (pointInParentFrame.referenceFrame == null) return pointInParentFrame;

            // Otherwise, if referenceFrame is not null, the following transformation is required:
            // Rotate the point by the negative of the frame angle to get orientation to 0
            pointInParentFrame.rotateDegrees(Math.toDegrees(referenceFrame.positionInReferenceFrame.theta));
            // Translate the point by the x and y of the frame in its reference.
            pointInParentFrame.x += referenceFrame.positionInReferenceFrame.x;
            pointInParentFrame.y += referenceFrame.positionInReferenceFrame.y;
            // Adjust referenceFrame to match
            if (pointInParentFrame.referenceFrame.referenceFrame == null) {
                pointInParentFrame.referenceFrame = null;
            } else {
                pointInParentFrame.referenceFrame = pointInParentFrame.referenceFrame.referenceFrame.copy();
            }

            return pointInParentFrame;
        }

        public Navigation2D getNav2DInWorldFrame() {
            Navigation2D pointInNewFrame = this.copy();
            // When the parent is null, we are in the world frame.
            while(pointInNewFrame.referenceFrame != null) {
                pointInNewFrame = pointInNewFrame.getNav2DInParentFrame();
            }
            return pointInNewFrame;
        }

        public Navigation2D getNav2DInLocalFrame(Frame2D localFrame) {
            Navigation2D localPoint = this.copy();
            Navigation2D globalPoint = this.getNav2DInWorldFrame(); // Null referenceFrame

            // If localFrame is really the world frame, then return the world frame.
            if(localFrame == null) return this.getNav2DInWorldFrame();

            // Get position of local frame origin in world frame
            Navigation2D localFrameOriginInWorld_N2D = localFrame.positionInReferenceFrame.copy();
            localFrameOriginInWorld_N2D.referenceFrame = localFrame.referenceFrame;
            localFrameOriginInWorld_N2D = localFrameOriginInWorld_N2D.getNav2DInWorldFrame();

            // Apply shift then rotation to move globalPoint into the new localfFrameOriginInWorld
            localPoint.x = globalPoint.x - localFrameOriginInWorld_N2D.x;
            localPoint.y = globalPoint.y - localFrameOriginInWorld_N2D.y;
            // It is important that the initial theta before applying this rotation come from the
            // original heading.
            localPoint.rotateDegrees(-Math.toDegrees(localFrameOriginInWorld_N2D.theta));
            // Correct localFrame set
            localPoint.referenceFrame = localFrame; // TODO DEBUG Frame Not copied


            return localPoint;
        }
    }

    /**
     * Represents the frame that a Navigation2D point is referenced in. x,y,theta is given in the
     * reference coordinates.
     */
    public static class Frame2D {
        public Navigation2D positionInReferenceFrame = new Navigation2D(0,0,0);
        public Frame2D referenceFrame = null;

        public Frame2D() {
            this.positionInReferenceFrame = new Navigation2D(0,0,0);
            this.referenceFrame = null;
        }

        public Frame2D(Navigation2D positionInReferenceFrame, Frame2D referenceFrame) {
            this.positionInReferenceFrame = positionInReferenceFrame.copy();
            // Handle potential null reference frame in a safe way, while terminating recursion.
            if (referenceFrame == null) {
                this.referenceFrame = null;  // Breaks recursion, null safe
            } else {
                this.referenceFrame = referenceFrame.copy();
            }
        }

        public Frame2D(Navigation2D positionInReferenceFrame) {
            this(positionInReferenceFrame,null);
        }

        public Frame2D(double x, double y, double theta) {
            this(new Navigation2D(x,y,theta),null);
        }

        public Frame2D copy() {
            return new Frame2D(this.positionInReferenceFrame, this.referenceFrame);
        }

        public boolean isWorldFrame() {
            return this.referenceFrame == null;
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
