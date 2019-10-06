package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.VectorMath;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Vector;

public class RobotHardware extends OpMode {

    // All motors on the robot, in order of MotorName.
    private ArrayList<DcMotor> allMotors;

    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;

    // Names of only the drive motors.
    private ArrayList<MotorName> driveMotorNames;

    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;

    // Format for displaying decimals.
    public DecimalFormat df;
    public DecimalFormat df_prec;


    // IMU reference
    public BNO055IMU imu;

    // Execution cycle period monitor.
    private ElapsedTime period = new ElapsedTime();
    private Vector<Double> pastPeriods = new Vector<Double>();


    // The motors on the robot.
    public enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
        ARM,
        WRIST,
        FEEDER,
        LIFT_WINCH,
    }

    /**
     * Sets the power of the motor.
     *
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    public void setPower(MotorName motor, double power) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name() + ": " + df.format(power));
        } else {
            m.setPower(power);
        }
    }

    /**
     * Get motor power
     *
     * @param motor MotorName
     * @return Motor Power, or zero if it cannot be found
     */
    public double getPower(MotorName motor) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
            return 0;
        } else {
            return m.getPower();
        }
    }

    /**
     * Gets the encoder value of the motor.
     *
     * @param motor MotorName enum value.
     * @return integer encoder position in ticks.
     */
    public int getEncoderValue(MotorName motor) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
            return 0;
        } else {
            return m.getCurrentPosition();
        }
    }

    /**
     * Stops all motors.
     */
    public void stopAllMotors() {
        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
    }

    public void resetAndStopAllMotors() {
        for (MotorName name : MotorName.values()) {
            DcMotor motor = allMotors.get(name.ordinal());
            if (motor == null) {
                telemetry.addData("Motor missing" ,name.name());
            } else {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    /**
     * Set all four drive motors to the same runMode.
     * Options are RUN_WITHOUT_ENCODER, RUN_USING_ENCODER,
     * RUN_TO_POSITION is used with setTargetPosition()
     * STOP_AND_RESET_ENCODER
     *
     * @param runMode
     */
    protected void setDriveMotorsRunMode(DcMotor.RunMode runMode) {
        for (MotorName motor : driveMotorNames) {
            DcMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing", motor.name());
            } else {
                m.setMode(runMode);
            }
        }
    }

    /**
     * Set zero power braking behavior for all drive wheels.
     *
     * @param zeroPowerBraking boolean to activate or deactivate zero power braking
     */
    protected void setDriveMotorsZeroPowerBraking(boolean zeroPowerBraking) {
        DcMotor.ZeroPowerBehavior brakingMode = zeroPowerBraking ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        for (MotorName motor : driveMotorNames) {
            DcMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing", motor.name());
            } else {
                m.setZeroPowerBehavior(brakingMode);
            }
        }
    }


    /**
     * Sets the drive chain power.
     *
     * @param left  The power for the left two motors.
     * @param right The power for the right two motors.
     */
    public void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }

    /**
     * Apply motor power matching the wheels object
     * @param wheels Provides all four mecanum wheel powers, [-1, 1]
     */
    public void setDriveForMecanumWheels(Mecanum.Wheels wheels) {
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    /**
     * Sets mecanum drive chain power using simplistic calculations.
     * @param leftStickX Unmodified Gamepad leftStickX inputs.
     * @param leftStickY Unmodified Gamepad leftStickY inputs.
     * @param rightStickX Unmodified Gamepad rightStickX inputs.
     * @param rightStickY Unmodified Gamepad rightStickY inputs.
     */
    public void setDriveForSimpleMecanum(double leftStickX, double leftStickY,
                                         double rightStickX, double rightStickY) {
        Mecanum.Wheels wheels = Mecanum.simpleJoystickToWheels (leftStickX, leftStickY, rightStickX, rightStickY);
        setDriveForMecanumWheels(wheels);
    }



    // The servos on the robot.
    public enum ServoName {
//        FLIPPER_RIGHT,
//        FLIPPER_LEFT,
        FEEDER_LIFTER,
    }

    // Servo methods

    /**
     * Sets the angle of the servo.
     *
     * @param servo    The servo to modify.
     * @param position The angle to set [0, 1].
     */
    public void setAngle(ServoName servo, double position) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name() + ": " + df.format(position));
        } else {
            s.setPosition(position);
        }
    }

    /**
     * Get the position of a servo.
     *
     * @param servo ServoName enum to check
     * @return double servo position [0,1]
     */
    public double getAngle(ServoName servo) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name());
            return -1;
        } else {
            return s.getPosition();
        }
    }

    public boolean moveServoAtRate(ServoName servo, double targetPos, double rate) {
        boolean isMovementDone;
        double distanceThreshold = 0.05;
        rate = Range.clip(rate,0,10);
        targetPos = Range.clip(targetPos,0,1);
        double currentPosition = getAngle(servo);
        double distance = targetPos - currentPosition;
        double direction = targetPos > currentPosition ? 1 : -1;
        double nextPosition;
        if ( Math.abs(distance) > distanceThreshold ) {
            nextPosition = rate * direction * getLastPeriodSec() + currentPosition;
            isMovementDone = false;
        } else {
            nextPosition = targetPos;
            isMovementDone = true;
        }
        nextPosition = Range.clip(nextPosition,0,1);
        setAngle(servo, nextPosition);

        return isMovementDone;
    }

    // The color sensors on the robot.
    public enum ColorSensorName {
//        MINERAL_COLOR,
    }

    /**
     * Gets the color value on the sensor.
     *
     * @param sensor The sensor to read.
     * @param color  The color channel to read intensity.
     */
    public int getColorSensor(ColorSensorName sensor, Color.Channel color) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
            return 0;
        }

        switch (color) {
            case RED:
                return s.red();
            case GREEN:
                return s.green();
            case BLUE:
                return s.blue();
            case ALPHA:
                return s.alpha();
            default:
                return 0;
        }
    }

    public boolean colorSensorExists(ColorSensorName sensor) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Sets the LED power for the color sensor.
     *
     * @param sensor  The sensor to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    public void setColorSensorLedEnabled(ColorSensorName sensor,
                                         boolean enabled) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
        } else {
            s.enableLed(enabled);
        }
    }

    /**
     * Checks MINERAL_COLOR sensor and returns enum based on color detected.
     *
     * @return RED, BLUE, or null
     */
//    public Color.Mineral getMineralColor() {
//        int red = getColorSensor(ColorSensorName.MINERAL_COLOR, Color.Channel.RED);
//        int blue = getColorSensor(ColorSensorName.MINERAL_COLOR, Color.Channel.BLUE);
//        if (red > blue) {
//            return Color.Mineral.GOLD;
//        } else if (blue > red) {
//            return Color.Mineral.SILVER;
//        } else {
//            return Color.Mineral.UNKNOWN;
//        }
//    }
//
//    public void displayColorSensorTelemetry() {
//        telemetry.addData("Color RED", getColorSensor(ColorSensorName.MINERAL_COLOR, Color.Channel.RED));
//        telemetry.addData("Color GREEN", getColorSensor(ColorSensorName.MINERAL_COLOR, Color.Channel.GREEN));
//        telemetry.addData("Color BLUE", getColorSensor(ColorSensorName.MINERAL_COLOR, Color.Channel.BLUE));
//        telemetry.addData("Mineral Color:", getMineralColor().toString());
//    }

    // Possible starting positions.
    public enum StartPosition {
        FIELD_CRATER,
        FIELD_DEPOT,
    }

    /**
     * Gets the Vuforia license key.
     */
    protected String getVuforiaLicenseKey() {
        String vuforiaLicenseKey = "AdhUZeT/////AAABmZzp9UHWY0xvhsM/ycx2t6ZVzVQkoHxi/L3Seg8ZsTZoOWhthLLS481295WPHcmQGzxpPfZdoDwf7cdSjCWQ9wS/mUybv81LrzdDJ01LIzhRigSltT36iYZhFno+j8mtHiU9RQbNOmI5KMP6zCJRoU6hqxi8BZdH97u86+iX2XzuzCeE6WDrjPLcnIfIxq8FpIa9maMi2GRlLx9RxmD0be0AJfeKN9Cw6fBo6hrdSnQX2Jx92qhEqwS6DB4JQxfgBTsNcM2igPiFz1GUTdmk4dLQBJjrJimGu3uHqyQpMbCydEj9wiog4FsfiShfLWGxezMfVUWEhrn+5fS4Ti1/00w5L3Xi5Qck/uWuKabUZjcR";
        return vuforiaLicenseKey;
    }

    // IMU Names (Could support multiple REV IMUs)
    public enum IMUNames {
        IMU,
    }

    /**
     * Should be executed at the beginning of loop() function.
     * Adds the most recent period length (in seconds) to a vector.
     * then, calculates the average period length.
     * @return Average period of past execution cycles.
     */
    public double updatePeriodTime(){
        pastPeriods.add(period.seconds());
        period.reset();
        if (pastPeriods.size()>= 200) {
            pastPeriods.remove(0);
        }
        return VectorMath.average(pastPeriods);
    }

    public double getAveragePeriodSec() {
        return VectorMath.average(pastPeriods);
    }

    public double getMaxPeriodSec() {
        return Collections.max(pastPeriods);
    }

    public double getLastPeriodSec() {
        if (pastPeriods.size() != 0) {
            return pastPeriods.lastElement();
        } else {
            return 0;
        }
    }


    public void init() {

        allMotors = new ArrayList<DcMotor>();
        for (MotorName m : MotorName.values()) {
            try {
                allMotors.add(hardwareMap.get(DcMotor.class, m.name()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.name());
                allMotors.add(null);
            }
        }

        // Collect a list of only the drive motors.
        driveMotorNames = new ArrayList<MotorName>();
        driveMotorNames.add(MotorName.DRIVE_FRONT_LEFT);
        driveMotorNames.add(MotorName.DRIVE_FRONT_RIGHT);
        driveMotorNames.add(MotorName.DRIVE_BACK_LEFT);
        driveMotorNames.add(MotorName.DRIVE_BACK_RIGHT);

        resetAndStopAllMotors();

        // Set motor directions.
        try {
            allMotors.get(MotorName.DRIVE_FRONT_RIGHT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.DRIVE_BACK_RIGHT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.ARM.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.WRIST.ordinal()).setDirection(DcMotor.Direction.FORWARD);
            allMotors.get(MotorName.FEEDER.ordinal()).setDirection(DcMotor.Direction.FORWARD);
            allMotors.get(MotorName.LIFT_WINCH.ordinal()).setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            telemetry.addData("Unable to set motor direction", "");
        }

        // Set drive motors to use encoders
        setDriveMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set drive motors to float instead of brake when power is zero.
        setDriveMotorsZeroPowerBraking(false);

        // Set arm motor to brake
        try {
            allMotors.get(MotorName.ARM.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            allMotors.get(MotorName.ARM.ordinal()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            allMotors.get(MotorName.WRIST.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            allMotors.get(MotorName.WRIST.ordinal()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            allMotors.get(MotorName.FEEDER.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            allMotors.get(MotorName.FEEDER.ordinal()).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            allMotors.get(MotorName.LIFT_WINCH.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            allMotors.get(MotorName.LIFT_WINCH.ordinal()).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            telemetry.addData("Unable to set arm motor to zero power brake or encoder use", "");
        }

        allServos = new ArrayList<Servo>();
        for (ServoName s : ServoName.values()) {
            try {
                allServos.add(hardwareMap.get(Servo.class, s.name()));
            } catch (Exception e) {
                telemetry.addData("Servo Missing", s.name());
                allServos.add(null);
            }
        }
        // Set servo direction
//        try {
//            allServos.get(ServoName.FLIPPER_LEFT.ordinal()).setDirection(Servo.Direction.REVERSE);
//        } catch (Exception e) {
//            telemetry.addData("Unable to set left servo direction", "");
//        }

        allColorSensors = new ArrayList<ColorSensor>();
        for (ColorSensorName s : ColorSensorName.values()) {
            try {
                allColorSensors.add(hardwareMap.get(ColorSensor.class,
                        s.name()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing", s.name());
                allColorSensors.add(null);
            }
        }

        df = new DecimalFormat("0.00");
        df_prec = new DecimalFormat("0.0000");

        stopAllMotors();
        period.reset(); // Reset timer
    }

    public void start() {
        stopAllMotors();
        period.reset(); // Reset timer
    }

    public void loop() {
        updatePeriodTime();
    }

    public void stop() {
        super.stop();

        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
        for (ColorSensorName s : ColorSensorName.values()) {
            setColorSensorLedEnabled(s, false);
        }
    }
}

