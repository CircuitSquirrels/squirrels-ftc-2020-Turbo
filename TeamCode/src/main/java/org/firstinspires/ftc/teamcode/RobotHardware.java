package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.teamcode.DeadWheels.OdometryConfig;
import org.firstinspires.ftc.teamcode.DeadWheels.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.Utilities.*;
import org.firstinspires.ftc.teamcode.Vision.*;
import org.openftc.revextensions2.*;

import java.text.DecimalFormat;
import java.util.*;

//Todo Clean up docs and add comments for newly added functions
public class RobotHardware extends OpMode {

    // All motors on the robot, in order of MotorName.
    private ArrayList<ExpansionHubMotor> allMotors;

    // Expansion hubs and bulk reads
    private ExpansionHubEx expansionHubDrive, expansionHubArm;
    private RevBulkData bulkDataDrive, bulkDataArm;


    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;

    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;

    // Format for displaying decimals.
    public DecimalFormat df = new DecimalFormat("0.00");
    public DecimalFormat df_prec = new DecimalFormat("0.0000");

    // IMU reference
    public BNO055IMU imu;

    // Execution cycle period monitor.
    private ElapsedTime period = new ElapsedTime();
    private Vector<Double> pastPeriods = new Vector<>();

    public Controller controller1, controller2;
    public MecanumNavigation mecanumNavigation;
    public OdometryConfig odometryConfig = new OdometryConfig();
    public OdometryLocalizer odometryLocalizer;
    public AutoDrive autoDrive;
    public PositionController positionController;
    public IMUUtilities imuUtilities;
    public SkystoneDetector skystoneDetector;
    public InteractiveInit interactiveInit;


    // The motors on the robot, must be the same names defined in the robot's Configuration file.
    public enum MotorName {
        DRIVE_FRONT_LEFT("DRIVE_FRONT_LEFT", ExpansionHubs.DRIVE, Types.DRIVE, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER),
        DRIVE_FRONT_RIGHT("DRIVE_FRONT_RIGHT", ExpansionHubs.DRIVE, Types.DRIVE, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER),
        DRIVE_BACK_LEFT("DRIVE_BACK_LEFT", ExpansionHubs.DRIVE, Types.DRIVE, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER),
        DRIVE_BACK_RIGHT("DRIVE_BACK_RIGHT", ExpansionHubs.DRIVE, Types.DRIVE, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER),
        LIFT_WINCH("LIFT_WINCH", ExpansionHubs.ARM, Types.ARM, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER),
        RIGHT_WHEEL("RIGHT_WHEEL", ExpansionHubs.ARM, Types.DEADWHEEL, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER),
        CENTER_WHEEL("CENTER_WHEEL", ExpansionHubs.ARM, Types.DEADWHEEL, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER),
        LEFT_WHEEL("LEFT_WHEEL", ExpansionHubs.ARM, Types.DEADWHEEL, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        private final String configName;
        private final ExpansionHubs expansionHub;
        private final DcMotor.Direction direction;
        private final DcMotor.ZeroPowerBehavior zeroPowerBehavior;
        private final DcMotor.RunMode runMode;
        private final Types type;

        MotorName(String configName, ExpansionHubs expansionHub, Types type, DcMotor.Direction direction, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.RunMode runMode) {
            this.configName = configName;
            this.expansionHub = expansionHub;
            this.direction = direction;
            this.zeroPowerBehavior = zeroPowerBehavior;
            this.runMode = runMode;
            this.type = type;
        }

        public String getConfigName() {
            return configName;
        }

        public ExpansionHubs getExpansionHub() {
            return expansionHub;
        }

        public DcMotor.Direction getDirection() {
            return direction;
        }

        public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
            return zeroPowerBehavior;
        }

        public DcMotor.RunMode getRunMode() {
            return runMode;
        }

        public Types getType() {
            return type;
        }
    }

    private enum ExpansionHubs {
        DRIVE("Expansion Hub 2"),
        ARM("Expansion Hub 1");

        private final String hubName;

        ExpansionHubs(String hubName) {
            this.hubName = hubName;
        }

        public String getHubName() {
            return hubName;
        }
    }

    private enum Types {
        DRIVE,
        DEADWHEEL,
        ARM
    }

    /**
     * Sets the power of the motor.
     *
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    public void setPower(MotorName motor, double power) {
        ExpansionHubMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name() + ": " + df.format(power));
        } else {
            m.setPower(power);
        }
    }

    /**
     * Get motor power.
     *
     * @param motor MotorName.
     * @return Motor Power, or zero if it cannot be found.
     */
    public double getPower(MotorName motor) {
        ExpansionHubMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing: ", motor.name());
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
        ExpansionHubMotor m = allMotors.get(motor.ordinal());
        if(m != null) {
            RevBulkData bulkData;
            for (MotorName motorName : MotorName.values()) {
                bulkData = motorName.getExpansionHub().equals(ExpansionHubs.DRIVE) ? bulkDataDrive : bulkDataArm;

                if(bulkData == null) break;

                if(motorName.name().equals(motor.name())) {
                    return bulkData.getMotorCurrentPosition(m);
                }
            }
            Log.w("RobotHardware","Not using bulk reads for motor: " + motor.name());
            return m.getCurrentPosition();
        } else {
            telemetry.addData("Motor Missing: ", motor.name());
            return 0;
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

    public void resetAndStopMotor(MotorName motor) {
        ExpansionHubMotor m = allMotors.get(motor.ordinal());
        if(m == null) {
            telemetry.addData("Motor Missing: " ,motor.name());
        } else {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    /**
     * Stops all motors and resets the Encoder Positions.
     */
    public void resetAndStopAllMotors() {
        for (MotorName name : MotorName.values()) {
            ExpansionHubMotor motor = allMotors.get(name.ordinal());
            if (motor == null) {
                telemetry.addData("Motor Missing: " ,name.name());
            } else {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    protected void setDeadWheelMotorsRunMode(DcMotor.RunMode runMode) {
        for (MotorName motor : MotorName.values()) {
            if(!motor.getType().equals(Types.DEADWHEEL)) continue;
            ExpansionHubMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing: ", motor.name());
            } else {
                m.setMode(runMode);
            }
        }
    }

    /**
     * Set all four drive motors to the same runMode.
     * Options are RUN_WITHOUT_ENCODER, RUN_USING_ENCODER,
     * RUN_TO_POSITION is used with setTargetPosition()
     * STOP_AND_RESET_ENCODER.
     *
     */
    protected void setDriveMotorsRunMode(DcMotor.RunMode runMode) {
        for (MotorName motor : MotorName.values()) {
            if(!motor.getExpansionHub().equals(ExpansionHubs.DRIVE)) continue;
            ExpansionHubMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing: ", motor.name());
            } else {
                m.setMode(runMode);
            }
        }
    }

    /**
     * Set zero power braking behavior for all drive wheels.
     *
     * @param zeroPowerBraking boolean to activate or deactivate zero power braking.
     */
    protected void setDriveMotorsZeroPowerBraking(boolean zeroPowerBraking) {
        DcMotor.ZeroPowerBehavior brakingMode = zeroPowerBraking ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        for (MotorName motor : MotorName.values()) {
            if(!motor.getExpansionHub().equals(ExpansionHubs.DRIVE)) continue;
            ExpansionHubMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing: ", motor.name());
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
     * Apply motor power matching the wheels object.
     *
     * @param wheels Provides all four mecanum wheel powers, [-1, 1].
     */
    public void setDriveForMecanumWheels(Mecanum.Wheels wheels) {
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    public void setDriveForMecanumCommand(Mecanum.Command command) {
        Mecanum.Wheels wheels = Mecanum.commandToWheels(command);
        setDriveForMecanumWheels(wheels);
    }

    /**
     * Sets mecanum drive chain power using simplistic calculations.
     *
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



    // The servos on the robot, names must be defined in robot Configure file
    public enum ServoName {
        CLAW_LEFT(Servo.Direction.FORWARD),
        CLAW_RIGHT(Servo.Direction.FORWARD);

        private final Servo.Direction direction;

        ServoName(Servo.Direction direction) {
            this.direction = direction;
        }

        public Servo.Direction getDirection() {
            return direction;
        }
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
            telemetry.addData("Servo Missing: ", servo.name() + ": " + df.format(position));
        } else {
            s.setPosition(position);
        }
    }

    /**
     * Get the position of a servo.
     *
     * @param servo ServoName enum to check.
     * @return double servo position [0,-1].
     */
    public double getAngle(ServoName servo) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing: ", servo.name());
            return -1;
        } else {
            return s.getPosition();
        }
    }

    /**
     * Move a servo to a position at a specific speed.
     *
     * @param servo The servo that will be driven.
     * @param targetPos The position that you want the servo to go to; Range [1-0].
     * @param rate The speed at which the servo will move.
     * @return Returns whether or not the servo has arrived to its targetPos.
     */
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

    public enum ClawPositions {
        CLOSED(Constants.RIGHT_CLAW_CLOSED, Constants.LEFT_CLAW_CLOSED),
        OPEN(Constants.RIGHT_CLAW_OPEN, Constants.LEFT_CLAW_OPEN),
        VERTICAL(Constants.RIGHT_CLAW_VERTICAL, Constants.LEFT_CLAW_VERTICAL),
        NONE(-1, -1);
        private final double rightPos;
        private final double leftPos;

        ClawPositions(double RightPos, double LeftPos) {
            this.rightPos = RightPos;
            this.leftPos = LeftPos;
        }

        public double getRightPos() {
            return rightPos;
        }

        public double getLeftPos() {
            return leftPos;
        }
    }

    public void commandClaw(ClawPositions positions) {
        double rAngle = positions.getRightPos() != -1 ? positions.getRightPos() : getAngle(ServoName.CLAW_RIGHT);
        double lAngle = positions.getLeftPos() != -1 ? positions.getLeftPos() : getAngle(ServoName.CLAW_LEFT);

        setAngle(ServoName.CLAW_RIGHT, rAngle);
        setAngle(ServoName.CLAW_LEFT, lAngle);
    }

    // The color sensors on the robot.
    public enum ColorSensorName {

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
            telemetry.addData("Color Sensor Missing: ", sensor.name());
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

    /**
     * Check whether or not the sensor is detected on the robot.
     *
     * @param sensor The sensor to check.
     * @return Returns whether or not the sensor exists.
     */
    public boolean colorSensorExists(ColorSensorName sensor) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        return s != null;
    }

    /**
     * Sets the LED power for the color sensor.
     *
     * @param sensor  The sensor to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    public void setColorSensorLedEnabled(ColorSensorName sensor, boolean enabled) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing: ", sensor.name());
        } else {
            s.enableLed(enabled);
        }
    }

    // Possible starting positions.
    public enum StartPosition {
        FIELD_LOADING,
        FIELD_BUILD,
    }

    /**
     * Gets the Vuforia license key.
     */
    protected String getVuforiaLicenseKey() {
        String vuforiaLicenseKey = "AVFkXXL/////AAABmbZECMrRQkzRjDc4Fz5X9EwXnUQbKq9G/gERF/bpOt9TIpneSXsY3Qyv878mUAqY1coOvRiFFj/ZoK+uIs+qK3IHQUwgqJW6y9EhzwRNqcdzEHvrnttZnUJmWjBh0O93lrrC8mGzDw+/wozmT/jr1Peu4qOijWfYhgW2GGtszwl3/u6u9Pca43FcykCY52RsXcsGkM2/8z0Ini3hc/HxrWoKcYnycJf5yXLSxXWJz+vHVdnhu8Wen18HXX4ec9MA+P0psoVuUqtwZGpcXDEXSlk//Z1p9tN3vdBApm2fzdoYfGSqhLnN0gITvIPB3VMpXRUo0Zbb7QXCs0Ydh2EjPNTKlBFNGFewr+7Xhd2XVPcd";
        return vuforiaLicenseKey;
    }


    public void loadVision(RobotHardware opmode, Color.Ftc teamColor) {
        skystoneDetector = new SkystoneDetector(opmode, teamColor);
        skystoneDetector.init(new AveragingPipeline());
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

    public double getTime() {
        return time;
    }

    /**
     * Provides an entry point for fake timing during unit testing.
     */
    public ElapsedTime getNewElapsedTime() {
        return new ElapsedTime();
    }

    /**
     * Initialize all motors, servos, sensors, set motor direction, motor runMode
     * Output whether or not the defined motors, servos, sensors could not be found
     */
    public void init() {

        // Setup expansion hubs for bulk reads.
        try {
            expansionHubDrive = hardwareMap.get(ExpansionHubEx.class, ExpansionHubs.DRIVE.getHubName());
            expansionHubArm = hardwareMap.get(ExpansionHubEx.class, ExpansionHubs.ARM.getHubName());
        } catch (Exception e) {
            telemetry.addData("Could not find expansion hub", e.getMessage());
        }

        allMotors = new ArrayList<>();
        for (MotorName m : MotorName.values()) {
            try {
                ExpansionHubMotor motor = hardwareMap.get(ExpansionHubMotor.class, m.getConfigName());
                allMotors.add(motor);
                resetAndStopMotor(m);
                motor.setMode(m.getRunMode());
                motor.setDirection(m.getDirection());
                motor.setZeroPowerBehavior(m.getZeroPowerBehavior());
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.getConfigName());
                allMotors.add(null);
            }
        }

//        initializePID(); // Makes current drive PIDF parameters available as K_P, K_I, K_D, K_F.

        allServos = new ArrayList<>();
        for (ServoName s : ServoName.values()) {
            try {
                Servo servo = hardwareMap.get(Servo.class, s.name());
                allServos.add(servo);
                servo.setDirection(s.getDirection());
            } catch (Exception e) {
                telemetry.addData("Servo Missing: ", s.name());
                allServos.add(null);
            }
        }

        allColorSensors = new ArrayList<>();
        for (ColorSensorName s : ColorSensorName.values()) {
            try {
                allColorSensors.add(hardwareMap.get(ColorSensor.class, s.name()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing: ", s.name());
                allColorSensors.add(null);
            }
        }

        interactiveInit = new InteractiveInit(this);

        stopAllMotors();
        period.reset(); // Reset timer
    }

    public void init_loop() {
        interactiveInit.update();

        bulkDataDrive = expansionHubDrive.getBulkInputData();
        bulkDataArm = expansionHubArm.getBulkInputData();
    }

    public void start() {
        stopAllMotors();
        bulkDataDrive = expansionHubDrive.getBulkInputData();
        bulkDataArm = expansionHubArm.getBulkInputData();
        interactiveInit.lock();
        period.reset(); // Reset timer
    }

    public void loop() {
        updatePeriodTime();
        bulkDataDrive = expansionHubDrive.getBulkInputData();
        bulkDataArm = expansionHubArm.getBulkInputData();
    }

    public void stop() {
        super.stop();

        for (MotorName m : MotorName.values())
            setPower(m, 0);

        for (ColorSensorName s : ColorSensorName.values())
            setColorSensorLedEnabled(s, false);
    }


    // Todo Fix PIDF tuner due to changing the motor enums from 3 to 1 enum.
    public double K_P = 2.5;
    public double K_I = 0.1;
    public double K_D = 0.2;
    public double K_F = 0.0;

    public void initializePID() {
        PIDFCoefficients pidfOrig = allMotors.get(0).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        K_P = pidfOrig.p;
        K_I = pidfOrig.i;
        K_D = pidfOrig.d;
        K_F = pidfOrig.f;
    }

    public void configureDriveMotorVelocityPID(double K_P, double K_I, double K_D, double K_F) {
        for(MotorName motorName : MotorName.values()) {
            configureMotorVelocityPID(motorName,K_P,K_I,K_D,K_F);
        }
    }

    public void configureMotorVelocityPID(MotorName motorName, double K_P, double K_I, double K_D, double K_F) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(K_P,K_I,K_D,K_F);
        allMotors.get(motorName.ordinal()).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        this.K_P = K_P;
        this.K_I = K_I;
        this.K_D = K_D;
        this.K_F = K_F;
    }

    public enum ControlParameter {
        P,I,D,F
    }

    public void changeDriveControlParameterByFactor(ControlParameter controlParameter, double scaleFactor) {
        if(scaleFactor < 0) return; // Error prevention.
        double currentValue = getSingleDriveControlParameter(controlParameter);
        setSingleDriveControlParameter(controlParameter, currentValue * scaleFactor);
    }


    public void setSingleDriveControlParameter(ControlParameter controlParameter, double newValue) {
        if(newValue < 0) return; // Error prevention.
        double P,I,D,F;
        P = this.K_P;
        I = this.K_I;
        D = this.K_D;
        F = this.K_F;
        switch (controlParameter) {
            case P:
                P = newValue;
                break;
            case I:
                I = newValue;
                break;
            case D:
                D = newValue;
                break;
            case F:
                F = newValue;
        }
        configureDriveMotorVelocityPID(P,I,D,F);
    }

    public double getSingleDriveControlParameter(ControlParameter controlParameter) {
        switch (controlParameter) {
            case P:
                return this.K_P;
            case I:
                return this.K_I;
            case D:
                return this.K_D;
            default:
                return this.K_F;
        }
    }
}

