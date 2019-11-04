package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;

@TeleOp (name="Manual",group="Competition")
public class Manual extends RobotHardware {

    // Setting controller variables
    Controller controllerDrive = null;
    Controller controllerArm = null;

    // Adding interactive init variables
    private InteractiveInit interactiveInit = null;
    private Mutable<Double> LiftSpeed = new Mutable<>(1.0);
    private Mutable<Boolean> CoPilot = new Mutable<>(false);
    private Mutable<Double> Exponential = new Mutable<>(1.0);
    private Mutable<Double> Slowmode = new Mutable<>(1.0);

    // Define interactive init variable holders
    private double lifterSpeed;
    private double exponential;
    private boolean copilotEnabled;
    private double slowmode;

    // Define the MecanumNav and other useful variables
    MecanumNavigation mecanumNavigation;
    private AutoDrive autoDrive;
    private double triggerThreshold = 0.1;

    @Override
    public void init() {
        super.init();
        //Changing gamepads to controllers
        controllerDrive = new Controller (gamepad1);
        controllerArm = new Controller (gamepad2);

        //Adding Interactive init options
        interactiveInit = new InteractiveInit(this);
        interactiveInit.addDouble(LiftSpeed, "Lifter speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(Slowmode, "Slow Mode Multiplier",  0.25, 0.5, 0.75, 1.0);
        interactiveInit.addDouble(Exponential, "Exponential", 3.0, 1.0);
        interactiveInit.addBoolean(CoPilot, "Copilot Enable", false, true);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        interactiveInit.update();
    }

    @Override
    public void start() {
        super.start();
        // Initialize the Mecanum Navigation for use
        mecanumNavigation = new MecanumNavigation(this,Constants.getDriveTrainMecanum());
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0));
        autoDrive = new AutoDrive(this, mecanumNavigation);

        // Assign the variables to Interactive Init values
        lifterSpeed = LiftSpeed.get();
        exponential = Exponential.get();
        slowmode = Slowmode.get();
        copilotEnabled = CoPilot.get();

        // Lock Interactive Init so it no longer receives inputs
        interactiveInit.lock();
    }

    @Override
    public void loop() {
        super.loop();

        // Update variables with new values
        controllerDrive.update();
        controllerArm.update();
        mecanumNavigation.update();

        // Display the robot's position compared to where it started
        mecanumNavigation.displayPosition();

        // Mecanum Drive Control
        setDriveForSimpleMecanum(Math.pow(controllerDrive.left_stick_x, exponential) * slowmode, Math.pow(controllerDrive.left_stick_y, exponential) * slowmode,
                Math.pow(controllerDrive.right_stick_x, exponential) * slowmode, Math.pow(controllerDrive.right_stick_y, exponential) * slowmode);
        nonDriveControls();

    }

    /**
     * Robot controls for one or two people, customizable in the InteractiveInit
     */
    private void nonDriveControls() {
        if (copilotEnabled) {
            // Reset the robot's current position
            if(controllerDrive.YOnce()) {
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            }
            // Add claw servo controls
            if (controllerArm.leftBumper()) {
                setAngle(ServoName.CLAW_LEFT, 0.4);
                telemetry.addData("Claw: ", "DOWN");
            } else if (controllerArm.rightBumper()) {
                setAngle(ServoName.CLAW_LEFT, 1);
                telemetry.addData("Claw: ", "UP");
            }

            // Lifter Control
            setPower(MotorName.LIFT_WINCH, Math.pow(controllerArm.left_stick_y, exponential) * lifterSpeed);
        } else {
            if (controllerDrive.leftBumper()) {
                setAngle(ServoName.CLAW_LEFT, 0.4);
                telemetry.addData("SERVO: ", "UP");
            } else if (controllerDrive.rightBumper()) {
                setAngle(ServoName.CLAW_LEFT, 1);
                telemetry.addData("SERVO: ", "DOWN");
            }
        }
    }

    /**
     * @param motorName The motor that will be driven
     * @param targetTicks The position where the motor will be driven. Must be in encoder Ticks
     * @param power The power at which the robot will be driven
     * @param rampThreshold The position when the robot will start slowing the motor down before its destination
     * @return Returns whether or not the motor arrived to the specified position
     */
    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power, double rampThreshold) {
        power = Range.clip(Math.abs(power), 0, 1);
        int poweredDistance = 0;
        int arrivedDistance = 50;
        double maxRampPower = 1.0;
        double minRampPower = 0.0;
        int errorSignal = getEncoderValue(motorName) - targetTicks;
        double direction = errorSignal > 0 ? -1.0: 1.0;
        double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampThreshold, maxRampPower, minRampPower);

        if (Math.abs(errorSignal) >= poweredDistance) {
            setPower(motorName, direction * power * rampDownRatio);
        } else {
            setPower(motorName, 0);
        }

        return Math.abs(errorSignal) <= arrivedDistance;
    }

    /**
     * @param motorName The motor that will be driven
     * @param targetTicks The position where the motor will be driven. Must be in encoder Ticks
     * @param power The power at which the robot will be driven
     * @return Returns whether or not the motor arrived to the specified position
     */
    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power) {
        int rampDistanceTicks = 400;
        return driveMotorToPos (motorName,targetTicks,power,rampDistanceTicks);
    }
}
