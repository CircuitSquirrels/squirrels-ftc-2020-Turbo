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
    private Mutable<Double> DriveSpeed = new Mutable<>(1.0);
    private Mutable<Double> RotationSpeed = new Mutable<>(1.0);

    // Define interactive init variable holders
    private double lifterSpeed;
    private double exponential;
    private boolean copilotEnabled;
    private double driveSpeed;

    // Define the MecanumNav and other useful variables
    MecanumNavigation mecanumNavigation;
    private AutoDrive autoDrive;
    private double triggerThreshold = 0.1;
    private boolean precisionMode = false;
    private double precisionSpeed = 0.3;

    @Override
    public void init() {
        super.init();
        //Changing gamepads to controllers
        controllerDrive = new Controller (gamepad1);
        controllerArm = new Controller (gamepad2);

        //Adding Interactive init options
        interactiveInit = new InteractiveInit(this);
        interactiveInit.addDouble(LiftSpeed, "Lifter speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(DriveSpeed, "Drive Speed Multiplier",  0.25, 0.5, 0.75, 1.0);
        interactiveInit.addDouble(RotationSpeed, "Rotation Speed Multiplier",  0.25, 0.5, 0.75, 1.0, 0.5);
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

        // Lock Interactive Init so it no longer receives inputs
        interactiveInit.lock();

        // Assign the variables to Interactive Init values
        lifterSpeed = LiftSpeed.get();
        exponential = Exponential.get();
        driveSpeed = DriveSpeed.get();
        copilotEnabled = CoPilot.get();
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

        if(controllerDrive.AOnce()) precisionMode = !precisionMode;
        double precisionOutput = precisionMode ? precisionSpeed : 1;
        telemetry.addData("Precision Mode", precisionMode);

        // Mecanum Drive Control
        setDriveForSimpleMecanum(
                Math.pow(controllerDrive.left_stick_x, exponential) * driveSpeed * precisionOutput,
                Math.pow(controllerDrive.left_stick_y, exponential) * driveSpeed * precisionOutput,
                Math.pow(controllerDrive.right_stick_x, exponential) * driveSpeed * RotationSpeed.get() * precisionOutput,
                Math.pow(controllerDrive.right_stick_y, exponential) * driveSpeed * precisionOutput);

        nonDriveControls();
    }

    /**
     * Robot controls for one or two people, customizable in the InteractiveInit
     */
    private void nonDriveControls() {
        Controller clawController;

        if (copilotEnabled) {
            clawController = controllerArm;
            // Lifter Control
            setPower(MotorName.LIFT_WINCH, Math.pow(controllerArm.left_stick_y, exponential) * lifterSpeed);
        } else {
            clawController = controllerDrive;
            // Lifter Control
            setPower(MotorName.LIFT_WINCH, Math.pow(controllerDrive.right_stick_y, 5) * lifterSpeed);
        }

        // Reset the robot's current position
        if(controllerDrive.YOnce()) {
            mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
        }
        // Add claw servo controls, operated by Driver if copilot is disabled, or copilot if enabled.
        telemetry.addData("CoPilot Mode", copilotEnabled);
        if(copilotEnabled) {
            if (gamepad2.left_bumper) {
                closeClaw();
                telemetry.addData("Claw: ", "CLOSED");
            } else if (gamepad2.right_bumper) {
                openClaw();
                telemetry.addData("Claw: ", "OPEN");
            } else if(gamepad2.right_trigger > 0.1) {
                verticalClaw();
                telemetry.addData("Claw: ", "VERTICAL");
            }
        } else {
            if (clawController.leftBumper()) {
                closeClaw();
                telemetry.addData("Claw: ", "CLOSED");
            } else if (clawController.rightBumper()) {
                openClaw();
                telemetry.addData("Claw: ", "OPEN");
            } else if(clawController.right_trigger > 0.1) {
                verticalClaw();
                telemetry.addData("Claw: ", "VERTICAL");
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
