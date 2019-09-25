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

    //Setting controller variables
    Controller controllerDrive = null;
    Controller controllerArm = null;

    //Adding interactive init variables
    private Mutable<Double> LiftSpeed = new Mutable<>(1.0);
    private Mutable<Boolean> CoPilot = new Mutable<>(false);
    private Mutable<Double> Exponential = new Mutable<>(1.0);

    private double lifterSpeed;
    private double exponential;
    private boolean copilotEnabled;
    private int liftEncoderHoldPosition = 0;


    private InteractiveInit interactiveInit = null;
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
        // MecanumNavigation and auto control
        mecanumNavigation = new MecanumNavigation(this,Constants.getDriveTrainMecanum());

        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0));
        autoDrive = new AutoDrive(this, mecanumNavigation);

        interactiveInit.lock();
    }

    @Override
    public void loop() {
        super.loop();

        //Updates variables
        controllerDrive.update();
        controllerArm.update();

        mecanumNavigation.update();

        lifterSpeed = LiftSpeed.get();
        exponential = Exponential.get();
        copilotEnabled = CoPilot.get();

        // Telemetry
        mecanumNavigation.displayPosition();

        // Mecanum Drive Control
        setDriveForSimpleMecanum(Math.pow(controllerDrive.left_stick_x, exponential), Math.pow(controllerDrive.left_stick_y, exponential),
                Math.pow(controllerDrive.right_stick_x, exponential), Math.pow(controllerDrive.right_stick_y, exponential));
        nonDriveControls();

    }


    private void nonDriveControls() {
        // Based on copilotEnabled, sets controls for
        // Arm, Wrist, Feeder
        // Feeder Lift, and Lift Winch
        if (copilotEnabled) {
            // Copilot Controls

            if (controllerArm.leftBumper()) {
                setAngle(ServoName.FOUNDATION, 0.5);
                telemetry.addData("SERVO: ", "UP");
            } else if (controllerArm.rightBumper()) {
                setAngle(ServoName.FOUNDATION, 0.8);
                telemetry.addData("SERVO: ", "DOWN");
            }

            // Lift Control
            setPower(MotorName.LEFT_LIFT_WINCH, Math.pow(controllerArm.left_stick_y, exponential) * lifterSpeed);
        } else {
            // Pilot Controls

            if (controllerArm.leftBumper()) {
                setAngle(ServoName.FOUNDATION, 0.5);
                telemetry.addData("SERVO: ", "UP");
            } else if (controllerArm.rightBumper()) {
                setAngle(ServoName.FOUNDATION, 0.8);
                telemetry.addData("SERVO: ", "DOWN");
            }

            // Lift Control
            if (controllerDrive.dpadUp()) {
                setPower(MotorName.LEFT_LIFT_WINCH, lifterSpeed);
            } else if (controllerDrive.dpadDown()) {
                setPower(MotorName.LEFT_LIFT_WINCH, -lifterSpeed);
            } else {
                setPower(MotorName.LEFT_LIFT_WINCH, 0);
            }
        }
    }


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


    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power) {
        int rampDistanceTicks = 400;
        return driveMotorToPos (motorName,targetTicks,power,rampDistanceTicks);
    }
}
