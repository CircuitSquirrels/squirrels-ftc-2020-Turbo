package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.InteractiveInit;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.Mutable;

@TeleOp (name = "Manual", group = "Competition")
public class Manual extends RobotHardware {

    //Setting controller variables
    Controller controller1 = null;
    Controller controller2 = null;

    //Adding interactive init variables
    Mutable<Double> ArmSpeed = new Mutable<>(0.5);
    Mutable<Double> WristSpeed = new Mutable<>(0.2);
    Mutable<Double> FeederSpeed = new Mutable<>(.8);
    Mutable<Boolean> CoPilot = new Mutable<>(false);
    Mutable<Double> Exponential = new Mutable<>(1.0);
    Mutable<Double> GoToPosPower = new Mutable<>(1.0);
    Mutable<Double> LifterSpeed = new Mutable<>(1.0);
    Mutable<Boolean> UsingMiniRobot = new Mutable<>(false);

    double armSpeed;
    double wristSpeed;
    double feederSpeed;
    double lifterSpeed;
    double exponential;
    boolean copilotEnabled;
    int liftEncoderHoldPosition = 0;

    //Enum for arm states
    public enum ArmStates{
        ARM_START,
        WRIST_UP,
        ARM_LEVEL,
        ARM_VERTICAL,
        ARM_SCORE,
        MANUAL,
    }

    ArmStates armState = ArmStates.MANUAL;

    InteractiveInit interactiveInit = null;
    MecanumNavigation mecanumNavigation;
    AutoDrive autoDrive;

    double triggerThreshold = 0.1;



    @Override
    public void init() {
        super.init();

        //Changing gamepads to controllers
        controller1 = new Controller (gamepad1);
        controller2 = new Controller (gamepad2);

        //Adding Interactive init options
        interactiveInit = new InteractiveInit(this);
        interactiveInit.addDouble(ArmSpeed, "Arm speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(WristSpeed, "Wrist speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.8, 0.9, 1.0, 0.7);
        interactiveInit.addDouble(FeederSpeed, "Feeder speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.9, 1.0, 0.8);
        interactiveInit.addDouble(GoToPosPower, "Power to goto position", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(LifterSpeed, "Lifter Speed", 0.1, 0.2, .3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0);
        interactiveInit.addDouble(Exponential, "Exponential", 3.0, 1.0);
        interactiveInit.addBoolean(CoPilot, "Copilot Enable", false, true);
        interactiveInit.addBoolean(UsingMiniRobot, "Using MiniRobot", true, false);

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
        if(UsingMiniRobot.get()) {
            mecanumNavigation = new MecanumNavigation(this,Constants.MiniRobot.getDriveTrainMecanum());
        } else {
            mecanumNavigation = new MecanumNavigation(this,Constants.getDriveTrainMecanum());
        }
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0));
        autoDrive = new AutoDrive(this, mecanumNavigation);

        interactiveInit.lock();
    }

    @Override
    public void loop() {
        super.loop();

        //Updates variables
        controller1.update();
        controller2.update();

        mecanumNavigation.update();

        armSpeed = ArmSpeed.get();
        wristSpeed = WristSpeed.get();
        feederSpeed = FeederSpeed.get();
        lifterSpeed = LifterSpeed.get();
        exponential = Exponential.get();
        copilotEnabled = CoPilot.get();

        // Telemetry
        mecanumNavigation.displayPosition();
        telemetry.addData("ArmStates", armState.toString());


        if(!copilotEnabled &&  controller1.leftBumper() && controller1.rightBumper()) {
            chordControls();
        } else {

            armStateMachine();

            // Drive Controls
            // Mecanum Drive Control
            setDriveForSimpleMecanum(Math.pow(controller1.left_stick_x, exponential), Math.pow(controller1.left_stick_y, exponential),
                    Math.pow(controller1.right_stick_x, exponential), Math.pow(controller1.right_stick_y, exponential));
            nonDriveControls();
        }
    }

    private void chordControls(){
        if(!copilotEnabled && controller1.leftBumper() && controller1.rightBumper()) {
            // Y recall mecanum navigation point
            // X store current mecanum navigation point to navMem
            // A set mecanumNavigation to vision position
            // B
            if(controller1.X()) {

            }

        }
    }

    private void nonDriveControls() {
        // Based on copilotEnabled, sets controls for
        // Arm, Wrist, Feeder
        // Feeder Lift, and Lift Winch
        if (copilotEnabled) {
            // Copilot Controls
            // Arm Control
            setPower(MotorName.ARM, Math.pow(-controller2.left_stick_y, exponential) * armSpeed);
            // Wrist Control
            setPower(MotorName.WRIST, Math.pow(-controller2.right_stick_y, exponential) * wristSpeed);

            // Feeder Control
            if (controller2.right_trigger > triggerThreshold) {
                setPower(MotorName.FEEDER, Math.pow(controller2.right_trigger, exponential) * feederSpeed);
            } else if (controller2.left_trigger > triggerThreshold) {
                setPower(MotorName.FEEDER, Math.pow(-controller2.left_trigger, exponential) * feederSpeed);
            } else {
                setPower(MotorName.FEEDER, 0);
            }

            // Move Feeder Servo
            if (controller2.dpadRight()) {
                setAngle(ServoName.FEEDER_LIFTER, getAngle(ServoName.FEEDER_LIFTER) - 10);
                telemetry.addData("Feeder", "Up");
            } else if (controller2.dpadLeft()) {
                setAngle(ServoName.FEEDER_LIFTER, getAngle(ServoName.FEEDER_LIFTER) + 10);
                telemetry.addData("Feeder", "Down");
            } else {
                setAngle(ServoName.FEEDER_LIFTER, getAngle(ServoName.FEEDER_LIFTER));
            }

            // Lift Control
            if (controller2.dpadUp()) {
                setPower(MotorName.LIFT_WINCH, lifterSpeed);
                liftEncoderHoldPosition = getEncoderValue(MotorName.LIFT_WINCH);
                telemetry.addData("LIFT", "UP");
            } else if (controller2.dpadDown()) {
                setPower(MotorName.LIFT_WINCH, -lifterSpeed);
                liftEncoderHoldPosition = getEncoderValue(MotorName.LIFT_WINCH);
                telemetry.addData("LIFT", "DOWN");
            } else {
                setPower(MotorName.LIFT_WINCH, 0);
                driveMotorToPos(MotorName.LIFT_WINCH,liftEncoderHoldPosition,1.0, 100);
            }


        } else {
            // Pilot Controls
            // Arm Control
            setPower(MotorName.ARM, Math.pow(-controller1.right_stick_y, exponential) * armSpeed);
            // Wrist Control
            if (controller1.right_trigger > triggerThreshold) {
                setPower(MotorName.WRIST, Math.pow(controller1.right_trigger, exponential) * wristSpeed);
            } else if (controller1.left_trigger > triggerThreshold) {
                setPower(MotorName.WRIST, Math.pow(-controller1.left_trigger, exponential) * wristSpeed);
            } else {
                setPower(MotorName.WRIST, 0);
            }

            // Feeder Control
            if (controller1.rightBumper() && !controller1.leftBumper()) {
                setPower(MotorName.FEEDER, feederSpeed);
            } else if (controller1.leftBumper() && !controller1.rightBumper()) {
                setPower(MotorName.FEEDER, -feederSpeed);
            } else {
                setPower(MotorName.FEEDER, 0);
            }

            // Move Feeder Servo
            if (controller1.dpadRight()) {
                setAngle(ServoName.FEEDER_LIFTER, getAngle(ServoName.FEEDER_LIFTER) - 10);
                telemetry.addData("Feeder", "Up");
            } else if (controller1.dpadLeft()) {
                setAngle(ServoName.FEEDER_LIFTER, getAngle(ServoName.FEEDER_LIFTER) + 10);
                telemetry.addData("Feeder", "Down");
            } else {
                setAngle(ServoName.FEEDER_LIFTER, getAngle(ServoName.FEEDER_LIFTER));
            }

            // Lift Control
            if (controller1.dpadUp()) {
                setPower(MotorName.LIFT_WINCH, lifterSpeed);
                liftEncoderHoldPosition = getEncoderValue(MotorName.LIFT_WINCH);
                telemetry.addData("LIFT", "UP");
            } else if (controller1.dpadDown()) {
                setPower(MotorName.LIFT_WINCH, -lifterSpeed);
                liftEncoderHoldPosition = getEncoderValue(MotorName.LIFT_WINCH);
                telemetry.addData("LIFT", "DOWN");
            } else {
                setPower(MotorName.LIFT_WINCH, 0);
                driveMotorToPos(MotorName.LIFT_WINCH,liftEncoderHoldPosition,1.0, 100);
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


    //Arm and wrist state machine
    public void armStateMachine () {

        double goToPos = GoToPosPower.get();
        boolean wristArrived = false;
        boolean armArrived = false;
        double power = goToPos;
        boolean up;
        boolean down;

        if (CoPilot.get() == true) {
            up = controller2.rightBumperOnce();
            down = controller2.leftBumperOnce();

        } else {
            up = controller1.XOnce();
            down = controller1.YOnce();

        }

            if (
                    CoPilot.get() == true && ((Math.abs(controller2.right_stick_y) >= triggerThreshold) ||
                            (Math.abs(controller2.left_stick_y) >= triggerThreshold)) ||
                            CoPilot.get() == false && ((Math.abs(controller1.right_stick_y) >= triggerThreshold) ||
                                    (Math.abs(controller1.right_trigger) >= triggerThreshold) || (Math.abs(controller1.left_trigger) >= triggerThreshold))){
                armState = ArmStates.MANUAL;
            }

        switch (armState) {
            case ARM_START:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_BOTTOM_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_START_TICKS, power);
                if (armArrived && wristArrived && up) {
                    armState = ArmStates.WRIST_UP;
                }
                break;

            case WRIST_UP:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_BOTTOM_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_MAX_TICKS, power);
                if (armArrived && wristArrived && up) {
                    armState = ArmStates.ARM_LEVEL;
                } else if (armArrived && wristArrived && down) {
                    armState = ArmStates.ARM_START;
                }
                break;

            case ARM_LEVEL:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_LEVEL_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_MAX_TICKS, power);
                if (armArrived && wristArrived && up) {
                    armState = ArmStates.ARM_VERTICAL;
                } else if (armArrived && wristArrived && down) {
                    armState = ArmStates.WRIST_UP;
                }
                break;

            case ARM_VERTICAL:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_VERTICAL_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_STRAIGHT_TICKS, power);
                if (armArrived && wristArrived && up) {
                    armState = ArmStates.ARM_SCORE;
                } else if (armArrived && wristArrived && down) {
                    armState = ArmStates.ARM_LEVEL;
                }
                break;

            case ARM_SCORE:
                armArrived = driveMotorToPos(MotorName.ARM, Constants.ARM_VERTICAL_TICKS, power);
                wristArrived = driveMotorToPos(MotorName.WRIST, Constants.WRIST_MIN_TICKS, power);
                if (armArrived && wristArrived && up) {
                    //armState = ArmStates.ARM_LEVEL;
                } else if (armArrived && wristArrived && down) {
                    armState = ArmStates.ARM_VERTICAL;
                }
                break;

            case MANUAL:
                if (getEncoderValue(MotorName.ARM) < Constants.ARM_LEVEL_TICKS) {
                    // If Arm is currently below level
                    if (up) {
                        armState = ArmStates.ARM_START;
                    } else if (down) {
                        armState = ArmStates.WRIST_UP;
                    }
                } else {
                    // else if Arm is currently Above Level
                    if (up) {
                        armState = ArmStates.ARM_VERTICAL;
                    } else if (down) {
                        armState = ArmStates.ARM_LEVEL;
                    }
                }

                break;

            default:
                telemetry.addData("Default arm state ", "if this ran you messed up...");
                break;

        }

    }

}
