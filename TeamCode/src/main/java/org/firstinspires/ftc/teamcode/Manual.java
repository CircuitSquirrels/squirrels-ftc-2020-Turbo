package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DeadWheels.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.DeadWheels.OdometryTicks;
import org.firstinspires.ftc.teamcode.Utilities.*;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.*;

@TeleOp (name="Manual",group="Competition")
public class Manual extends RobotHardware {

    private Executive.StateMachine<Manual> stateMachine;

    // Adding interactive init variables
    private Mutable<Double> DriveSpeed = new Mutable<>(0.7);
    private Mutable<Double> RotationSpeed = new Mutable<>(0.75);
    private Mutable<Double> LiftSpeed = new Mutable<>(1.0);
    private Mutable<Boolean> CoPilot = new Mutable<>(true);
    private Mutable<Double> Exponential = new Mutable<>(1.0);
    private Mutable<Boolean> Debug = new Mutable<>(false);

    // Define interactive init variable holders
    private double lifterSpeed;
    private double exponential;
    private boolean copilotEnabled;
    private double driveSpeed;

    // Define the MecanumNav and other useful variables
    private final double triggerThreshold = 0.1;
    private boolean precisionMode = false;
    private final double precisionSpeed = 0.3;
    private Controller clawController;
    private MecanumNavigation.Navigation2D waypoint = new MecanumNavigation.Navigation2D(0, 0, 0);

    @Override
    public void init() {
        super.init();
        //Changing gamepads to controllers
        controller1 = new Controller (gamepad1);
        controller2 = new Controller (gamepad2);
        clawController = controller2;

        //Adding Interactive init options
        interactiveInit.addOption(DriveSpeed, "Drive Speed Multiplier",  0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0)
                    .addOption(LiftSpeed, "Lifter speed", 0.25, 0.5, 0.75, 1.0)
                    .addOption(RotationSpeed, "Rotation Speed Multiplier",  0.25, 0.5, 0.75, 1.0)
                    .addOption(Exponential, "Exponential",1.0, 3.0)
                    .addOption(CoPilot, "Copilot Enable", true, false)
                    .addOption(Debug, "Debug", true, false);
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
        odometryLocalizer = new OdometryLocalizer(odometryConfig);
        odometryLocalizer.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));
        odometryLocalizer.setEncoderPosition(new OdometryTicks(0,0,0));
        autoDrive = new AutoDrive(this, mecanumNavigation);

        stateMachine = new Executive.StateMachine<>(this);
        stateMachine.changeState(DRIVE, new ManageArmStates());
        stateMachine.init();

        imuUtilities = new IMUUtilities(this,"IMU_1");
        imuUtilities.setCompensatedHeading(0);

        // Assign the variables to Interactive Init values
        lifterSpeed = LiftSpeed.get();
        exponential = Exponential.get();
        driveSpeed = DriveSpeed.get();
        copilotEnabled = CoPilot.get();
    }

    @Override
    public void loop() {
        super.loop();

        stateMachine.update();

        // Update variables with new values
        controller1.update();
        controller2.update();
        mecanumNavigation.update();
        odometryLocalizer.update(new OdometryTicks(getEncoderValue(MotorName.CENTER_WHEEL), getEncoderValue(MotorName.LEFT_WHEEL), getEncoderValue(MotorName.RIGHT_WHEEL)));

        telemetry.addLine("---Drive---");
        // Display the robot's position compared to where it started
        mecanumNavigation.displayPosition();
        telemetry.addData("Dead Wheels: ", odometryLocalizer.getCurrentPosition());

        if(controller1.AOnce()) precisionMode = !precisionMode;
        double precisionOutput = precisionMode ? precisionSpeed : 1;
        telemetry.addData("Precision Mode: ", precisionMode);
        
        imuUtilities.update();
        telemetry.addData("IMU Heading: ", imuUtilities.getCompensatedHeading());

        // Mecanum Drive Control
        setDriveForSimpleMecanum(
                Math.pow(controller1.left_stick_x, exponential) * driveSpeed * precisionOutput,
                Math.pow(controller1.left_stick_y, exponential) * driveSpeed * precisionOutput,
                Math.pow(controller1.right_stick_x, exponential) * driveSpeed * RotationSpeed.get() * precisionOutput,
                Math.pow(controller1.right_stick_y, exponential) * driveSpeed * precisionOutput);

        nonDriveControls();
    }

    /**
     * Robot controls for one or two people, customizable in the InteractiveInit
     */
    private void nonDriveControls() {

        if (copilotEnabled) {
            clawController = controller2;
        } else {
            clawController = controller1;
        }

        // Reset the robot's current position
        if(controller1.YOnce()) {
            imuUtilities.setCompensatedHeading(0);
            mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
        }
        if(Debug.get()) {
            if (controller1.left_trigger > 0.1) {
                waypoint = mecanumNavigation.currentPosition.copy();
            }

            if (controller1.B()) {
                autoDrive.driveToPositionTranslateOnly(waypoint, driveSpeed);
            }
        }

        // Add claw servo controls, operated by Driver if copilot is disabled, or copilot if enabled.
        telemetry.addData("CoPilot Mode", copilotEnabled);

        if (clawController.leftBumper()) {
            commandClaw(ClawPositions.CLOSED);
            telemetry.addData("Claw: ", "CLOSED");
        } else if (clawController.rightBumper()) {
            commandClaw(ClawPositions.OPEN);
            telemetry.addData("Claw: ", "OPEN");
        } else if(clawController.right_trigger > 0.1) {
            commandClaw(ClawPositions.VERTICAL);
            telemetry.addData("Claw: ", "VERTICAL");
        }
    }

    /**
     * @param motorName The motor that will be driven
     * @param targetTicks The position where the motor will be driven. Must be in encoder Ticks
     * @param power The power at which the robot will be driven
     * @param rampThreshold The position when the robot will start slowing the motor down before its destination
     * @return Returns whether or not the motor arrived to the specified position
     */
    private boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power, double rampThreshold) {
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
    private boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power) {
        int rampDistanceTicks = 400;
        return driveMotorToPos (motorName,targetTicks,power,rampDistanceTicks);
    }

    private class ManageArmStates extends Executive.StateBase<Manual> {
        private int placeIndex = 1;
        private boolean arrived = false;
        private int offset_encoder = 0;
        private int lastEncoderPosition = 0;
        Controller armController;

        @Override
        public void update() {
            super.update();

            armController = copilotEnabled ? controller2 : controller1;

            if(gotoManualArmControl()) {
                stateMachine.changeState(ARM, new ManualArmControl());
            } else if(armController.AOnce()) {
                stateMachine.changeState(ARM, new GoToLiftLevel());
            } else if(armController.BOnce()) {
                stateMachine.changeState(ARM, new GoToBottom());
            } else if(armController.YOnce()) {
                placeIndex = 1;
            } else if(armController.dpadUpOnce()) {
                placeIndex++;
            } else if(armController.dpadDownOnce()) {
                placeIndex--;
            } else if(armController.rightStickButtonOnce()) {
                offset_encoder = 0;
            } else if(arrived) {
                stateMachine.changeState(ARM, new ManualArmControl());
            }

            offset_encoder += (int) -armController.right_stick_y * 250 * getAveragePeriodSec();

            telemetry.addLine("---Lifter---");
            telemetry.addData("Offset", offset_encoder);
            telemetry.addData("Place Index", placeIndex);
        }

        private class ManualArmControl extends Executive.StateBase<Manual> {
            @Override
            public void update() {
                super.update();
                if(Math.abs(armController.left_stick_y) > 0.05) {
                    setPower(MotorName.LIFT_WINCH, Math.pow(-armController.left_stick_y, exponential) * lifterSpeed);
                    lastEncoderPosition = getEncoderValue(MotorName.LIFT_WINCH);
                } else {
                    driveMotorToPos(MotorName.LIFT_WINCH, lastEncoderPosition, lifterSpeed);
                }
            }
        }

        private class GoToLiftLevel extends Executive.StateBase<Manual> {
            @Override
            public void update() {
                super.update();
                arrived = driveMotorToPos(MotorName.LIFT_WINCH, Constants.liftArmTicksForLevelFoundationKnob(placeIndex, true, true) + offset_encoder, lifterSpeed);
                lastEncoderPosition = getEncoderValue(MotorName.LIFT_WINCH);
                if(arrived) {
                    placeIndex++;
                }
            }
        }

        private class GoToBottom extends Executive.StateBase<Manual> {
            @Override
            public void update() {
                super.update();
                arrived = driveMotorToPos(MotorName.LIFT_WINCH, offset_encoder, lifterSpeed);
                lastEncoderPosition = getEncoderValue(MotorName.LIFT_WINCH);
            }
        }

        private boolean gotoManualArmControl() {
            double threshold = 0.1;
            if(copilotEnabled) return Math.abs(clawController.right_stick_y) > threshold;
            else return Math.abs(clawController.left_stick_y) > threshold;
        }
    }
}

