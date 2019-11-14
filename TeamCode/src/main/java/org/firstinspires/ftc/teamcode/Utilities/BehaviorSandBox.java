package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;

import java.util.HashMap;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.*;

public class BehaviorSandBox implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine<AutoOpmode> stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;
    double driveSpeed = 0.8;
    Controller controller1;

    public BehaviorSandBox(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor);
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_Menu());
//        stateMachine.changeState(Executive.StateMachine.StateType.ARM, new ArmLevelState());
        stateMachine.init();
        controller1 = opMode.controller1;
    }

    public void update() {
        stateMachine.update();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentStates();
    }

    /**
     * Define Concrete State Classes
     */

    class Start_Menu extends Executive.StateBase {
        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opMode.telemetry.clear();
        }

        @Override
        public void update() {
            super.update();
            opMode.mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            opMode.imuUtilities.updateNow();
            opMode.telemetry.addData("---Start Menu---", "")
                    .addData("Manual: ", "A")
                    .addData("Motor Tester: ", "B")
                    .addData("Servo Tester: ", "X")
                    .addData("Skystone Detector: ", "Y")
                    .addData("Autonomous: ", "Right Bumper");
            if(controller1.AOnce()) stateMachine.changeState(DRIVE, new Manual());
            else if (controller1.BOnce()) stateMachine.changeState(DRIVE, new Motor_Tester());
            else if (controller1.XOnce()) stateMachine.changeState(DRIVE, new Servo_Tester());
            else if (controller1.YOnce()) stateMachine.changeState(DRIVE, new Skystone_Detection());
            else if (controller1.rightBumper()) stateMachine.changeState(DRIVE, new Skystone_Detection());
        }
    }

    class Manual extends Executive.StateBase {
        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opMode.telemetry.clear();
        }

        @Override
        public void update() {
            super.update();
            if(controller1.startOnce()) {
                stateMachine.changeState(DRIVE, new Start_Menu());
            }

            opMode.setDriveForSimpleMecanum(controller1.left_stick_x, controller1.left_stick_y, controller1.right_stick_x, controller1.right_stick_y);

            if(controller1.rightBumper()) {
                opMode.openClaw();
            } else if(controller1.leftBumper()) {
                opMode.closeClaw();
            }

            for (RobotHardware.MotorName m : RobotHardware.MotorName.values()) {
                opMode.telemetry.addData(m.name() + ": ", opMode.getEncoderValue(m));
            }
        }
    }

    class Motor_Tester extends Executive.StateBase {

        double forwardSpeed = 0;
        double rotationalSpeed = 0;
        double strafeSpeed = 0;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opMode.telemetry.clear();
        }

        @Override
        public void update() {
            super.update();
            if(controller1.startOnce()) {
                stateMachine.changeState(DRIVE, new Start_Menu());
            } else if(controller1.AOnce()) {
                forwardSpeed = 0;
                rotationalSpeed = 0;
                strafeSpeed = 0;
            }

            if(controller1.dpadUpOnce()) forwardSpeed = forwardSpeed >= 1.0 ? 1 : forwardSpeed + 0.01;
            else if(controller1.dpadDown()) forwardSpeed = forwardSpeed <= -1 ? -1 : forwardSpeed - 0.01;

            if(controller1.dpadRightOnce()) strafeSpeed = strafeSpeed >= 1.0 ? 1 : strafeSpeed + 0.01;
            else if(controller1.dpadLeftOnce()) strafeSpeed = strafeSpeed <= -1 ? -1 : strafeSpeed - 0.01;

            if(controller1.XOnce()) rotationalSpeed = rotationalSpeed >= 1.0 ? 1 : rotationalSpeed + 0.01;
            else if(controller1.BOnce()) rotationalSpeed = rotationalSpeed <= -1 ? -1 : rotationalSpeed - 0.01;

            opMode.setDriveForSimpleMecanum(strafeSpeed, -forwardSpeed, rotationalSpeed, 0);

            opMode.telemetry.addData("Speed: ", opMode.df.format(forwardSpeed) + ", " + opMode.df.format(strafeSpeed) + ", " + opMode.df.format(rotationalSpeed) + "\n");

            for (RobotHardware.MotorName m : RobotHardware.MotorName.values()) {
                opMode.telemetry.addData(m.name() + ": ", opMode.getEncoderValue(m));
            }
        }
    }

    class Servo_Tester extends Executive.StateBase {

        HashMap<RobotHardware.ServoName, Double> servoPositions = new HashMap<>();
        int servoIndex = 0;
        int maxServoIndex;
        int inputDivider = 10;
        double speed = 1;
        double nextServoPosition;
        RobotHardware.ServoName currentServo;
        boolean disableClawServoTest = true;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opMode.telemetry.clear();
            opMode.verticalClaw();

            for (RobotHardware.ServoName s : RobotHardware.ServoName.values()) {
                try {
                    double previousServoPosition = opMode.getAngle(s);
                    servoPositions.put(s, previousServoPosition);
                } catch (Exception e) {
                    opMode.telemetry.addData("Servo Missing: ", s.name());
                }
            }
            maxServoIndex = servoPositions.size()-1;
        }

        @Override
        public void update() {
            super.update();

            // Change state back to Menu
            if(controller1.startOnce()) {
                stateMachine.changeState(DRIVE, new Start_Menu());
            }

            // Select Index of servo to control.
            if(controller1.dpadUpOnce() && servoIndex < maxServoIndex) ++servoIndex;
            if(controller1.dpadDownOnce() &&  servoIndex > 0) --servoIndex;

            // Add a customizable controller input divider for more precise testing.
            if(controller1.dpadRightOnce()) inputDivider *= 10;
            if(controller1.dpadLeftOnce()) inputDivider /=  10;

            // Get the current servo that is selected and move it to the new position
            currentServo = RobotHardware.ServoName.values()[servoIndex];
            nextServoPosition = Range.clip(-controller1.left_stick_y / inputDivider + servoPositions.get(currentServo), -1, 1);
            servoPositions.put(currentServo, nextServoPosition);

            //Move Servo to position stored in servoPositions HashMap
            for (RobotHardware.ServoName s : RobotHardware.ServoName.values()) {
                try {
                    if(disableClawServoTest && (s == RobotHardware.ServoName.CLAW_LEFT || s == RobotHardware.ServoName.CLAW_RIGHT)) {
                        opMode.telemetry.addData("SERVO DISABLED: ","Claw Servos are Physically Attached!");
                        continue; // Skip this part of the loop to prevent moving servo incorrectly.
                    }
                    opMode.setAngle(s, servoPositions.get(s));

                } catch (Exception e) {
                    opMode.telemetry.addData("Error couldn't set server position for:  ", s + ", " + opMode.df.format(servoPositions.get(s)));
                }
            }
            opMode.telemetry.addData("Input Divider: ", inputDivider)
                    .addData("Servo: ", servoIndex + ", " + currentServo)
                    .addData("Servo Angle: ", opMode.df.format(servoPositions.get(currentServo)));
        }
    }

    class Start extends Executive.StateBase {
        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
        }

        @Override
        public void update() {
            super.update();
        }
    }

    class Skystone_Detection extends Executive.StateBase {
        double skystone_absolute_x = 0;
        double bot_absolute_x = 0;
        double bot_relative_to_skystone_y = 0;
        double skystone_index_double = 0;
        int skystone_index = 0;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opMode.mecanumNavigation.setCurrentPosition(waypoints.loading.get(scanPosition_A));
        }

        @Override
        public void update() {
            super.update();
            if(opMode.controller1.startOnce()) stateMachine.changeState(DRIVE,new Start_Menu());

            if(opMode.simpleVision == null) {
                opMode.telemetry.addData("Vision: ", "Not Loaded.");
                return;
            }

            bot_absolute_x = opMode.mecanumNavigation.currentPosition.x;
            bot_relative_to_skystone_y = opMode.simpleVision.getPositionSkystoneRelativeNav2d().y;
            skystone_absolute_x = bot_absolute_x - bot_relative_to_skystone_y;
            skystone_index_double = 6.5 -(skystone_absolute_x + 72) / 8;
            skystone_index = (int) Math.round(skystone_index_double);

            opMode.telemetry.addData("Bot Absolute: ", bot_absolute_x)
                    .addData("Bot Relative To Skystone: ", bot_relative_to_skystone_y)
                    .addData("Skystone Index: ", skystone_index_double)
                    .addData("Skystone Index Rounded: ", skystone_index);
        }
    }

    class Auto extends Executive.StateBase<AutoOpmode> {
        RobotStateContext robotStateContext;

        @Override
        public void init(Executive.StateMachine stateMachine) {
            super.init(stateMachine);
            opMode.telemetry.clear();
            robotStateContext = new RobotStateContext(opMode, teamColor, startPosition);
            robotStateContext.init();
        }

        @Override
        public void update() {
            super.update();
            robotStateContext.update();
            opMode.telemetry.addData("Sub State: ", robotStateContext.getCurrentState());
            if(controller1.startOnce()) stateMachine.changeState(DRIVE, new Start_Menu());
        }
    }

    class Stop_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
        }
    }

    /* Class template for easy copy paste.

     class Template extends Executive.StateBase {
            @Override
            public void init(Executive.StateMachine stateMachine) {
                super.init(stateMachine);
            }

            @Override
            public void update() {
                super.update();
            }
     }
     */


    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }
}
