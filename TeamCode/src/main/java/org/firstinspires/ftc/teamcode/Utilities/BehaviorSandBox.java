package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.DRIVE;

public class BehaviorSandBox implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;
    double driveSpeed = 0.8;
    Controller controllerDrive;

    public BehaviorSandBox(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor, startPosition, true);
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_Menu());
//        stateMachine.changeState(Executive.StateMachine.StateType.ARM, new ArmLevelState());
        stateMachine.init();
        controllerDrive = opMode.controller;
    }

    public void update() {
        stateMachine.update();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentState();
    }

    /**
     * Define Concrete State Classes
     */



    class Start_Menu extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 1) {
                opMode.mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
                opMode.imuUtilities.updateNow();
                opMode.telemetry.addData("---Start Menu---", "")
                        .addData("Manual: ", "A");
                if(controllerDrive.AOnce()) {
                    stateMachine.changeState(DRIVE, new Manual());
                }
            }
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
            if(controllerDrive.startOnce()) {
                stateMachine.changeState(DRIVE, new Start_Menu());
            }

            opMode.setDriveForSimpleMecanum(controllerDrive.left_stick_x, controllerDrive.left_stick_y, controllerDrive.right_stick_x, controllerDrive.right_stick_y);

            for (RobotHardware.MotorName m : RobotHardware.MotorName.values()) {
                opMode.telemetry.addData(m.name() + ": ", opMode.getEncoderValue(m));
            }
        }
    }

    class Stop_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
        }
    }



    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }
}
