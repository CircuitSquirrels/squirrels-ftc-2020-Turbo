package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.*;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine<AutoOpmode> stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;
    double driveSpeed;
    boolean simple;
    boolean parkInner;

    double liftSpeed = 1;
    int liftRaised = 1500;

    private Controller controller1;


    public RobotStateContext(AutoOpmode opMode, Color.Ftc teamColor, AutoOpmode.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor);
        stateMachine.update();
        driveSpeed = opMode.AutoDriveSpeed.get();
        simple = opMode.SimpleAuto.get();
        parkInner = opMode.ParkInner.get();
        controller1 = opMode.controller1;
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_State());

        stateMachine.init();
    }



    public void update() {
        stateMachine.update();
        driveSpeed = opMode.AutoDriveSpeed.get();
        simple = opMode.SimpleAuto.get();
        parkInner = opMode.ParkInner.get();
    }

    public String getCurrentState() {
        return stateMachine.getCurrentStates();
    }

    /**
     * Define Concrete State Classes
     */

    // Fork between Build and Load side starting positions.
    class Start_State extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            switch (startPosition) {
                case FIELD_LOADING:
                    setupInitialPosition(waypoints.loading.get(Waypoints.LocationLoading.INITIAL_POSITION));
                    stateMachine.changeState(DRIVE, new Start_Loading_Side());
                    break;
                case FIELD_BUILD:
                    setupInitialPosition(waypoints.building.get(Waypoints.LocationBuild.INITIAL_POSITION));
                    stateMachine.changeState(DRIVE, new Start_Building_Side());
                    break;
                default:
                   throw new IllegalStateException("Field Position must be either FIELD_LOADING or FIELD_BUILD");
            }
        }

        private void setupInitialPosition(Navigation2D initialPosition) {
            opMode.mecanumNavigation.setCurrentPosition(initialPosition);
            opMode.imuUtilities.updateNow();
            opMode.imuUtilities.setCompensatedHeading(radiansToDegrees(initialPosition.theta));
        }
    }

    /**
     * Loading Drive State
     * The initial start state
     */
    class Start_Loading_Side extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 1 && opMode.shouldContinue() && simple) stateMachine.changeState(DRIVE, new Simple_Start());
            else if(stateTimer.time() > 1 && opMode.shouldContinue()) stateMachine.changeState(DRIVE, new Scan_Position_A());
        }
    }
    /**
     * Loading Drive State
     * The state for scanning the first 3 stones to identify the skystone
     */
    class Scan_Position_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
            stateMachine.changeState(ARM, new Lower_Open_Claw());
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(SCAN_POSITION_A), driveSpeed);
            if(arrived) {
                stateMachine.changeState(DRIVE, new Align_Skystone_A());
            }
        }
    }
    /**
     * Loading Drive State
     * The state for aligning in-front of the detected skystone
     */
    class Align_Skystone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
            waypoints.setSkystoneDetectionPosition(0);
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(ALIGNMENT_POSITION_A), driveSpeed);
            if(arrived) {
                stateMachine.changeState(DRIVE, new Grab_Skystone_A());
            }
        }
    }
    /**
     * Loading Drive State
     * The state for grabbing the detected skystone
     */
    class Grab_Skystone_A extends Executive.StateBase<AutoOpmode> {
        boolean armStateCreated = false;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(GRAB_SKYSTONE_A), driveSpeed);
            if(arrived) {
                if(!armStateCreated) {
                    stateMachine.changeState(ARM, new Lower_Close_Claw());
                    armStateCreated = true;
                }
                if(stateMachine.getStateReference(ARM).arrived) {
                    stateMachine.changeState(DRIVE, new Backup_Skystone_A());
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for backing up from the detected skystone to avoid knocking other stones over
     */
    class Backup_Skystone_A extends Executive.StateBase<AutoOpmode> {
        boolean armStateCreated = false;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
        }

        @Override
        public void update() {
            super.update();
            if (stateTimer.seconds() > 1) {
                if(!armStateCreated) {
                    stateMachine.changeState(ARM, new Raise_Close_Claw());
                    armStateCreated = true;
                }
                if (stateMachine.getStateReference(ARM).arrived) {
                    arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(ALIGNMENT_POSITION_A), driveSpeed);
                    if (arrived) {
                        stateMachine.changeState(DRIVE, new Build_Zone());
                    }
                }
            }
        }
    }
    /**
     * Loading Drive State
     * The state for driving to the build zone
     */
    class Build_Zone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(BUILD_ZONE), driveSpeed);

            if(arrived) {
                stateMachine.changeState(DRIVE, new Place_Foundation());
            }
        }
    }

    class Place_Foundation extends Executive.StateBase<AutoOpmode> {
        boolean armStateExists = false;
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
        }

        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(new Navigation2D(37.32, 25.74, degreesToRadians(-90)), driveSpeed);
            if(arrived) {
                if(!stateMachine.getCurrentStates(ARM).equals("Place_On_Foundation")) {
                    stateMachine.changeState(ARM, new Place_On_Foundation());
                    armStateExists = true;
                }
                if(stateMachine.getStateReference(ARM).arrived) {
                    stateMachine.changeState(DRIVE, new A_Manual());
                }
            }
        }
    }

    class Raise_Open_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised,liftSpeed);
            if(arrived) {
                opMode.openClaw();
            }
        }
    }

    class Raise_Close_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised,liftSpeed);
            if(arrived) {
                opMode.closeClaw();
            }
        }
    }

    class Lower_Close_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, 0,liftSpeed);
            if(arrived) {
                opMode.closeClaw();
            }
        }
    }

    class Place_On_Foundation extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, 800,liftSpeed);
            if(arrived) {
                opMode.closeClaw();
            }
        }
    }

    class Lower_Open_Claw extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.openClaw();
            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, 0,liftSpeed);
        }
    }

    class Simple_Start extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            if(opMode.shouldContinue()) {
                stateMachine.changeState(DRIVE, new Simple_Align());
            }
        }
    }

    class Simple_Align extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(parkInner) arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(SIMPLE_ALIGNMENT_INNER), driveSpeed);
            else arrived = true;
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Simple_Park());
                }
            }
        }
    }

    class Simple_Park extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            if(parkInner) arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(PARK_INNER), driveSpeed);
            else arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(PARK_OUTER), driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Stop_State());
                }
            }
        }
    }

    class A_Manual extends Executive.StateBase<AutoOpmode> {
        @Override
        public void init(Executive.StateMachine<AutoOpmode> stateMachine) {
            super.init(stateMachine);
            opMode.updateMecanumHeadingFromGyroNow();
            opMode.telemetry.clear();
            stateMachine.removeStateType(ARM);
        }

        @Override
        public void update() {
            super.update();

            opMode.setDriveForSimpleMecanum(controller1.left_stick_x * 0.2, controller1.left_stick_y * 0.2, controller1.right_stick_x * 0.2, controller1.right_stick_y * 0.2);
            opMode.setPower(RobotHardware.MotorName.LIFT_WINCH, controller1.right_stick_y);
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

    class Stop_State extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
            stateMachine.removeStateType(ARM);
        }
    }


    class Start_Building_Side extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            // TODO: Provide real building side routine. Move the foundation?
            nextState(DRIVE, new Simple_Park());
        }
    }


    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }
}
