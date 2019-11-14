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
    int liftRaised = 1000;




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
            Navigation2D initialPosition;
            switch (startPosition) {
                case FIELD_LOADING:
                    setupInitialPosition(waypoints.loading.get(Waypoints.LocationLoading.initialPosition));
                    stateMachine.changeState(DRIVE, new Start_Loading_Side());
                    break;
                case FIELD_BUILD:
                    setupInitialPosition(waypoints.building.get(Waypoints.LocationBuild.initialPosition));
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

    class Start_Loading_Side extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            if(stateTimer.seconds() > 1 && opMode.shouldContinue() && simple) stateMachine.changeState(DRIVE, new Simple_Start());
            else if(stateTimer.time() > 1 && opMode.shouldContinue()) stateMachine.changeState(DRIVE, new Scan_Position_A());
        }
    }

    class Scan_Position_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(scanPosition_A), driveSpeed);
            if(arrived) {
                waypoints.setSkystoneDetectionPosition(1);
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Goto_Skystone_A());
                    stateMachine.changeState(ARM, new Raise_Lift_Open());
                }
            }
        }
    }

    class Goto_Skystone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(grabSkystone_A), driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(ARM, new Grab_Skystone());
                    if (stateMachine.getStateReference(ARM).arrived && opMode.shouldContinue()) stateMachine.changeState(DRIVE, new Backup_A());
                }
            }
        }
    }

    class Backup_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(backupPosition_A), driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) stateMachine.changeState(DRIVE, new Build_Zone_A());
            }
        }
    }

    class Build_Zone_A extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(buildZone_A), driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Stop_State());
                    stateMachine.changeState(ARM, new Raise_Lift_Open());
                }
            }
        }
    }

    class Raise_Lift_Open extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised, liftSpeed);
            if(arrived) opMode.openClaw();
        }
    }

    class Grab_Skystone extends Executive.StateBase<AutoOpmode> {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised, liftSpeed);
            if(arrived) opMode.closeClaw();
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
            if(parkInner) arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(simpleAlignment_Inner), driveSpeed);
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

            if(parkInner) arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(parkInner), driveSpeed);
            else arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.loading.get(parkOuter), driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Stop_State());
                }
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
