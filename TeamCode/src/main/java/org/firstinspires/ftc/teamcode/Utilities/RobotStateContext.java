package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.ARM;
import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.DRIVE;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    RobotHardware opMode;
    Executive.StateMachine stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;
    double driveSpeed;
    boolean simple;
    boolean parkInner;

    double liftSpeed = 1;
    int liftRaised = 1000;




    public RobotStateContext(RobotHardware opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor, startPosition);
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

    class Start_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.mecanumNavigation.setCurrentPosition(waypoints.initialPosition);
            opMode.imuUtilities.updateNow();
            opMode.imuUtilities.setCompensatedHeading(radiansToDegrees(waypoints.initialPosition.theta));
            if(stateTimer.seconds() > 1 && opMode.shouldContinue() && simple) stateMachine.changeState(DRIVE, new Simple_Start());
            else if(stateTimer.time() > 1 && opMode.shouldContinue()) stateMachine.changeState(DRIVE, new Scan_Position_A());
        }
    }

    class Scan_Position_A extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.scanPosition_A, driveSpeed);
            if(arrived) {
                waypoints.setSkystoneDetectionPosition(1);
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Goto_Skystone_A());
                    stateMachine.changeState(ARM, new Raise_Lift_Open());
                }
            }
        }
    }

    class Goto_Skystone_A extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.grabSkystone_A, driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(ARM, new Grab_Skystone());
                    if (stateMachine.getStateReference(ARM).arrived && opMode.shouldContinue()) stateMachine.changeState(DRIVE, new Backup_A());
                }
            }
        }
    }

    class Backup_A extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.backupPosition_A, driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) stateMachine.changeState(DRIVE, new Build_Zone_A());
            }
        }
    }

    class Build_Zone_A extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.buildZone_A, driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Stop_State());
                    stateMachine.changeState(ARM, new Raise_Lift_Open());
                }
            }
        }
    }

    class Raise_Lift_Open extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised, liftSpeed);
            if(arrived) opMode.openClaw();
        }
    }

    class Grab_Skystone extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised, liftSpeed);
            if(arrived) opMode.closeClaw();
        }
    }

    class Simple_Start extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            if(opMode.shouldContinue()) {
                stateMachine.changeState(DRIVE, new Simple_Align());
            }
        }
    }

    class Simple_Align extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            if(parkInner) arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.simpleAlignment_Inner, driveSpeed);
            else arrived = true;
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Simple_Park());
                }
            }
        }
    }

    class Simple_Park extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            if(parkInner) arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.parkInner, driveSpeed);
            else arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.parkOuter, driveSpeed);
            if(arrived) {
                if(opMode.shouldContinue()) {
                    stateMachine.changeState(DRIVE, new Stop_State());
                }
            }
        }
    }


    class Stop_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.stopAllMotors();
            stateMachine.removeStateType(ARM);
        }
    }



    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }
}
