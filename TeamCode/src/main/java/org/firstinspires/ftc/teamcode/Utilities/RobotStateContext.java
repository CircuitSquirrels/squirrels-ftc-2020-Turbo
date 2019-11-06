package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.ARM;
import static org.firstinspires.ftc.teamcode.Utilities.Executive.StateMachine.StateType.DRIVE;

public class RobotStateContext implements Executive.RobotStateMachineContextInterface {

    AutoOpmode opMode;
    Executive.StateMachine stateMachine;
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    Waypoints waypoints;

    double liftSpeed = 1;
    double driveSpeed = 0.8;

    int liftRaised = 1000;

    String set = "a";


    public RobotStateContext(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        stateMachine = new Executive.StateMachine(opMode);
        waypoints = new Waypoints(teamColor, startPosition);
    }

    public void init() {
        stateMachine.changeState(DRIVE, new Start_State());

        stateMachine.init();
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

    class Start_State extends Executive.StateBase {
        @Override
        public void update() {
            super.update();
            opMode.mecanumNavigation.setCurrentPosition(waypoints.initialPosition);
            opMode.imuUtilities.updateNow();
            opMode.imuUtilities.setCompensatedHeading(radiansToDegrees(waypoints.initialPosition.theta));
            stateMachine.changeState(DRIVE, new Scan_Position());
        }
    }

    class Scan_Position extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.scanPosition_A, driveSpeed);
            if(arrived) {
                waypoints.setSkystoneDetectionPosition(1);
                stateMachine.changeState(DRIVE, new Goto_Skystone_A());
                stateMachine.changeState(ARM, new Raise_Lift());
            }
        }
    }

    class Goto_Skystone_A extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.grabSkystone_A, driveSpeed);
            if(arrived) {
                stateMachine.changeState(ARM, new Grab_Skystone());
            }
        }
    }

    class Backup_A extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.backupPosition_A, driveSpeed);
            if(arrived) {
                stateMachine.changeState(DRIVE, new Build_Zone_A());
            }
        }
    }

    class Build_Zone_A extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.buildZone_A, driveSpeed);
            if(arrived) {
                stateMachine.changeState(DRIVE, new Stop_State());
            }
        }
    }

    class Raise_Lift extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised, 1);
        }
    }

    class Grab_Skystone extends Executive.StateBase {
        @Override
        public void update() {
            super.update();

            arrived = opMode.autoDrive.driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, liftRaised, 1);
            if(arrived) {
                opMode.closeClaw();
                if(set.equals("a")) stateMachine.changeState(DRIVE, new Backup_A());
                else stateMachine.changeState(DRIVE, new Stop_State());
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
