package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Ashley on 12/16/2017.
 */

public class RobotStateMachine {

    public enum AutoState {
        START,
        LAND,
        UNHOOK,
        DISMOUNT,
        IDENTIFY_CENTER,
        IDENTIFY_LEFT,
        IDENTIFY_RIGHT,
        ALIGN_CENTER_MINERAL,
        ALIGN_LEFT_MINERAL,
        ALIGN_RIGHT_MINERAL,
        UNKNOWN,
        KNOCK_GOLD_CENTER,
        KNOCK_GOLD_LEFT,
        KNOCK_GOLD_RIGHT,
        ALIGN_CENTER_DEPOT,
        ALIGN_LEFT_DEPOT,
        ALIGN_RIGHT_DEPOT,
        DRIVE_DEPOT,
        PHOTO_ROTATE,
        JOLT_FEEDER_ARM,
        EARLY_DROP_FLAG,
        DROP_FLAG,
        CLAIM_DEPOT,
        DRIVE_CRATER,
        ENTER_CRATER,
        STOP,
        SIMPLE_CRATER,
        SIMPLE_DEPOT,
        SIMPLE_START,
    }

    public AutoState state = AutoState.START;
    private AutoOpmode opMode;
    private ElapsedTime stateLoopTimer = new ElapsedTime();
    private double lastStateLoopPeriod = 0;
    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime mineralIdentificationTimer = new ElapsedTime();
    private Waypoints waypoints;

    // Colors
    public Color.Ftc teamColor = Color.Ftc.UNKNOWN;
    public Color.Mineral centerMineral = Color.Mineral.UNKNOWN;
    public Color.Mineral leftMineral = Color.Mineral.UNKNOWN;
    public Color.Mineral rightMineral = Color.Mineral.UNKNOWN;
    public SimpleVision.GoldMineralPosition goldMineralPosition = SimpleVision.GoldMineralPosition.UNKNOWN;

    // Kinematics
    private ArrayList<MecanumNavigation.Navigation2D> waypointArrayGlobal;

    public RobotHardware.StartPosition startPosition;
    private int currentDriveWaypoint = 0;

    public double speed = 1;
    public boolean foundMineral = false;
    public boolean centerGold = false;
    public boolean arrived = false;
    boolean lifter_arrived = false;

    private ArrayList<MecanumNavigation.Navigation2D> simpleWaypointArray;

    DcMotor armMotor;


    public RobotStateMachine(AutoOpmode opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
    }

    public void init() {
        stateLoopTimer.reset();
        stateTimer.reset();
        mineralIdentificationTimer.reset();
        waypoints = new Waypoints(teamColor, startPosition, opMode.doPartnerMinerals.get());
    }

    public boolean driveMotorToPos (RobotHardware.MotorName motorName, int targetTicks, double power, double rampThreshold) {
        power = Range.clip(Math.abs(power), 0, 1);
        int poweredDistance = 0;
        int arrivedDistance = 50;
        //int rampThreshold = 400;
        double maxRampPower = 1.0;
        double minRampPower = 0.0;
        int errorSignal = opMode.getEncoderValue(motorName) - targetTicks;
        double direction = errorSignal > 0 ? -1.0: 1.0;
        double rampDownRatio = AutoDrive.rampDown(Math.abs(errorSignal), rampThreshold, maxRampPower, minRampPower);

        if (Math.abs(errorSignal) >= poweredDistance) {
            opMode.setPower(motorName, direction * power * rampDownRatio);
        } else {
            opMode.setPower(motorName, 0);
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


    public void update() {

        double identificationTime = 2;
        double timeout = 10; // Identification timeout.
        lastStateLoopPeriod = stateLoopTimer.seconds();
        stateLoopTimer.reset();

        if (state != AutoState.DROP_FLAG) {
            driveMotorToPos(RobotHardware.MotorName.ARM, 250, 1.0);
        }

        speed = opMode.AutoDriveSpeed.get();

        if (state == AutoState.START) {
            if (opMode.Simple.get()) {
                stateTimer.reset();
                state = AutoState.SIMPLE_START;

            } else {
                // This needs to be set based on our starting location.
                opMode.mecanumNavigation.setCurrentPosition(waypoints.initialPosition);
                if (opMode.useIMU.get()) {
                    opMode.imuUtilities.updateNow();
                    opMode.imuUtilities.setCompensatedHeading(radiansToDegrees(waypoints.initialPosition.theta));
                }
                stateTimer.reset();
                state = AutoState.LAND;
            }
        } else if (state == AutoState.LAND) {
            arrived = driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, Constants.LIFTER_MAX_TICKS, 1.0);

            if (arrived || stateTimer.seconds() >= 8) {
                opMode.stopAllMotors();
                updateMecanumHeadingFromGyro();
                stateTimer.reset();
                state = AutoState.UNHOOK;
            }
        } else if (state == AutoState.UNHOOK) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.unhookPosition, speed);
            if (arrived) {
                state = AutoState.DISMOUNT;
                stateTimer.reset();
            }
        } else if (state == AutoState.DISMOUNT) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.dismountPosition, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.ALIGN_CENTER_MINERAL;
                updateMecanumHeadingFromGyro();
            }
        } else if (state == AutoState.ALIGN_CENTER_MINERAL) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_center, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.IDENTIFY_CENTER;
            }

        } else if (state == AutoState.IDENTIFY_CENTER) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.scanMineral_center, speed);
            lifter_arrived = driveMotorToPos(RobotHardware.MotorName.LIFT_WINCH, Constants.LIFTER_MIN_TICKS, 1.0);
            if (!arrived || !lifter_arrived) {
                mineralIdentificationTimer.reset();
            }

            if (arrived && lifter_arrived && mineralIdentificationTimer.seconds() > identificationTime && stateTimer.seconds() < timeout) {
                // Detect mineral at image center
                centerMineral = opMode.simpleVision.identifyMineral(SimpleVision.MineralIdentificationLocation.BOTTOM);

                if (centerMineral == Color.Mineral.GOLD) {
                    opMode.soundManager.play("gold");
                    stateTimer.reset();
                    state = AutoState.KNOCK_GOLD_CENTER;
                } else if(centerMineral == Color.Mineral.SILVER){
                    opMode.soundManager.play("silver");
                    stateTimer.reset();
                    state = AutoState.IDENTIFY_LEFT;
                }

            } else if (stateTimer.seconds() >= timeout) {
                // Timed out: assume detection wasn't possible, act as if it were gold.
                stateTimer.reset();
                state = AutoState.KNOCK_GOLD_CENTER; // Problematic.
            }
        } else if (state == AutoState.IDENTIFY_LEFT) {
            // First rotate robot to point camera toward the left mineral.
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_left, speed);
            if (!arrived) {
                mineralIdentificationTimer.reset();
            }

            // Detect mineral at image center
            leftMineral = opMode.simpleVision.identifyMineral(SimpleVision.MineralIdentificationLocation.BOTTOM);
            if (mineralIdentificationTimer.seconds() > identificationTime && stateTimer.seconds() < timeout) {
                if (leftMineral == Color.Mineral.GOLD) {
                    if (arrived) { // Why only check for arrived on GOLD mineral?
                        opMode.soundManager.play("gold");
                        foundMineral = true;
                        stateTimer.reset();
                        state = AutoState.KNOCK_GOLD_LEFT;
                    }
                } else if (leftMineral == Color.Mineral.SILVER) {
                    opMode.soundManager.play("silver");
                    stateTimer.reset();
                    state = AutoState.ALIGN_RIGHT_MINERAL;
                }
            } else if (stateTimer.seconds() >= timeout) {
                stateTimer.reset();
                state = AutoState.KNOCK_GOLD_LEFT;
            }
        } else if (state == AutoState.ALIGN_RIGHT_MINERAL) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.alignMineral_right, speed);
            if(arrived && stateTimer.seconds() > 1 ) {
                    stateTimer.reset();
                    state = AutoState.KNOCK_GOLD_RIGHT;
            }

        } else if (state == AutoState.KNOCK_GOLD_CENTER)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_center, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.ALIGN_CENTER_DEPOT;
            }
        } else if (state == AutoState.KNOCK_GOLD_LEFT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_left, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.ALIGN_LEFT_DEPOT;
            }
        } else if (state == AutoState.KNOCK_GOLD_RIGHT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.knockMineral_right, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.ALIGN_RIGHT_DEPOT;
            }
        } else if (state == AutoState.ALIGN_CENTER_DEPOT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.depot_Alignment_center, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.DRIVE_DEPOT;
                updateMecanumHeadingFromGyro();
            }
        } else if (state == AutoState.ALIGN_RIGHT_DEPOT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.depot_Alignment_right, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.DRIVE_DEPOT;
                updateMecanumHeadingFromGyro();
            }
        } else if (state == AutoState.ALIGN_LEFT_DEPOT)  {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.depot_Alignment_left, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.DRIVE_DEPOT;
                updateMecanumHeadingFromGyro();
            }
        } else if (state == AutoState.DRIVE_DEPOT) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.photoPosition, speed);

            if (arrived) {
                state = AutoState.PHOTO_ROTATE;
                stateTimer.reset();
            }
        } else if (state == AutoState.PHOTO_ROTATE) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.photoRotate, speed);
            if (arrived) {
                stateTimer.reset();
                state = AutoState.JOLT_FEEDER_ARM;
            }

        } else if (state == AutoState.JOLT_FEEDER_ARM) {
            double joltDelay = 1.0;
            double joltInterval = 0.1;
            double joltSpeed = 1.0;

            if (stateTimer.seconds() > joltDelay && stateTimer.seconds() <= joltDelay + joltInterval) {
                opMode.setDriveForTank(joltSpeed, joltSpeed);
            } else if (stateTimer.seconds() > joltDelay + joltInterval && stateTimer.seconds() <= joltDelay + 2 * joltInterval) {
                opMode.setDriveForTank(-joltSpeed, -joltSpeed);
            } else if (stateTimer.seconds() > joltDelay + 2 * joltInterval) {
                state = AutoState.DROP_FLAG;
                stateTimer.reset();
            }

        } else if (state == AutoState.DROP_FLAG) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.flagDrop, speed);
            if (arrived) {
                arrived = driveMotorToPos(RobotHardware.MotorName.ARM, 8787, speed);
                if (arrived) {
                    updateMecanumHeadingFromGyro();
                    stateTimer.reset();
                    state = AutoState.CLAIM_DEPOT;
                }
            }


        } else if (state == AutoState.CLAIM_DEPOT) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.depotPush, speed);
            if (arrived) {
                updateMecanumHeadingFromGyro();
                stateTimer.reset();
                state = AutoState.DRIVE_CRATER;
            }

        } else if (state == AutoState.DRIVE_CRATER) {
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypoints.craterPark, speed);
                if (arrived) {
                    state = AutoState.STOP;
                    stateTimer.reset();
                }
        } else if (state == AutoState.SIMPLE_START) {
            // In these waypoint lists, the first entry is only used to set the
            // initial position for navigating.
            // The last position is moved to at full speed to get into the crater.
            // The intermediate positions are moved to at a reduced speed.
            // Motions first rotate, then translate.

            if (startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                simpleWaypointArray = new ArrayList<>(Arrays.asList(
                    // START POSITION (used to set mecanumNavigation initial position)
                    new MecanumNavigation.Navigation2D(19, 19, degreesToRadians(-45)),
                    // DISMOUNT (make space to turn)
                    new MecanumNavigation.Navigation2D(24, 24, degreesToRadians(-45)),
                    // ROTATE IN PLACE
                    new MecanumNavigation.Navigation2D(24, 24, degreesToRadians(45)),
                    // JUMP INTO CRATER (full speed waypoint)
                    new MecanumNavigation.Navigation2D(48, 48, degreesToRadians(45))
                ));

            } else {
                simpleWaypointArray = new ArrayList<>(Arrays.asList(
                    // START POSITION
                    new MecanumNavigation.Navigation2D(-19, 19, degreesToRadians(45)),
                    // DISMOUNT
                    new MecanumNavigation.Navigation2D(-24, 24, degreesToRadians(45)),
                    // ROTATE IN PLACE
                    new MecanumNavigation.Navigation2D(-24, 24, degreesToRadians(270)),
                    // HIT LEFT MINERAL
                    new MecanumNavigation.Navigation2D(-48, 24, degreesToRadians(270)),
                    // LINE UP WITH CRATER
                    new MecanumNavigation.Navigation2D(-60, 24, degreesToRadians(270)),
                    // JUMP INTO CRATER (full speed)
                    new MecanumNavigation.Navigation2D(-60, -48, degreesToRadians(270))

                ));
            }

            currentDriveWaypoint = 1;
            opMode.mecanumNavigation.setCurrentPosition( simpleWaypointArray.get(0));
            stateTimer.reset();
            if (startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                state = AutoState.SIMPLE_CRATER;
            } else {
                state = AutoState.SIMPLE_DEPOT;
            }


        } else if (state == AutoState.SIMPLE_CRATER ) {
            boolean done = false;

            if (!opMode.controller.X()) { // Pause Robot while X button is held.
                done = simpleWaypointDrive(simpleWaypointArray);
            } else {
                opMode.stopAllMotors();
            }

            if(done) {
                stateTimer.reset();
                state = AutoState.STOP;
                opMode.stop();
            }


        } else if (state == AutoState.SIMPLE_DEPOT) {
            state = AutoState.SIMPLE_CRATER; // They are the same right now.

        } else if (state == AutoState.STOP) {
            opMode.stopAllMotors();
            opMode.stop();
        } else {
            // error
            opMode.stopAllMotors();
        }
    }

    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    private boolean simpleWaypointDrive(ArrayList<MecanumNavigation.Navigation2D> waypointList) {
        boolean arrived = false;
        boolean finalArrived = false;
        // Skip first point, used for setting navigation start in SIMPLE_START
        if (currentDriveWaypoint < 1) {
            currentDriveWaypoint = 1;
        }
        // So we can ramp up speed on LAST waypoint to get into CRATER
        int lastIndex = waypointList.size() - 1;
        int secondToLastIndex = waypointList.size() - 2;
        if (currentDriveWaypoint <= secondToLastIndex){
            arrived = opMode.autoDrive.rotateThenDriveToPosition(waypointList.get(currentDriveWaypoint),0.5);
            if(arrived) {
                currentDriveWaypoint++;
            }
        }
        if (currentDriveWaypoint == lastIndex) {
            // Ramming SPEED! Get over crater.
            finalArrived = opMode.autoDrive.rotateThenDriveToPosition(waypointList.get(lastIndex),1.0);
        }
        if (finalArrived) {
            opMode.stopAllMotors();
        }
        opMode.telemetry.addData("Simple Waypoint Drive","");
        opMode.telemetry.addData("Current Waypoint: ", currentDriveWaypoint);
        opMode.telemetry.addData("Target", waypointList.get(currentDriveWaypoint).toString());
        return finalArrived;
    }


    private void updateMecanumHeadingFromGyro() {
        if (opMode.useIMU.get()) {
            // Modify current position to account for rotation during descent measured by gyro.
            opMode.imuUtilities.updateNow();
            double gyroHeading = opMode.imuUtilities.getCompensatedHeading();
            MecanumNavigation.Navigation2D currentPosition = opMode.mecanumNavigation.currentPosition.copy();
            currentPosition.theta = degreesToRadians(gyroHeading);
            opMode.mecanumNavigation.setCurrentPosition(currentPosition);
        }
    }
}
