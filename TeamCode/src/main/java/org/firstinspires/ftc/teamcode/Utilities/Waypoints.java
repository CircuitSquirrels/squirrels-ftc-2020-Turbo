package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

public class Waypoints {

    // Start position specific parameters
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    double genericRotate = 0;

    // Value should be 1-6, where 1 is near the center of the field, and 6 is adjacent to the outer wall.
    private int skystoneDetectionPosition = 1;

    /**
     * blueLoading waypoints arefirst defined to be used as a template.
     * Then a reflection on the X axis is used to define generic waypoints depending on the
     * start position and team color.
     *
     * These initial waypoints for blueLoading are generated from
     * a list of parameters to ease tweaking.
     *
     * Constructor uses skystoneDetectionPosition as an input.
     */

    /**
     * public waypoints customized for starting location
      */
    public Navigation2D initialPosition;

    public Navigation2D scanPosition_A;
    public Navigation2D grabSkystone_A;
    public Navigation2D backupPosition_A;
    public Navigation2D buildZone_A;

    public Navigation2D scanPosition_B;
    public Navigation2D grabSkystone_B;
    public Navigation2D backupPosition_B;
    public Navigation2D buildZone_B;

    public Navigation2D parkOuter;
    public Navigation2D parkInner;

    /**
     * Template parameter constants
     */
    double cameraOffset = -90; // extra rotation needed to point the camera in given direction

    double stoneWidth = 4;
    double stoneLength = 8;
    double stoneHeight = 4;
    double stoneKnobHeight = 5;
    double tileWidth = 24;

    double robotWidth = 17;
    double robotSidePadding = robotWidth/2;
    double robotFrontPadding = 5.25; //Distance from Robot front edge to wheelbase center.
    double robotBackPadding = 6.25; //Distance from Robot back edge to wheelbase center.

    double loadingStart_X = 0;
    double loadingStart_Y = 0;
    double grabOffset_X_Forward = 9; // Forward on Robot from navigation point, center of drivetrain.
    double grabOffset_Y_Left = 0; // Left on Robot

    double scanOffset_Y = 10;
    double backupDistance = 6;
    double buildZoneOffset = 5;


    /**
     * Blue Loading positions set and used as templates
     */
    Navigation2D blueLoading_initialPosition = new Navigation2D(-tileWidth-robotSidePadding,72-robotBackPadding,degreesToRadians(-90));

    Navigation2D blueLoading_scanPosition_A = new Navigation2D(-tileWidth-robotSidePadding,72-robotBackPadding-scanOffset_Y,degreesToRadians(-90));
    Navigation2D blueLoading_grabSkystone_A = new Navigation2D(-72 + stoneLength * (skystoneDetectionPosition + .5),72 - 47 - stoneWidth * 0.5 + grabOffset_X_Forward,degreesToRadians(-90));
    Navigation2D blueLoading_backupPosition_A = new Navigation2D(-72 + stoneLength * 5.5,72 - 47 - stoneWidth * 0.5 + grabOffset_X_Forward + backupDistance, degreesToRadians(-90));
    Navigation2D blueLoading_buildZone_A = new Navigation2D(tileWidth,tileWidth - buildZoneOffset,degreesToRadians(-90));

    Navigation2D blueLoading_scanPosition_B = new Navigation2D(0,0,degreesToRadians(-90));
    Navigation2D blueLoading_grabSkystone_B = new Navigation2D(0,0,degreesToRadians(-90));
    Navigation2D blueLoading_backupPosition_B = new Navigation2D(0,0,degreesToRadians(-90));
    Navigation2D blueLoading_buildZone_B = new Navigation2D(0,0,degreesToRadians(-90));

    Navigation2D blueLoading_parkOuter = new Navigation2D(0,0,degreesToRadians(-90));
    Navigation2D blueLoading_parkInner = new Navigation2D(0,0,degreesToRadians(-90));

    /**
     * Blue Building positions
     */
    Navigation2D blueBuilding_initialPosition = new Navigation2D(+tileWidth+robotSidePadding,72-robotBackPadding,degreesToRadians(-90));


    public Waypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition, int skystoneDetectionPosition) {
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        this.skystoneDetectionPosition = skystoneDetectionPosition;
        customizeWaypoints(teamColor, startPosition, skystoneDetectionPosition);
    }

    public Waypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        // Default skystone position to 1.
        this(teamColor, startPosition, 1);
    }

    /**
     * @param skystoneDetectionPosition
     * Sets the skystone position and recalculates the waypoint positions by calling customizeWaypoints()
     */
    public void setSkystoneDetectionPosition(int skystoneDetectionPosition) {
        this.skystoneDetectionPosition = skystoneDetectionPosition;
        customizeWaypoints(teamColor, startPosition, skystoneDetectionPosition);
    }

    void customizeWaypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition, int skystoneDetectionPosition) {
        if(teamColor == Color.Ftc.BLUE) {
            if(startPosition == RobotHardware.StartPosition.FIELD_LOADING) {
                // Blue Loading Zone
                create_blue_loading_waypoints();
            } else if (startPosition == RobotHardware.StartPosition.FIELD_BUILD) {
                // Blue Building Zone
                create_blue_build_waypoints();
            } else {
                throw new IllegalStateException("Invalid Starting Position");
            }
        } else if (teamColor == Color.Ftc.RED) {
            if(startPosition == RobotHardware.StartPosition.FIELD_LOADING) {
                // Red Building Zone
                create_blue_loading_waypoints();
                x_reflect_waypoints_in_place();
            } else if (startPosition == RobotHardware.StartPosition.FIELD_BUILD) {
                // Red Loading Zone
                create_blue_build_waypoints();
                x_reflect_waypoints_in_place();
            } else {
                throw new IllegalStateException("Invalid Starting Position");
            }
        } else {
            throw new IllegalStateException("Invalid Team Color");
        }
    }

    // Utility
    double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    void create_blue_loading_waypoints() {
        //Blue Loading Zone
        initialPosition = blueLoading_initialPosition.copy();
        scanPosition_A = blueLoading_scanPosition_A.copy();
        grabSkystone_A = blueLoading_grabSkystone_A.copy();
        backupPosition_A = blueLoading_backupPosition_A.copy();
        buildZone_A = blueLoading_buildZone_A.copy();
        scanPosition_B = blueLoading_scanPosition_B.copy();
        grabSkystone_B = blueLoading_grabSkystone_B.copy();
        backupPosition_B = blueLoading_backupPosition_B.copy();
        buildZone_B = blueLoading_buildZone_B.copy();
        parkOuter = blueLoading_parkOuter.copy();
        parkInner = blueLoading_parkInner.copy();
    }

    void create_blue_build_waypoints() {
        // Blue Building Zone
        initialPosition = blueBuilding_initialPosition.copy();
    }


    void x_reflect_waypoints_in_place() {
        // reflect the generic waypoints around x axis, storing
        // back into the generic waypoints.
        initialPosition.reflectInX();
        scanPosition_A.reflectInX();
        grabSkystone_A.reflectInX();
        backupPosition_A.reflectInX();
        buildZone_A.reflectInX();
        scanPosition_B.reflectInX();
        grabSkystone_B.reflectInX();
        backupPosition_B.reflectInX();
        buildZone_B.reflectInX();
        parkOuter.reflectInX();
        parkInner.reflectInX();

    }
}