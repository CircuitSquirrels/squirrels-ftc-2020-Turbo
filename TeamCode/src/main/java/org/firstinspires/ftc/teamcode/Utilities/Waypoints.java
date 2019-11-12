package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.EnumMap;
import java.util.Set;


public class Waypoints {

    // Both Build and Load waypoints are built for given Color.
    Color.Ftc teamColor;

    // Value should be 0-5, where 0 is near the center of the field, and 5 is adjacent to the outer wall.
    private int skystoneDetectionPosition = 0;

    public Color.Ftc getTeamColor() {
        return teamColor;
    }

    public int getSkystoneDetectionPosition() {
        return skystoneDetectionPosition;
    }

    // Constructor, color and skystone detection position.
    // skystoneDetectionPosition is 0-5, from field center to the front wall.
    public Waypoints(Color.Ftc teamColor, int skystoneDetectionPosition) {
        this.teamColor = teamColor;
        this.skystoneDetectionPosition = skystoneDetectionPosition;
        customizeWaypoints(teamColor, skystoneDetectionPosition);
    }

    public Waypoints(Color.Ftc teamColor) {
        // Default skystone position to 0.
        this(teamColor, 0);
    }

    /**
     * @param skystoneDetectionPosition
     * Sets the skystone position and recalculates the waypoint positions by calling customizeWaypoints()
     */
    public void setSkystoneDetectionPosition(int skystoneDetectionPosition) {
        this.skystoneDetectionPosition = skystoneDetectionPosition;
        customizeWaypoints(teamColor, skystoneDetectionPosition);
    }


    /**
     * blueLoading waypoints are first defined to be used as a template.
     * Then a reflection on the X axis is used to define generic waypoints
     * team color.
     *
     * These initial waypoints for blueLoading are generated from
     * a list of parameters to ease tweaking.
     *
     * Constructor uses skystoneDetectionPosition as an input, numbered 0-5
     * from field center to front wall.
     */

    /**
     * public waypoints customized for starting location
      */

    public enum LocationLoading {
        initialPosition,
        scanPosition_A,
        grabSkystone_A,
        backupPosition_A,
        buildZone_A,
        scanPosition_B,
        grabSkystone_B,
        backupPosition_B,
        buildZone_B,
        parkOuter,
        parkInner,
        simpleAlignment_Inner,
    }

    public enum LocationBuild {
        initialPosition,
        parkOuter,
        parkInner,
        simpleAlignment_Inner,
    }

    // Waypoint locations.
    public Map<LocationLoading,Navigation2D> genericLoading = new EnumMap<>(LocationLoading.class);
    public Map<LocationBuild,Navigation2D> genericBuild = new EnumMap<>(LocationBuild.class);

    //Blue is used as the template.
    Map<LocationLoading,Navigation2D> blueLoading = new EnumMap<>(LocationLoading.class);
    Map<LocationBuild,Navigation2D> blueBuild = new EnumMap<>(LocationBuild.class);

    // Skystone locations (6 stones from field center to wall numbered from 0-5)
    // Waypoint customized by color.
    public List<Navigation2D> stoneLocations = new ArrayList();
    List<Navigation2D> blueStoneLocations = new ArrayList();



    /**
     * Template parameter constants
     */
    double cameraOffset = -90; // extra rotation needed to point the camera in given direction

    // Game and field elements
    double stoneWidth = 4;
    double stoneLength = 8;
    double stoneHeight = 4;
    double stoneKnobHeight = 5;
    double tileBody = 22.75; // Width of a tile without its tabs
    double tileTabs = 0.9; // Width of interlocking tabs, from tile body to tile body.
    double halfField = 70.5; // Should equal (6*tileBody + 5*tileTabs)/2
    double stoneStartYOffset = (tileBody * 2) + (tileTabs * 2) + stoneWidth/2; // Distance from outside wall to stone center.

    // Robot Dimensions
    double robotWidth = 17;
    double robotSidePadding = robotWidth/2;
    double robotFrontPadding = 5.25; //Distance from Robot front edge to wheelbase center.
    double robotBackPadding = 6.25; //Distance from Robot back edge to wheelbase center.
    double grabOffset_X_Forward = 9; // Forward on Robot from navigation point, center of drivetrain.
    double grabOffset_Y_Left = 0; // Left on Robot


    double loadingStart_X = 0;
    double loadingStart_Y = 0;

    // Maneuver
    double scanOffset_Y = 10;
    double backupDistance = 6;
    double buildZoneOffset = 5;

    // Skystone index is numbered from 0 to 5, startin from field center.
    public double skystoneXFromIndex(int index) {
        return -halfField + stoneLength * (5.5 - index);
    }

    public double skystoneIndexFromX(double skystoneX) {
        return skystoneX = 5.5 -(skystoneX + halfField) / stoneLength;
    }



    /**
     * Blue Loading positions set and used as templates
     */
    private void create_blue_waypoints() {
        /**
         * Blue Quarry Stone Positions
         */
        for(int i = 0; i<=5; ++i) {
            blueStoneLocations.add(i,new Navigation2D(skystoneXFromIndex(i),halfField-2*tileBody-2*tileTabs-0.5*stoneWidth,0));
        }

        /**
         * Blue Loading Waypoints
         */
        blueLoading.put(LocationLoading.initialPosition,new Navigation2D(-tileBody - robotSidePadding, halfField - robotBackPadding, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.scanPosition_A, new Navigation2D(-tileBody - robotSidePadding, halfField - robotBackPadding - scanOffset_Y, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.grabSkystone_A, new Navigation2D(-halfField + stoneLength * (5.5 - skystoneDetectionPosition), halfField - stoneStartYOffset - stoneWidth * 0.5 + grabOffset_X_Forward, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.backupPosition_A, new Navigation2D(-halfField + stoneLength * (5.5 - skystoneDetectionPosition), halfField - stoneStartYOffset - stoneWidth * 0.5 + grabOffset_X_Forward + backupDistance, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.buildZone_A, new Navigation2D(tileBody, tileBody - buildZoneOffset, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.scanPosition_B, new Navigation2D(0, 0, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.grabSkystone_B, new Navigation2D(0, 0, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.backupPosition_B, new Navigation2D(0, 0, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.buildZone_B, new Navigation2D(0, 0, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.parkOuter, new Navigation2D(0, halfField - robotBackPadding, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.parkInner, new Navigation2D(0, halfField - 1.5 * tileBody + robotFrontPadding, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.simpleAlignment_Inner, new Navigation2D(-tileBody - robotSidePadding, halfField - 1.5 * tileBody + robotFrontPadding, degreesToRadians(-90)));

        /**
         * Blue Build positions
         */
        blueBuild.put(LocationBuild.initialPosition, new Navigation2D(+tileBody + robotSidePadding, halfField - robotBackPadding, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.parkOuter, new Navigation2D(0, halfField - robotBackPadding, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.parkInner, new Navigation2D(0, halfField - 1.5 * tileBody + robotFrontPadding, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.simpleAlignment_Inner, new Navigation2D(+tileBody + robotSidePadding, halfField - 1.5 * tileBody + robotFrontPadding, degreesToRadians(-90)));
    }


    void customizeWaypoints(Color.Ftc teamColor, int skystoneDetectionPosition) {
        if(teamColor == Color.Ftc.BLUE) {
            activate_blue_waypoints();
        } else if (teamColor == Color.Ftc.RED) {
            activate_red_waypoints();
        } else {
            throw new IllegalStateException("Invalid Team Color");
        }
    }


    void activate_blue_waypoints() {
        genericLoading.putAll(blueLoading);
        genericBuild.putAll(blueBuild);
        stoneLocations.addAll(blueStoneLocations);
    }

    void activate_red_waypoints() {
        genericLoading.putAll(blueLoading);
        genericBuild.putAll(blueBuild);
        stoneLocations.addAll(blueStoneLocations);
        x_reflectGenericWaypointsAndStoneLocationsInPlace();
    }


    void x_reflectGenericWaypointsAndStoneLocationsInPlace() {
        // reflect the generic waypoints around x axis, storing
        // back into the generic waypoints.
        for(Navigation2D waypoint: genericLoading.values()) {
            waypoint.reflectInX(); // Reflects in place, by reference.
        }

        // for Building
        for(Navigation2D waypoint: genericBuild.values()) {
            waypoint.reflectInX(); // Reflects in place, by reference.
        }

        // for stone locations
        for(Navigation2D waypoint: stoneLocations) {
            waypoint.reflectInX();
        }
    }

    public List<LabeledWaypoint> getLabeledWaypointListForLoad() {
        ArrayList<LabeledWaypoint> labeledWaypoints = new ArrayList<>();
        for(LocationLoading locationLoading: LocationLoading.values()) {
            labeledWaypoints.add(new LabeledWaypoint(locationLoading.toString(), genericLoading.get(locationLoading)));
        }
        return labeledWaypoints;
    }

    public List<LabeledWaypoint> getLabeledWaypointListForBuild() {
        ArrayList<LabeledWaypoint> labeledWaypoints = new ArrayList<>();
        for(LocationBuild locationBuild: LocationBuild.values()) {
            labeledWaypoints.add(new LabeledWaypoint(locationBuild.toString(), genericLoading.get(locationBuild)));
        }
        return labeledWaypoints;
    }

    public static class LabeledWaypoint {
        public String label = "";
        public Navigation2D waypoint_n2d = new Navigation2D(0,0,0);

        LabeledWaypoint(String label, Navigation2D waypoint_n2d) {
            this.label = label;
            this.waypoint_n2d = waypoint_n2d;
        }
    }

    // Utility
    double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

}