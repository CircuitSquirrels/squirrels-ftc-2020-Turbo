package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.EnumMap;

import static org.firstinspires.ftc.teamcode.Utilities.Color.Ftc.BLUE;

/**
 * @author Christian, Ashley
 *
 * Class used to manage all autonomous based waypoints.
 * Allows for easy tweaking, better readability.
 * Depending on alliance color, waypoints will automatically be mirrored from blue.
 * The alliance color can be changed via the constructor.
 * The constructor also requres the detected skystone position [0-2]
 * @see org.firstinspires.ftc.teamcode.Vision.AveragingPipeline
 */
public class Waypoints {

    private Color.Ftc teamColor;
    private int skystoneDetectionPosition;

    public Waypoints(Color.Ftc teamColor, int skystoneDetectionPosition) {
        this.teamColor = teamColor;
        this.skystoneDetectionPosition = skystoneDetectionPosition;
        customizeWaypoints(teamColor, skystoneDetectionPosition);
    }

    public Waypoints(Color.Ftc teamColor) {
        this(teamColor, 0);
    }

    public Waypoints() {
        this(BLUE, 0);
    }

    /**
     * Loading side waypoints.
     * All values are based off the blue side.
     */
    public enum LocationLoading {
        INITIAL_POSITION(-33, 65, -90),

        ALIGNMENT_POSITION_A(0,0,0),
        GRAB_SKYSTONE_A(0,0,0),

        ALIGNMENT_POSITION_B(0,0,0),
        GRAB_SKYSTONE_B(0,0,0),
        // Set extra stone A to the 1st stone. If the stone is already taken it'll change it to the 2nd stone.
        ALIGN_EXTRA_STONE_A(-26.5,40.5,-90),
        GRAB_EXTRA_STONE_A(-26.5,31.5,-90),
        // Set extra stone B to the 2nd stone. If the stone is already taken it'll change it to the 3rd stone.
        ALIGN_EXTRA_STONE_B(-34.5,40.5,-90),
        GRAB_EXTRA_STONE_B(-34.5,31.5,-90),

        BUILD_ZONE(24.5, 37.5, -90),

        FOUNDATION_ALIGNMENT(50.5, 40.75, -90),
        FOUNDATION_PLACE(50.5, 30.75, -90),

        PULL_FOUNDATION(27, 40.5, 0),
        PUSH_FOUNDATION(45, 40.5, 0),

        PARK_OUTER(0, 60.5, -90),
        PARK_INNER(0, 40.5, -90),

        SIMPLE_ALIGNMENT_INNER(-33, 40.5, -90);

        private double x;
        private double y;
        private double theta;
        private Navigation2D nav2D;

        /**
         * @param x Waypoint X coordinate
         * @param y Waypoint Y coordinate
         * @param theta Waypoint Theta, Degrees CCW
         */
        LocationLoading(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = Math.toRadians(theta);
            this.nav2D = getNewNavigation2D();
        }

        public void setX(double x) {
            this.x = x;
            this.nav2D = getNewNavigation2D();
        }

        public void setY(double y) {
            this.y = y;
            this.nav2D = getNewNavigation2D();
        }

        public void setTheta(double theta) {
            this.theta = theta;
            this.nav2D = getNewNavigation2D();
        }

        public Navigation2D getNav2D() {
            return nav2D;
        }

        public Navigation2D getNewNavigation2D(){
            return new Navigation2D(x, y, theta);
        }

        public void set(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.nav2D = getNewNavigation2D();
        }

        public void set(Navigation2D navigation2D) {
            this.x = navigation2D.x;
            this.y = navigation2D.y;
            this.theta = navigation2D.theta;
            this.nav2D = getNewNavigation2D();
        }
    }

    public enum LocationBuild {
        INITIAL_POSITION(0,0,0),
        PARK_OUTER(0,0,0),
        PARK_INNER(0,0,0),
        SIMPLE_ALIGNMENT_INNER(0,0,0);

        private double x;
        private double y;
        private double theta;
        private Navigation2D nav2D;

        /**
         * @param x Waypoint X coordinate
         * @param y Waypoint Y coordinate
         * @param theta Waypoint Theta, Degrees CCW
         */
        LocationBuild(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = Math.toRadians(theta);
            this.nav2D = getNewNavigation2D();
        }

        public void setX(double x) {
            this.x = x;
            this.nav2D = getNewNavigation2D();
        }

        public void setY(double y) {
            this.y = y;
            this.nav2D = getNewNavigation2D();
        }

        public void setTheta(double theta) {
            this.theta = theta;
            this.nav2D = getNewNavigation2D();
        }

        public Navigation2D getNav2D() {
            return nav2D;
        }

        public Navigation2D getNewNavigation2D(){
            return new Navigation2D(x, y, theta);
        }

        public void set(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.nav2D = getNewNavigation2D();
        }

        public void set(Navigation2D navigation2D) {
            this.x = navigation2D.x;
            this.y = navigation2D.y;
            this.theta = navigation2D.theta;
            this.nav2D = getNewNavigation2D();
        }
    }

    public List<Navigation2D> stoneLocations = new ArrayList<>();
    private List<Navigation2D> blueStoneLocations = new ArrayList<>();

    /**
     * Template parameter constants
     */

    // Game and field elements
    private double stoneWidth = 4;
    private double stoneLength = 8;
    private double stoneHeight = 4;
    private double stoneKnobHeight = 5;
    private double tileBody = 22.75; // Width of a tile without its tabs
    private double tileTabs = 0.9; // Width of interlocking tabs, from tile body to tile body.
    private double halfField = 70.875; // Should equal (6*tileBody + 5*tileTabs)/2
    private double stoneStartYOffset = (tileBody * 2) + (tileTabs * 2) + stoneWidth / 2; // Distance from outside wall to stone center.

    // Robot Dimensions
    private double robotWidth = 17;
    private double robotSidePadding = robotWidth/2;
    private double robotFrontPadding = 5.25; //Distance from Robot front edge to wheelbase center.
    private double robotBackPadding = 6.25; //Distance from Robot back edge to wheelbase center.
    private double grabOffset_X_Forward = 9.75; // Forward on Robot from navigation point, center of drivetrain.  // 9.75 when measured
    private double grabOffset_Y_Left = 0; // Left on Robot
    private double wallPadding = 1.5; // How far to stay from something you don't want to bump.
    public Navigation2D redDropOffFudgeFactor = new Navigation2D(0, 3, 0);

    // Maneuver
    private double scanOffset_Y = 10;  // Arbitrary distance to move forward before scanning for skystones.
    private double backupDistance = 6; // Distance to pull away from stones before going under bridge
    private double buildZoneOffset = 5; // Arbitrary distance to move forward on the build side before dropping off skystones.
    private double innerTileAlignment_Y = halfField - tileBody - tileTabs - robotSidePadding;


    // Skystone index is numbered from 0 to 5, starting from field center.
    public double skystoneXFromIndex(int index) {
        return -70 + stoneLength * (5.5 - index);
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
        List<Navigation2D> blueStonePickupLocations = new ArrayList<>(); // Location from which to pickup stone 0-5
        List<Navigation2D> blueStoneAlignmentLocations = new ArrayList<>(); // Location from which to align, scan, or drive under bridge from stone 0-5
        for(int i = 0; i<=5; ++i) {
            blueStoneLocations.add(i,new Navigation2D(skystoneXFromIndex(i),31.5, Math.toRadians(-90)));
            // Add robot's length for correct grab position.
            blueStonePickupLocations.add(i, blueStoneLocations.get(i).copy().addAndReturn(.5,0,0));
            blueStoneAlignmentLocations.add(i, new Navigation2D(skystoneXFromIndex(i), 40.5, Math.toRadians(-90)));
        }

        /**
         * Blue Loading Waypoints
         */
        LocationLoading.ALIGNMENT_POSITION_A.set(blueStoneAlignmentLocations.get(skystoneDetectionPosition).copy());
        LocationLoading.GRAB_SKYSTONE_A.set(blueStonePickupLocations.get(skystoneDetectionPosition).copy());
        LocationLoading.ALIGNMENT_POSITION_B.set(blueStoneAlignmentLocations.get(skystoneDetectionPosition + 2).copy());
        LocationLoading.GRAB_SKYSTONE_B.set(blueStonePickupLocations.get(skystoneDetectionPosition + 2).copy());
        switch (getSkystoneDetectionPosition()) {
            case 0:
                LocationLoading.ALIGN_EXTRA_STONE_A.set(blueStoneAlignmentLocations.get(1).copy());
                LocationLoading.GRAB_EXTRA_STONE_A.set(blueStoneAlignmentLocations.get(1).copy());
                LocationLoading.ALIGN_EXTRA_STONE_B.set(blueStoneAlignmentLocations.get(2).copy());
                LocationLoading.GRAB_EXTRA_STONE_B.set(blueStoneAlignmentLocations.get(2).copy());
                break;
            case 2:
                LocationLoading.ALIGN_EXTRA_STONE_B.set(blueStoneAlignmentLocations.get(1).copy());
                LocationLoading.GRAB_EXTRA_STONE_B.set(blueStoneAlignmentLocations.get(1).copy());
                break;
        }

        /**
         * Blue Build positions
         */
        //Todo get Build waypoint positions
    }

    /**
     * Creates the waypoints
     * @param teamColor Alliance team color.
     * @see Color
     * @param skystoneDetectionPosition The detected skystone's position [0-2]
     * @see org.firstinspires.ftc.teamcode.Vision.AveragingPipeline
     */
    private void customizeWaypoints(Color.Ftc teamColor, int skystoneDetectionPosition) {
        create_blue_waypoints();
        switch (teamColor) {
            case BLUE:
                activate_blue_waypoints();
                break;
            case RED:
                activate_red_waypoints();
                break;
            default:
                throw new IllegalStateException("Invalid Team Color");
        }

        labelAllWaypointsFromEnums();
    }


    void activate_blue_waypoints() {
        stoneLocations.addAll(blueStoneLocations);
    }

    void activate_red_waypoints() {
        stoneLocations.addAll(blueStoneLocations);
        x_reflectGenericWaypointsAndStoneLocationsInPlace();

        LocationLoading.FOUNDATION_PLACE.nav2D.addInPlace(redDropOffFudgeFactor);
        LocationLoading.FOUNDATION_ALIGNMENT.nav2D.addInPlace(redDropOffFudgeFactor);
        LocationLoading.BUILD_ZONE.nav2D.addInPlace(redDropOffFudgeFactor);
    }


    void x_reflectGenericWaypointsAndStoneLocationsInPlace() {
        // reflect the generic waypoints around x axis, storing
        // back into the generic waypoints.
        for(LocationLoading waypoint : LocationLoading.values()) {
            waypoint.nav2D.reflectInX(); // Reflects in place, by reference.
        }

        // for Building
        for(LocationBuild waypoint: LocationBuild.values()) {
            waypoint.nav2D.reflectInX(); // Reflects in place, by reference.
        }

        // for stone locations
        for(Navigation2D waypoint: stoneLocations) {
            waypoint.reflectInX();
        }
    }

    public void labelAllWaypointsFromEnums() {
        for(LocationLoading locationLoading : LocationLoading.values()) {
            locationLoading.nav2D.setLabel(locationLoading.name());
        }

        for(LocationBuild locationBuild : LocationBuild.values()) {
            locationBuild.nav2D.setLabel(locationBuild.name());
        }

        for(int iStone=0; iStone<=5; ++iStone) {
            stoneLocations.get(iStone).setLabel("Stone_" + iStone);
        }
    }

    public List<Navigation2D> getLabeledWaypointListForLoad() {
        ArrayList<Navigation2D> labeledWaypoints = new ArrayList<>();
        for(LocationLoading locationLoading: LocationLoading.values()) {
            labeledWaypoints.add(locationLoading.nav2D.copyAndLabel(locationLoading.toString()));
        }
        return labeledWaypoints;
    }

    public List<Navigation2D> getLabeledWaypointListForBuild() {
        ArrayList<Navigation2D> labeledWaypoints = new ArrayList<>();
        for(LocationBuild locationBuild: LocationBuild.values()) {
            labeledWaypoints.add(locationBuild.nav2D.copyAndLabel(locationBuild.toString()));
        }
        return labeledWaypoints;
    }


    /**
     * @param skystoneDetectionPosition
     * Sets the skystone position and recalculates the waypoint positions by calling customizeWaypoints()
     */
    public void setSkystoneDetectionPosition(int skystoneDetectionPosition) {
        skystoneDetectionPosition = skystoneDetectionPosition < 0 ? 0 : skystoneDetectionPosition;
        skystoneDetectionPosition = skystoneDetectionPosition > 2 ? 2 : skystoneDetectionPosition;
        this.skystoneDetectionPosition = skystoneDetectionPosition;
        customizeWaypoints(teamColor, skystoneDetectionPosition);
    }

    public int getSkystoneDetectionPosition() {
        return skystoneDetectionPosition;
    }

    public Color.Ftc getTeamColor() {
        return teamColor;
    }
}