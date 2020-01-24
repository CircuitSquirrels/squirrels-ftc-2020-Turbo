package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.EnumMap;


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
        skystoneDetectionPosition = skystoneDetectionPosition < 0 ? 0 : skystoneDetectionPosition;
        skystoneDetectionPosition = skystoneDetectionPosition > 2 ? 2 : skystoneDetectionPosition;
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
        INITIAL_POSITION,
        GRAB_SKYSTONE_A,
        ALIGNMENT_POSITION_A,
        BUILD_ZONE,
        GRAB_SKYSTONE_B,
        ALIGNMENT_POSITION_B,
        GRAB_EXTRA_STONE_A,
        ALIGN_EXTRA_STONE_A,
        GRAB_EXTRA_STONE_B,
        ALIGN_EXTRA_STONE_B,
        FOUNDATION_ALIGNMENT,
        FOUNDATION_DROP_OFF,
        DRAG_FOUNDATION_INSIDE_WALL,
        DRAG_FOUNDATION_OUTSIDE_WALL,
        STRAFE_AWAY_FROM_FOUNDATION,
        BRIDGE_ALIGNMENT_OUTER,
        BRIDGE_ALIGNMENT_INNER,
        PARK_OUTER,
        PARK_INNER,
        SIMPLE_ALIGNMENT_INNER,
    }

    public enum LocationBuild {
        INITIAL_POSITION,
        ALIGN_FOUNDATION,
        FOUNDATION,
        DRAG_FOUNDATION_INSIDE_WALL,
        DRAG_FOUNDATION_OUTSIDE_WALL,
        STRAFE_AWAY_FROM_FOUNDATION,
        BRIDGE_ALIGNMENT_OUTER,
        BRIDGE_ALIGNMENT_INNER,
        PARK_OUTER,
        PARK_INNER,
        SIMPLE_ALIGNMENT_INNER,
    }

    // Waypoint locations.
    public Map<LocationLoading,Navigation2D> loading = new EnumMap<>(LocationLoading.class);
    public Map<LocationBuild,Navigation2D> building = new EnumMap<>(LocationBuild.class);

    //Blue is used as the template.
    private Map<LocationLoading,Navigation2D> blueLoading = new EnumMap<>(LocationLoading.class);
    private Map<LocationBuild,Navigation2D> blueBuild = new EnumMap<>(LocationBuild.class);

    // Skystone locations (6 stones from field center to wall numbered from 0-5)
    // Waypoint customized by color.
    public List<Navigation2D> stoneLocations = new ArrayList();
    private List<Navigation2D> blueStoneLocations = new ArrayList();



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
        List<Navigation2D> blueStonePickupLocations = new ArrayList(); // Location from which to pickup stone 0-5
        List<Navigation2D> blueStoneAlignmentLocations = new ArrayList(); // Location from which to align, scan, or drive under bridge from stone 0-5
        for(int i = 0; i<=5; ++i) {
            blueStoneLocations.add(i,new Navigation2D(skystoneXFromIndex(i),halfField-2*tileBody-2*tileTabs-0.5*stoneWidth,degreesToRadians(-90)));
            blueStonePickupLocations.add(i, blueStoneLocations.get(i).copy().addAndReturn(0,grabOffset_X_Forward,0)); // setup grabOffset from stones.
            blueStoneAlignmentLocations.add(i, new Navigation2D(skystoneXFromIndex(i), innerTileAlignment_Y + 2, degreesToRadians(-90)));
        }

        /**
         * Blue Loading Waypoints
         */
        blueLoading.put(LocationLoading.INITIAL_POSITION,new Navigation2D(-tileBody - robotSidePadding, halfField - robotBackPadding + 1, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.GRAB_SKYSTONE_A, blueStonePickupLocations.get(skystoneDetectionPosition).copy());
        blueLoading.put(LocationLoading.ALIGNMENT_POSITION_A, blueStoneAlignmentLocations.get(skystoneDetectionPosition).copy());
        blueLoading.put(LocationLoading.BUILD_ZONE, new Navigation2D(tileBody - 6, blueStoneAlignmentLocations.get(skystoneDetectionPosition).y, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.GRAB_SKYSTONE_B, blueStonePickupLocations.get(skystoneDetectionPosition + 3).copy());
        blueLoading.put(LocationLoading.ALIGNMENT_POSITION_B, blueStoneAlignmentLocations.get(skystoneDetectionPosition + 3).copy());
        blueLoading.put(LocationLoading.GRAB_EXTRA_STONE_A,  blueStonePickupLocations.get(skystoneDetectionPosition == 0 ? 1 : 0).copy());
        blueLoading.put(LocationLoading.ALIGN_EXTRA_STONE_A,  blueStoneAlignmentLocations.get(skystoneDetectionPosition == 0 ? 1 : 0).copy());
        blueLoading.put(LocationLoading.GRAB_EXTRA_STONE_B,  blueStonePickupLocations.get(skystoneDetectionPosition == 0 ? 4 : 3).copy());
        blueLoading.put(LocationLoading.ALIGN_EXTRA_STONE_B,  blueStoneAlignmentLocations.get(skystoneDetectionPosition == 0 ? 4 : 3).copy());
        blueLoading.put(LocationLoading.FOUNDATION_ALIGNMENT, new Navigation2D(49, blueStoneAlignmentLocations.get(0).y, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.FOUNDATION_DROP_OFF, new Navigation2D(50.125, 26.74 + 4 + 2, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.BRIDGE_ALIGNMENT_OUTER, new Navigation2D(0, halfField - tileTabs - robotSidePadding, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.BRIDGE_ALIGNMENT_INNER, new Navigation2D(0, innerTileAlignment_Y, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.DRAG_FOUNDATION_INSIDE_WALL, new Navigation2D(blueLoading.get(LocationLoading.FOUNDATION_ALIGNMENT).x, blueLoading.get(LocationLoading.INITIAL_POSITION).y, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.DRAG_FOUNDATION_OUTSIDE_WALL, new Navigation2D(blueLoading.get(LocationLoading.FOUNDATION_ALIGNMENT).x, blueLoading.get(LocationLoading.INITIAL_POSITION).y + 7, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.STRAFE_AWAY_FROM_FOUNDATION, new Navigation2D(24, halfField - robotBackPadding, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.PARK_OUTER, new Navigation2D(0, halfField - robotBackPadding, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.PARK_INNER, new Navigation2D(0, halfField - 1.8 * tileBody + robotFrontPadding, degreesToRadians(-90)));
        blueLoading.put(LocationLoading.SIMPLE_ALIGNMENT_INNER, new Navigation2D(-tileBody - robotSidePadding, halfField - 1.2 * tileBody + robotFrontPadding, degreesToRadians(-90)));

        /**
         * Blue Build positions
         */
        blueBuild.put(LocationBuild.INITIAL_POSITION, new Navigation2D(+tileBody + robotSidePadding, halfField - robotBackPadding, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.ALIGN_FOUNDATION, new Navigation2D(49, blueStoneAlignmentLocations.get(0).y, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.FOUNDATION, new Navigation2D(50.125, 26.74 + 4 + 2, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.BRIDGE_ALIGNMENT_OUTER, new Navigation2D(0, halfField - tileTabs - robotSidePadding, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.BRIDGE_ALIGNMENT_INNER, new Navigation2D(0, innerTileAlignment_Y, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.DRAG_FOUNDATION_INSIDE_WALL, new Navigation2D(blueLoading.get(LocationLoading.FOUNDATION_ALIGNMENT).x, blueLoading.get(LocationLoading.INITIAL_POSITION).y, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.DRAG_FOUNDATION_OUTSIDE_WALL, new Navigation2D(blueLoading.get(LocationLoading.FOUNDATION_ALIGNMENT).x, blueLoading.get(LocationLoading.INITIAL_POSITION).y + 7, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.STRAFE_AWAY_FROM_FOUNDATION, new Navigation2D(24, halfField - robotBackPadding, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.PARK_OUTER, new Navigation2D(0, halfField - robotBackPadding, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.PARK_INNER, new Navigation2D(0, halfField - 1.5 * tileBody + robotFrontPadding, degreesToRadians(-90)));
        blueBuild.put(LocationBuild.SIMPLE_ALIGNMENT_INNER, new Navigation2D(+tileBody + robotSidePadding, halfField - 1.5 * tileBody + robotFrontPadding, degreesToRadians(-90)));
    }

    // skystoneDetectionPosition is 0-5, from field center to the front wall.
    private void customizeWaypoints(Color.Ftc teamColor, int skystoneDetectionPosition) {
        this.teamColor = teamColor;
        this.skystoneDetectionPosition = skystoneDetectionPosition; // ensure class property is set.
        create_blue_waypoints(); // gets skystoneDetectionPosition from class property.
        if(teamColor == Color.Ftc.BLUE) {
            activate_blue_waypoints();
        } else if (teamColor == Color.Ftc.RED) {
            activate_red_waypoints();
        } else {
            throw new IllegalStateException("Invalid Team Color");
        }
        labelAllWaypointsFromEnums();
    }


    void activate_blue_waypoints() {
        loading.putAll(blueLoading);
        building.putAll(blueBuild);
        stoneLocations.addAll(blueStoneLocations);
    }

    void activate_red_waypoints() {
        loading.putAll(blueLoading);
        building.putAll(blueBuild);
        stoneLocations.addAll(blueStoneLocations);
        x_reflectGenericWaypointsAndStoneLocationsInPlace();
        // I apologize in advance for the stupidity of our code
        loading.get(LocationLoading.FOUNDATION_DROP_OFF).addInPlace(redDropOffFudgeFactor);
        loading.get(LocationLoading.FOUNDATION_ALIGNMENT).addInPlace(redDropOffFudgeFactor);
        loading.get(LocationLoading.BUILD_ZONE).addInPlace(redDropOffFudgeFactor);
    }


    void x_reflectGenericWaypointsAndStoneLocationsInPlace() {
        // reflect the generic waypoints around x axis, storing
        // back into the generic waypoints.
        for(Navigation2D waypoint: loading.values()) {
            waypoint.reflectInX(); // Reflects in place, by reference.
        }

        // for Building
        for(Navigation2D waypoint: building.values()) {
            waypoint.reflectInX(); // Reflects in place, by reference.
        }

        // for stone locations
        for(Navigation2D waypoint: stoneLocations) {
            waypoint.reflectInX();
        }
    }

    public void labelAllWaypointsFromEnums() {
        for(LocationLoading locationLoading: LocationLoading.values()) {
            loading.get(locationLoading).setLabel(locationLoading.toString());
        }

        for(LocationBuild locationBuild: LocationBuild.values()) {
            building.get(locationBuild).setLabel(locationBuild.toString());
        }

        for(int iStone=0; iStone<=5; ++iStone) {
            stoneLocations.get(iStone).setLabel("Stone_" + String.valueOf(iStone));
        }
    }

    public List<Navigation2D> getLabeledWaypointListForLoad() {
        ArrayList<Navigation2D> labeledWaypoints = new ArrayList<>();
        for(LocationLoading locationLoading: LocationLoading.values()) {
            labeledWaypoints.add(loading.get(locationLoading).copyAndLabel(locationLoading.toString()));
        }
        return labeledWaypoints;
    }

    public List<Navigation2D> getLabeledWaypointListForBuild() {
        ArrayList<Navigation2D> labeledWaypoints = new ArrayList<>();
        for(LocationBuild locationBuild: LocationBuild.values()) {
            labeledWaypoints.add(building.get(locationBuild).copyAndLabel(locationBuild.toString()));
        }
        return labeledWaypoints;
    }

    // Utility
    double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

}