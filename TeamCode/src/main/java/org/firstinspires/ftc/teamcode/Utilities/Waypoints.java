package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

public class Waypoints {

    // Start position specific parameters
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    double genericRotate = 0;

    /**
     * blueCrater waypoints, as well as partner_blueDepot waypoints (for partner mineral field) are
     * first defined to be used as a template.  Then a series of rotations are used to define
     * generic waypoints depending on the start position and team color.
     *
     * These initial waypoints for blueCrater and the partner_blueDepot are generated from
     * a list of parameters to ease tweaking.
     *
     * We are allowing the 'left' and 'right' mineral to refer to the robot's perspective,
     * regardless of which side of the mineral field it is on.  This isn't consistent with a
     * global naming convention, but it doesn't matter as long as we knock the gold mineral
     * wherever we find it.
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

    double skystoneWidth = 0;
    double skystoneLength = 0;
    double tileWidth = 0;

    double loadingStart_X = 0;
    double loadingStart_Y = 0;
    double grabOffset_X = 0;
    double grabOffset_Y = 0;

    /**
     * Blue crater positions set and used as templates
     */
//    Navigation2D blueCrater_initialPosition = new Navigation2D(blueCrater_start_x,blueCrater_start_y,degreesToRadians(blueCrater_start_degrees + blueCrater_hanging_biased));

    public Waypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.teamColor = teamColor;
        this.startPosition = startPosition;
    }

    void customizeWaypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition, boolean doPartnerMineralField) {
        if(teamColor == Color.Ftc.BLUE) {
            if(startPosition == RobotHardware.StartPosition.FIELD_PICKUP) {
                create_blue_crater_waypoints();

            } else if (startPosition == RobotHardware.StartPosition.FIELD_BUILD) {
                //Blue Depot
                create_blue_depot_waypoints();

            } else {
                throw new IllegalStateException("Invalid Starting Position");
            }
        } else if (teamColor == Color.Ftc.RED) {
            if(startPosition == RobotHardware.StartPosition.FIELD_PICKUP) {
                //Red Crater
                create_blue_crater_waypoints();
                rotate_waypoints_in_place(180);

            } else if (startPosition == RobotHardware.StartPosition.FIELD_BUILD) {
                //Red Depot
                create_blue_depot_waypoints();
                rotate_waypoints_in_place(180);

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

    void create_blue_crater_waypoints() {
        //Blue Crater
//        initialPosition = blueCrater_initialPosition.copy();


    }


    void rotate_waypoints_in_place(double rotateDegrees) {
        // rotate the generic waypoints around (0,0), storing
        // back into the generic waypoints.

        initialPosition.rotate(rotateDegrees);

    }


    void create_blue_depot_waypoints() {
        create_blue_crater_waypoints();
        // generally 90 degree rotation from blueCrater points
        genericRotate = 90;
        rotate_waypoints_in_place(genericRotate);


        // This angle doesn't give a view of the vuforia target, which could be
        // problematic later if that is used. In that case, this photoRotate definition can be
        // commented out, and the state machine can be modified to skip this waypoint when
        // a view of the vuforia target is not needed.
//        photoRotate = new Navigation2D(-wallOffsetPosition, 0, degreesToRadians(-90));
//        flagDrop = new Navigation2D(-wallOffsetPosition, flagDropDepth, degreesToRadians(-90));
//        depotPush = new Navigation2D(-wallOffsetPosition,depotDepth,degreesToRadians(-90));
//        craterPark = new Navigation2D(-craterPark_wall_offset, -craterPark_depth, degreesToRadians(-90));
//
//
//        // Optional team mineral scan
//        // For blueDepot, using same points as partner standard scan and knock.
//        partner_scanMineral_center = blueCrater_scanMineral_center.copy();
//        partner_scanMineral_left = blueCrater_scanMineral_left.copy();
//        partner_scanMineral_right = blueCrater_scanMineral_right.copy();
//
//        partner_alignMineral_center = blueCrater_alignMineral_center.copy();
//        partner_alignMineral_left = blueCrater_alignMineral_left.copy();
//        partner_alignMineral_right = blueCrater_alignMineral_right.copy();
//
//        partner_knockMineral_center = blueCrater_knockMineral_center.copy();
//        partner_knockMineral_left = blueCrater_knockMineral_left.copy();
//        partner_knockMineral_right = blueCrater_knockMineral_right.copy();
    }
}