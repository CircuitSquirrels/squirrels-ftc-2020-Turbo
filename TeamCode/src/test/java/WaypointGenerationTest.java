import static com.google.common.truth.Truth.assertThat;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints.LabeledWaypoint;
import org.junit.Test;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import static org.firstinspires.ftc.teamcode.RobotHardware.StartPosition.FIELD_LOADING;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.*;

public class WaypointGenerationTest {


    private void assertWaypointNotNull(Navigation2D navigation2D) {
        assertThat(navigation2D).isNotNull();
    }

    private void assertWaypoint_xEqual(Navigation2D first_n2d, Navigation2D second_n2d) {
        assertThat(first_n2d.x).isEqualTo(second_n2d.x);
    }

    private void assertWaypoint_yEqual(Navigation2D first_n2d, Navigation2D second_n2d) {
        assertThat(first_n2d.y).isEqualTo(second_n2d.y);
    }

    private void assertWaypoint_thetaEqual(Navigation2D first_n2d, Navigation2D second_n2d) {
        // Convert theta radians to degrees to make debugging easier.
        assertThat(Math.toDegrees(first_n2d.theta)).isEqualTo(Math.toDegrees(second_n2d.theta));
    }

    @Test
    public void WaypointsDefinedForAllCases() {
        // Check RED and BLUE, BUILD AND LOAD, skystone 0,1,2, false skystone 3,4,5,6, -1 for errors.
    }

    @Test
    public void Waypoint_startPoint_not_null() {
        System.out.println("Start Test");
        Waypoints waypoints = new Waypoints(Color.Ftc.BLUE);
        waypoints.setSkystoneDetectionPosition(1);

        Navigation2D alignmentA_n2d = waypoints.loading.get(ALIGNMENT_POSITION_A);
        Navigation2D pickupA_n2d = waypoints.loading.get(GRAB_SKYSTONE_A);

        assertWaypointNotNull(alignmentA_n2d);
        assertWaypoint_thetaEqual(alignmentA_n2d,pickupA_n2d);
        assertWaypoint_xEqual(alignmentA_n2d,pickupA_n2d);

    }


    @Test
    public void GenerateAllWaypoits() throws IOException {
        for(RobotHardware.StartPosition startPosition: RobotHardware.StartPosition.values()) {
            Display_Waypoints(Color.Ftc.BLUE, startPosition, 0);
        }
    }


    private void Display_Waypoints(Color.Ftc color, RobotHardware.StartPosition startPosition, int skystoneIndex_0to5) throws IOException {

        System.out.println("\n"+"*** Start Waypoint Display **** ");
        Waypoints waypoints = new Waypoints(color,skystoneIndex_0to5);
        waypoints.setSkystoneDetectionPosition(1);
        System.out.println("Team Color:                     " + waypoints.getTeamColor());
        System.out.println("Start Position:                 " + startPosition);
        System.out.println("Skystone Detection Position:    " + waypoints.getSkystoneDetectionPosition());
        System.out.println();

        List<LabeledWaypoint> waypointList;
        if(startPosition == FIELD_LOADING) {
            waypointList = waypoints.getLabeledWaypointListForLoad();
        } else {
            waypointList = waypoints.getLabeledWaypointListForBuild();
        }

        for(LabeledWaypoint waypoint: waypointList) {
            System.out.println(waypoint.waypoint_n2d.toString() + ",    " + waypoint.label + ";");
        }

        String filename_waypointCSV =
                waypoints.getTeamColor().toString() + "_" +
                startPosition.toString() + "_" +
                waypoints.getSkystoneDetectionPosition() + ".csv";
        writeWaypointCSV(filename_waypointCSV,waypointList);

    }

    private void writeWaypointCSV(String filename,List<LabeledWaypoint> labeledWaypoints) throws IOException {
        FileWriter csvWriter = new FileWriter(filename);

        try {
            csvWriter.append("Name,");
            csvWriter.append("X inches,");
            csvWriter.append("Y inches,");
            csvWriter.append("Theta degrees;\n");

            // Add data
            for(LabeledWaypoint waypoint: labeledWaypoints) {
                csvWriter.append(waypoint.label + ",");
                csvWriter.append(waypoint.waypoint_n2d.toString() + ";\n");
            }
            System.out.println("\n"+"Wrote file:  " + filename);
        } catch (IOException ioException) {
            ioException.printStackTrace();
        } finally {
            if (csvWriter != null) {
                csvWriter.flush();
                csvWriter.close();
            }
        }





    }

}
