import static com.google.common.truth.Truth.assertThat;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

import static org.firstinspires.ftc.teamcode.RobotHardware.StartPosition.FIELD_LOADING;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;

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

    String WAYPOINT_OUTPUT_PATH = "./TestData/waypoint_output/";
/*
    Waypoints waypoints;

    @Before
    public void setupTest() {
        waypoints = new Waypoints(Color.Ftc.BLUE,0);
    }
 */

//    @Test
    public void WaypointsDefinedForAllCases() {
        // Check RED and BLUE, BUILD AND LOAD, skystone 0,1,2, false skystone 3,4,5,6, -1 for errors.
    }

    @Test
    public void Red_Waypoint_Mirror_Checking() {
        System.out.println("Start Red Waypoint Mirroring Test");
        Waypoints waypoints = new Waypoints(Color.Ftc.RED,1);
        waypoints.setSkystoneDetectionPosition(3);

        Navigation2D initialPosition_n2d = waypoints.loading.get(INITIAL_POSITION);
        System.out.println(INITIAL_POSITION.toString());
        System.out.println(initialPosition_n2d.toString());

        Navigation2D alignmentA_n2d = waypoints.loading.get(ALIGNMENT_POSITION_A);
        System.out.println(ALIGNMENT_POSITION_A.toString());
        System.out.println(alignmentA_n2d.toString());

        // Check that all headings are the same as the initial heading. Show any discrepancies.
        System.out.println();
        boolean didAngleFail = false;
        List<Navigation2D> waypointList = waypoints.getLabeledWaypointListForLoad();
        for(Navigation2D labeledWaypoint: waypointList) {
            if(initialPosition_n2d.theta != labeledWaypoint.theta) {
                double newWaypointDegrees = Math.toDegrees(labeledWaypoint.theta);
                System.out.println(labeledWaypoint.getLabel() + "  angle is  " + newWaypointDegrees + "  which is incorrect.");
                didAngleFail = true;
            }
        }
        assertThat(didAngleFail).isFalse();
    }



    @Test
    public void GenerateAllWaypoits() throws IOException {
        for(RobotHardware.StartPosition startPosition: RobotHardware.StartPosition.values()) {
            Display_Waypoints(Color.Ftc.BLUE, startPosition, 2);
        }
    }


    private void Display_Waypoints(Color.Ftc color, RobotHardware.StartPosition startPosition, int skystoneIndex_0to5) throws IOException {

        System.out.println("\n"+"*** Start Waypoint Display **** ");
        Waypoints waypoints = new Waypoints(color,skystoneIndex_0to5);
        System.out.println("Team Color:                     " + waypoints.getTeamColor());
        System.out.println("Start Position:                 " + startPosition);
        System.out.println("Skystone Detection Position:    " + waypoints.getSkystoneDetectionPosition());
        System.out.println();

        List<Navigation2D> waypointList;
        if(startPosition == FIELD_LOADING) {
            waypointList = waypoints.getLabeledWaypointListForLoad();
        } else {
            waypointList = waypoints.getLabeledWaypointListForBuild();
        }

        for(Navigation2D waypoint: waypointList) {
            System.out.println(waypoint.toString() + ",    " + waypoint.getLabel() + ";");
        }

        // Add stone positions to csv for comparison
        for(int i = 0; i<=5; ++i) {
            waypointList.add(waypoints.stoneLocations.get(i).copyAndLabel("Stone " + String.valueOf(i)));
        }

        String filename_waypointCSV =
                WAYPOINT_OUTPUT_PATH +
                waypoints.getTeamColor().toString() + "_" +
                startPosition.toString() + "_" +
                waypoints.getSkystoneDetectionPosition() + ".csv";
        writeWaypointCSV(filename_waypointCSV,waypointList);

    }

    private void writeWaypointCSV(String filename,List<Navigation2D> labeledWaypoints) throws IOException {
        File file = new File(filename);
        File directory = new File(file.getParent());
//        System.out.println(directory.getPath());
        if(!directory.exists()) {
            directory.mkdir();
        }

        FileWriter csvWriter = new FileWriter(filename);

        try {
            csvWriter.append("Name,");
            csvWriter.append("X inches,");
            csvWriter.append("Y inches,");
            csvWriter.append("Theta degrees;\n");

            // Add data
            for(Navigation2D waypoint: labeledWaypoints) {
                csvWriter.append(waypoint.getLabel() + ",");
                csvWriter.append(waypoint.toString() + ";\n");
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
