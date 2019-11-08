import android.util.Pair;

import com.google.common.truth.Truth;
import static com.google.common.truth.Truth.assertThat;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.junit.Test;

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
        assertThat(first_n2d.theta).isEqualTo(second_n2d.theta);
    }


    @Test
    public void Waypoint_startPoint_not_null() {
        System.out.println("Start Test");
        Waypoints waypoints = new Waypoints(Color.Ftc.BLUE, RobotHardware.StartPosition.FIELD_LOADING);
        waypoints.setSkystoneDetectionPosition(1);

        Navigation2D initial_n2d = waypoints.initialPosition;
        Navigation2D scanA_n2d = waypoints.scanPosition_A;

        assertWaypointNotNull(initial_n2d);
        assertWaypoint_thetaEqual(initial_n2d,scanA_n2d);
        assertWaypoint_xEqual(initial_n2d,scanA_n2d);

    }

    @Test
    public void Display_Waypoints() {
        System.out.println("Start Display Test");
        Waypoints waypoints = new Waypoints(Color.Ftc.BLUE, RobotHardware.StartPosition.FIELD_LOADING);
        waypoints.setSkystoneDetectionPosition(1);

        System.out.println("Start Test");
        System.out.println(waypoints.initialPosition.toString());
        System.out.println(waypoints.scanPosition_A.toString());



    }


    class LabeledWaypoint {
        String label = "";
        Navigation2D waypoint_n2d = new Navigation2D(0,0,0);

        LabeledWaypoint(String label, Navigation2D waypoint_n2d) {
            this.label = label;
            this.waypoint_n2d = waypoint_n2d;
        }
    }
}
