package TestUtilities;

import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;


import static com.google.common.truth.Truth.assertThat;

public class AssertFTC {
    static double tolerance = 0.01;

    static public void assertWaypoint_xEqual(Navigation2D first_n2d, Navigation2D second_n2d) {
        assertThat(first_n2d.x).isWithin(tolerance).of(second_n2d.x);
    }

    static public void assertWaypoint_yEqual(Navigation2D first_n2d, Navigation2D second_n2d) {
        assertThat(first_n2d.y).isWithin(tolerance).of(second_n2d.y);
    }

    static public void assertWaypoint_thetaEqual(Navigation2D first_n2d, Navigation2D second_n2d) {
        // Convert theta radians to degrees to make debugging easier.
        assertThat(Math.toDegrees(first_n2d.theta)).isWithin(tolerance).of(Math.toDegrees(second_n2d.theta));
    }

    static public void assertWaypoint_Equal(Navigation2D first_n2d, Navigation2D second_n2d) {
        assertThat(first_n2d.x).isWithin(tolerance).of(second_n2d.x);
        assertThat(first_n2d.y).isWithin(tolerance).of(second_n2d.y);
        assertThat(Math.toDegrees(first_n2d.theta)).isWithin(tolerance).of(Math.toDegrees(second_n2d.theta));
    }

    private void assertMecanumWheelEqual(Mecanum.Wheels wheels1, Mecanum.Wheels wheels2) {
        assertThat(wheels1.backLeft).isWithin(tolerance).of(wheels2.backLeft);
        assertThat(wheels1.backRight).isWithin(tolerance).of(wheels2.backRight);
        assertThat(wheels1.frontLeft).isWithin(tolerance).of(wheels2.frontLeft);
        assertThat(wheels1.frontRight).isWithin(tolerance).of(wheels2.frontRight);
    }
}
