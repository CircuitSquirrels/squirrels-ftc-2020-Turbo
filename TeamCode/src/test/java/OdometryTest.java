import org.firstinspires.ftc.teamcode.DeadWheels.OdometryConfig;
import org.firstinspires.ftc.teamcode.DeadWheels.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.DeadWheels.OdometryTicks;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Frame2D;

public class OdometryTest {

    OdometryLocalizer odometryLocalizer;

    @Before
    public void initialize() {
        odometryLocalizer = new OdometryLocalizer(new OdometryConfig());
        odometryLocalizer.setCurrentPosition(new Navigation2D(0,0,0));
        odometryLocalizer.setEncoderPosition(new OdometryTicks(0,0,0));
    }

    @Test
    public void stuff() {
        System.out.println(odometryLocalizer.getCurrentPosition());
        odometryLocalizer.update(new OdometryTicks(8192, 0, 8192));
        System.out.println(odometryLocalizer.getCurrentPosition());

        odometryLocalizer.update(new OdometryTicks(-8192.0 * (9.0 / 10.0), 8192.0 * (9.0 / 10.0) * (1.5 / 8.5), 8192.0 * (9.0 / 10.0)));
        System.out.println(odometryLocalizer.getCurrentPosition());
        odometryLocalizer.update(new OdometryTicks(-8192, 0, -8192));
        System.out.println(odometryLocalizer.getCurrentPosition());
        odometryLocalizer.update(new OdometryTicks(0, 8192, 0));
        System.out.println(odometryLocalizer.getCurrentPosition());
        odometryLocalizer.update(new OdometryTicks(8192, 0, -8192));
        System.out.println(odometryLocalizer.getCurrentPosition());
        odometryLocalizer.update(new OdometryTicks(0, 8192, 0));
        System.out.println(odometryLocalizer.getCurrentPosition());

    }

}
