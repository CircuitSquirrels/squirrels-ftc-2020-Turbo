import org.firstinspires.ftc.teamcode.DeadWheels.OdometryConfig;
import org.firstinspires.ftc.teamcode.DeadWheels.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.DeadWheels.OdometryTicks;
import org.junit.Before;
import org.junit.Test;

import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

public class OdometryTest {

    OdometryLocalizer odometryLocalizer;
    OdometryConfig odometryConfig = new OdometryConfig();

    @Before
    public void initialize() {
        odometryLocalizer = new OdometryLocalizer(odometryConfig);
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

    @Test
    public void odometryMotion() {
        // Calculating a series of motions in the robot frame.
        // Cannot support motion in angle and rotation together.

        System.out.println(odometryConfig.ticksFromInches(odometryConfig.inchesFromTicks(100)));

        odometryLocalizer.update(getTicksForMotion(new Navigation2D(0,0,Math.toRadians(0)),odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition()));
        System.out.println(odometryLocalizer.getCurrentPosition());

        odometryLocalizer.update(getTicksForMotion(new Navigation2D(10,0,Math.toRadians(0)),odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition()));
        System.out.println(odometryLocalizer.getCurrentPosition());

        odometryLocalizer.update(getTicksForMotion(new Navigation2D(10,0,Math.toRadians(0)),odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition()));
        System.out.println(odometryLocalizer.getCurrentPosition());

        odometryLocalizer.update(getTicksForMotion(new Navigation2D(0,0,Math.toRadians(90)),odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition()));
        System.out.println(odometryLocalizer.getCurrentPosition());

        odometryLocalizer.update(getTicksForMotion(new Navigation2D(10,20,Math.toRadians(0)),odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition()));
        System.out.println(odometryLocalizer.getCurrentPosition());

        odometryLocalizer.update(getTicksForMotion(new Navigation2D(0,0,Math.toRadians(-90)),odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition()));
        System.out.println(odometryLocalizer.getCurrentPosition());

        odometryLocalizer.update(getTicksForMotion(new Navigation2D(0,-10,Math.toRadians(0)),odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition()));
        System.out.println(odometryLocalizer.getCurrentPosition());

        // Same as above, move by increment in robot frame
        updateOdometryForRobotFrameMotion(new Navigation2D(0,15,Math.toRadians(0)));

        // Move to specific location.
        Navigation2D globalPosition = new Navigation2D(50,50,0);
        updateOdometryForRobotFrameMotion(globalPosition.getNav2DInLocalFrame(odometryLocalizer.getRobotFrame()));

        updateOdometryForRobotFrameMotion((new Navigation2D(0,0,0)).getNav2DInLocalFrame(odometryLocalizer.getRobotFrame()));

        updateOdometryForRobotFrameMotion(new Navigation2D(0,0,Math.toRadians(17.357)));

        updateOdometryForRobotFrameMotion(new Navigation2D(0,0,Math.toRadians(-57.123)));

        updateOdometryForRobotFrameMotion((new Navigation2D(0,0,0)).getNav2DInLocalFrame(odometryLocalizer.getRobotFrame()));
        
    }


    void updateOdometryForRobotFrameMotion(Navigation2D robotFrameDeltaMotion) {
        OdometryTicks ticksForMotion = getTicksForMotion(robotFrameDeltaMotion,odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition());
        odometryLocalizer.update(ticksForMotion);
        System.out.println(odometryLocalizer.getCurrentPosition());
    }



    /**
     * Calculate ticks based on a motion in inches and radians.
     * Not currently valid for combined rotation and translation.
     * @param deltaPosition
     * @return OdometryTicks left,center,right
     */
    public OdometryTicks getTicksForMotion(Navigation2D deltaPosition, OdometryConfig odometryConfig) {
        double threshold = 1e-3;

        // Not valid for combined translation and rotation, so throw error.
        if((Math.abs(deltaPosition.x) > threshold || Math.abs(deltaPosition.y) > threshold) && Math.abs(deltaPosition.theta) > threshold) {
            throw new RuntimeException("getTicksForMotion() does not support calculating ticks for motions where " +
                    "both translation and rotation are non-zero.");
        }

        double rightTicks, leftTicks, centerTicks;
        if(Math.abs(deltaPosition.theta) > threshold) {
            // Rotating motion
            double outerWheelRadius = odometryConfig.outerWheelDistance/2.0;
            double innerWheelRadius = Math.abs(odometryConfig.centerWheelXPos);

            rightTicks  = odometryConfig.ticksFromInches( deltaPosition.theta * outerWheelRadius);
            leftTicks   = odometryConfig.ticksFromInches(-deltaPosition.theta * outerWheelRadius);
            centerTicks = odometryConfig.ticksFromInches(-deltaPosition.theta * innerWheelRadius);
        } else {
            // Translating moting
            rightTicks  = odometryConfig.ticksFromInches( deltaPosition.x);
            leftTicks   = odometryConfig.ticksFromInches( deltaPosition.x);
            centerTicks = odometryConfig.ticksFromInches( deltaPosition.y);
        }

        return new OdometryTicks(leftTicks, centerTicks, rightTicks);
    }

}
