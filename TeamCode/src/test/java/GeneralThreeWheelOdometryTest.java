import org.firstinspires.ftc.teamcode.deadWheels.OdometryConfig;
import org.firstinspires.ftc.teamcode.deadWheels.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.deadWheels.OdometryTicks;
import org.firstinspires.ftc.teamcode.deadWheels.StandardTrackingWheelLocalizer;
import org.junit.Before;
import org.junit.Test;

import FakeHardware.FakeRobotHardware;
import TestUtilities.SimFormat;

import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;


public class GeneralThreeWheelOdometryTest {

    private FakeRobotHardware opmode;
    private OdometryLocalizer odometryLocalizer;
    private StandardTrackingWheelLocalizer rrLocalizer;
    private final OdometryConfig odometryConfig = new OdometryConfig();


    @Before
    public void initialize() {
        Navigation2D initialPosition = new Navigation2D(0, 0, 0);
        OdometryTicks initialTicks = new OdometryTicks(0,0,0);
        opmode = new FakeRobotHardware();
        opmode.initializeFakeOpMode();
        opmode.setDeadWheelEncoders(initialTicks);

        odometryLocalizer = new OdometryLocalizer(odometryConfig);
        odometryLocalizer.setCurrentPosition(initialPosition);
        odometryLocalizer.setEncoderPosition(initialTicks);

        rrLocalizer = new StandardTrackingWheelLocalizer(opmode);
        rrLocalizer.setCurrentPosition(initialPosition);
        rrLocalizer.update();
    }

    @Test
    public void simpleEncoder() {
            OdometryTicks incrementalDeadWheelsTicks = new OdometryTicks(0, 100, 0);
//            OdometryTicks incrementalDeadWheelsTicks = new OdometryTicks(100, 10, 100);
//            OdometryTicks incrementalDeadWheelsTicks = new OdometryTicks(-100, 0, 100);
            linearSim(incrementalDeadWheelsTicks);
    }



    // Add a set odometryTick 1000x times and plot the predicted path from the
    // Naive OdometryLocalizer code, which assumes parallel wheels, and the
    // General rrLocalizer, which includes a wheel angle compensation.
    public void linearSim(OdometryTicks incrementalTicks) {
        boolean printedHeading = false;
        OdometryTicks totalDeadwheelTicks = new OdometryTicks(0,0,0);

        for (int i = 0; i < 1000; ++i) {
            totalDeadwheelTicks = totalDeadwheelTicks.addAndReturn(incrementalTicks);
            opmode.setDeadWheelEncoders(totalDeadwheelTicks);
            odometryLocalizer.update(opmode);
            rrLocalizer.update(opmode);

            if (!printedHeading) {
                System.out.println("Increment:  " + incrementalTicks.toString());
                System.out.print(SimFormat.padCenteredStringTo(5, "Time").concat("|"));
                System.out.print(SimFormat.padCenteredStringTo(30, "Naive Localizer").concat("|"));
                System.out.print(SimFormat.padCenteredStringTo(30, "Angled Localizer").concat("|"));
                System.out.println(SimFormat.padCenteredStringTo(50, "Total Ticks").concat("|"));
                System.out.println("---------------------------------------------------------------------------------------------------------------------------------------");
                printedHeading = true;
            }

            if (i % 50 == 0) {
                System.out.print(SimFormat.padCenteredStringTo(5, String.valueOf(i)).concat("|"));
                System.out.print(SimFormat.padCenteredStringTo(30, odometryLocalizer.getCurrentPosition().toString()).concat("|"));
                System.out.print(SimFormat.padCenteredStringTo(30, rrLocalizer.getCurrentPosition().toString()).concat("|"));
                System.out.println(SimFormat.padCenteredStringTo(50, totalDeadwheelTicks.toString()).concat("|"));
            }
        }
    }



    void updateOdometryForRobotFrameMotion(Navigation2D robotFrameDeltaMotion) {
        OdometryTicks ticksForMotion = getTicksForMotion(robotFrameDeltaMotion, odometryConfig).addAndReturn(odometryLocalizer.getEncoderPosition());
        odometryLocalizer.update(ticksForMotion);
        System.out.println(odometryLocalizer.getCurrentPosition());
    }


    /**
     * Calculate ticks based on a motion in inches and radians.
     * Not currently valid for combined rotation and translation.
     *
     * @param deltaPosition
     * @return OdometryTicks left,center,right
     */
    public OdometryTicks getTicksForMotion(Navigation2D deltaPosition, OdometryConfig odometryConfig) {
        double threshold = 1e-3;

        // Not valid for combined translation and rotation, so throw error.
        if ((Math.abs(deltaPosition.x) > threshold || Math.abs(deltaPosition.y) > threshold) && Math.abs(deltaPosition.theta) > threshold) {
            throw new RuntimeException("getTicksForMotion() does not support calculating ticks for motions where " +
                    "both translation and rotation are non-zero.");
        }

        double rightTicks, leftTicks, centerTicks;
        if (Math.abs(deltaPosition.theta) > threshold) {
            // Rotating motion
            double outerWheelRadius = odometryConfig.outerWheelDistance / 2.0;
            double innerWheelRadius = Math.abs(odometryConfig.centerWheelXPos);

            rightTicks = odometryConfig.ticksFromInches(deltaPosition.theta * outerWheelRadius);
            leftTicks = odometryConfig.ticksFromInches(-deltaPosition.theta * outerWheelRadius);
            centerTicks = odometryConfig.ticksFromInches(-deltaPosition.theta * innerWheelRadius);
        } else {
            // Translating moting
            rightTicks = odometryConfig.ticksFromInches(deltaPosition.x);
            leftTicks = odometryConfig.ticksFromInches(deltaPosition.x);
            centerTicks = odometryConfig.ticksFromInches(deltaPosition.y);
        }

        return new OdometryTicks(leftTicks, centerTicks, rightTicks);
    }

}
