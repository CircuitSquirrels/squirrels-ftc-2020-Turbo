import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Frame2D;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;


public class Frame2DTest {


    Frame2D f0, f1, f2;
    Navigation2D point0, point1, point2;

    @Before
    public void initialize() {
        f0 = new MecanumNavigation.Frame2D();
        f1 = new MecanumNavigation.Frame2D(-2,-1,0);
        f2 = new MecanumNavigation.Frame2D(-2,1,Math.toRadians(90));

        point0 = new Navigation2D(2,1,0);
        point1 = new Navigation2D(4,2,0,f1);
        point2 = new Navigation2D(0,-4,Math.toRadians(-90),f2);
    }


    @Test
    public void localPoints_toWorldFrame() {
        describeNav2dFrame(point0);
        TestFTC.assertWaypoint_Equal(point0,point0.getNav2DInWorldFrame());

        describeNav2dFrame(point1);
        TestFTC.assertWaypoint_Equal(point0,point1.getNav2DInWorldFrame());

        describeNav2dFrame(point2);
        TestFTC.assertWaypoint_Equal(point0,point2.getNav2DInWorldFrame());
    }

    @Test
    public void worldFramePoints_toLocalFrame() {
        // local1 and local2 are simply the point0 global position moved into the f1 and f2 local frames.
        // The comparison is between the local x,y,theta values.
        // The frames should also be equal.
        Navigation2D local0 = point0.getNav2DInLocalFrame(null);
        Navigation2D local1 = point0.getNav2DInLocalFrame(f1);
        Navigation2D local2 = point0.getNav2DInLocalFrame(f2);

        describeNav2dFrame(point0);
        TestFTC.assertWaypoint_Equal(point0,local0);

        describeNav2dFrame(point1);
        TestFTC.assertWaypoint_Equal(point1,local1);

        describeNav2dFrame(point2);
        TestFTC.assertWaypoint_Equal(point2,local2);
    }

    
    @Test
    public void globalToLocalToGlobal_Loop() {
        ArrayList<Navigation2D> globalPoints = new ArrayList<>();
        ArrayList<Frame2D> localFrames = new ArrayList<>();
        
        globalPoints.add(new Navigation2D(0,0,Math.toRadians(0)));
        globalPoints.add(new Navigation2D(1,0,Math.toRadians(0)));
        globalPoints.add(new Navigation2D(1.5,5.5,Math.toRadians(90)));
        globalPoints.add(new Navigation2D(-4,0,Math.toRadians(180)));
        globalPoints.add(new Navigation2D(4,10,-Math.toRadians(45)));
        
        localFrames.add(new Frame2D(0,0,Math.toRadians(0)));
        localFrames.add(new Frame2D(0,2,Math.toRadians(0)));
        localFrames.add(new Frame2D(0,0,Math.toRadians(90)));
        localFrames.add(new Frame2D(-10,-10,Math.toRadians(30)));
        localFrames.add(new Frame2D(-10,7,Math.toRadians(180)));
        localFrames.add(new Frame2D(9,-2.5,-Math.toRadians(361)));

        Navigation2D localIntermediate, globalReconstruction;
        for(Navigation2D point : globalPoints) {
            for( Frame2D localFrame : localFrames) {
                localIntermediate = point.getNav2DInLocalFrame(localFrame);
                globalReconstruction = localIntermediate.getNav2DInWorldFrame();
                TestFTC.assertWaypoint_Equal(point,globalReconstruction);
                describeNav2dFrame(localIntermediate);
            }
        }
    }

    @Test
    public void FoundationWaypoints() {
        Waypoints waypoints = new Waypoints(Color.Ftc.BLUE,0);

        // This frame will be the marker for pivoting the waypoints, and must be identified in the new position
        Frame2D foundationDropOffFrame = new Frame2D(waypoints.loading.get(Waypoints.LocationLoading.FOUNDATION_PLACE).copy());
        // New location of the foundation indicated
        Frame2D foundationDropOffFrame_moved = new Frame2D(foundationDropOffFrame.positionInReferenceFrame.addAndReturn(-10,10,Math.toRadians(135)));
        // Populate stone locations on platform
        ArrayList<Navigation2D> foundationDropLocations = new ArrayList<>();
        for(int i = 0; i<=2; ++i) {
            System.out.println("Waypoint index:" + i);
            Navigation2D positionInGlobalFrame = waypoints.loading.get(Waypoints.LocationLoading.FOUNDATION_PLACE).addAndReturn(-8.0*i, 0, 0);
//            System.out.println("Initial Position, Global Frame:");
//            describeNav2dFrame(positionInGlobalFrame);

            Navigation2D positionInInitialFoundationFrame = positionInGlobalFrame.getNav2DInLocalFrame(foundationDropOffFrame);
//            System.out.println("Initial Position, Local Frame:");
//            describeNav2dFrame(positionInInitialFoundationFrame);
            positionInInitialFoundationFrame.referenceFrame = foundationDropOffFrame_moved;

            Navigation2D positionOfRelocatedPointsInGlobalFrame = positionInInitialFoundationFrame.getNav2DInWorldFrame();
            foundationDropLocations.add(positionOfRelocatedPointsInGlobalFrame);
            System.out.println("Moved Position, Global Frame:");
            describeNav2dFrame(positionInInitialFoundationFrame);

        }

    }

    @Test
    public void calculateLastBlockWaypoint() {
        Waypoints waypoints = new Waypoints(Color.Ftc.BLUE,2);
        Navigation2D wallStone = waypoints.stoneLocations.get(5);
        Navigation2D currentStonePickupLocation = waypoints.loading.get(Waypoints.LocationLoading.GRAB_SKYSTONE_B);

        // Setup robot frame. This can be moved later (by adjusting
        Frame2D robotFrame = new Frame2D();
        robotFrame.positionInReferenceFrame = waypoints.loading.get(INITIAL_POSITION); // This is how the frame can be moved.
        Navigation2D grabber = new Navigation2D(9.0,0,0,robotFrame);
        Navigation2D rightWheel = new Navigation2D(8.0,-17.0/2.0,0,robotFrame);
        Navigation2D leftWheel = new Navigation2D(8.0,17.0/2.0,0,robotFrame);


        double approachAngleDegrees = -55;
        // Initial setup: find alignment and grab positions on a given angle
        Frame2D stoneFrame = new Frame2D(wallStone.addAndReturn(0,0,Math.toRadians(approachAngleDegrees)));
        Navigation2D grab1_local = new Navigation2D(-9,0,0, stoneFrame);
        Navigation2D align_local = new Navigation2D(-20, 0, 0, stoneFrame);

        System.out.println(approachAngleDegrees);
        System.out.println("Alignment Position");
        describeNav2dFrame(align_local);
        // Check for interference between wheel and wall
        robotFrame.positionInReferenceFrame = align_local.getNav2DInWorldFrame();
        System.out.println("Front Right Wheel Location:  " + rightWheel.getNav2DInWorldFrame());

        System.out.println("Grab Position");
        describeNav2dFrame(grab1_local);
        // Check for interference between wheel and wall
        robotFrame.positionInReferenceFrame = grab1_local.getNav2DInWorldFrame();
        System.out.println("Front Right Wheel Location:  " + rightWheel.getNav2DInWorldFrame());



        // Wheel pivot method (right wheel, blue side)
        Frame2D rightWheelFrame = new Frame2D(rightWheel,robotFrame);
        Frame2D robotFrameInWheelFrame = new Frame2D(robotFrame.positionInReferenceFrame.getNav2DInLocalFrame(rightWheelFrame),rightWheelFrame);
//        describeNav2dFrame(robotFrameInWheelFrame);


    }



    public void describeNav2dFrame(Navigation2D navigation2D) {
        Navigation2D nav2D_inWorld = navigation2D.getNav2DInWorldFrame();

        System.out.println("Nav2D Local Coordinates:   ");
        System.out.println(navigation2D.toString());
        System.out.println("Frame Origin in its reference: ");
        if(navigation2D.referenceFrame == null) {
            System.out.println("Reference frame is null, or the world frame.");
        } else {

            System.out.println(navigation2D.referenceFrame.positionInReferenceFrame.toString());
            System.out.println("Nav2D World Coordinates:   ");
            System.out.println(nav2D_inWorld);
        }
        System.out.println("");
    }



}
