import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.junit.Before;
import org.junit.Test;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import static org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Frame2D;
public class Frame2DTest {



    @Before
    public void initialize() {
    }


    @Test
    public void Frame2D_points() {
        Frame2D f0 = new MecanumNavigation.Frame2D();
        Frame2D f1 = new MecanumNavigation.Frame2D(-2,-1,0);
        Frame2D f2 = new MecanumNavigation.Frame2D(-2,1,Math.toRadians(90));

        Navigation2D point0 = new Navigation2D(2,1,0);
        Navigation2D point1 = new Navigation2D(4,2,0,f1);
        Navigation2D point2 = new Navigation2D(0,-4,Math.toRadians(-90),f2);

        // Point 1 absolute frame
        describeNav2dFrame(point0);
        describeNav2dFrame(point1);
        describeNav2dFrame(point2);

    }

    public void describeNav2dFrame(Navigation2D navigation2D) {
        Navigation2D nav2D_inWorld = navigation2D.getNav2dInWorldFrame();

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
