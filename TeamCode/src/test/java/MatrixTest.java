import android.opengl.Matrix;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static com.google.common.truth.Truth.assertThat;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class MatrixTest {



    @Before
    public void initialize() {
    }


    @Test
    public void SandboxMatrixF() {
        MatrixF testMatrix = MatrixF.identityMatrix(3);
        System.out.println(testMatrix);
        System.out.println(testMatrix.getColumn(0));
    }


//    @Test
    // Not functioning as I would expect.
    // In particular, I'm having trouble just getting an identity matrix.
    public void SandboxOpenGLMatrix() {
        OpenGLMatrix initialMatrix = OpenGLMatrix.identityMatrix();
        System.out.println(initialMatrix);

        OpenGLMatrix secondMatrix = OpenGLMatrix.translation(0, 0, 3)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 0, -90));
        System.out.println(secondMatrix);
    }



}
