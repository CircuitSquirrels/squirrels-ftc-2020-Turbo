package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Utilities.Color;

/**
 * Initializes a pipeline and starts streaming.
 * Pipeline is focused on image frame, providing relative position information (left, center, right)
 * SkystoneDetector can use teamColor and starting location information to provide more absolute
 * location information for the skystone, such as location index, where 0 is near the field center.
 */
public class SkystoneDetector {

    RobotHardware opmode;
    HardwareMap hardwareMap;
    Color.Ftc teamColor;
    public OpenCvCamera webcam;
    TernarySkystonePipeline ternarySkystonePipeline;

    private final int rows = 640;
    private final int cols = 480;


    public SkystoneDetector(RobotHardware opmode, Color.Ftc teamColor) {
       this.teamColor = teamColor;
       this.opmode = opmode;
       this.hardwareMap = opmode.hardwareMap;
       this.ternarySkystonePipeline = new AveragingPipeline();
       init(this.ternarySkystonePipeline);
    }

    public void init(TernarySkystonePipeline ternarySkystonePipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(ternarySkystonePipeline);
        // If the resolution specified is not supported by the camera, an error will be thrown.
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);
    }

    public SkystoneRelativeLocation getSkystoneRelativeLocation() {
        return ternarySkystonePipeline.getSkystoneRelativeLocation();
    }

}


