package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utilities.SimpleVision;


/**
 * Created by Ashley on 12/14/2017.
 */

@TeleOp(name="Diagnostic Vision", group="Diagnostic")
public class DiagnosticVision extends DiagnosticOpMode {

    SimpleVision simpleVision;
    private Thread thread;

    @Override
    public void init() {
        super.init();

        thread = new Thread(new VisionLoader());
        thread.start();
        telemetry.addData("Diagnostic Vision Mode ", " Initialized");
    }

    @Override
    public void init_loop(){
        super.init_loop();
        if (simpleVision == null) {
            telemetry.addData("Vision:", "LOADING...");
        } else {
            telemetry.addData("Vision:", "INITIALIZED");
        }
    }

    @Override
    public void loop() {
        super.loop();
        if (simpleVision != null) {
            simpleVision.updateVuMarkPose();
            telemetry.addData("VuForia Nav2D",simpleVision.getPositionNav2D());
            simpleVision.updateTensorFlow(false);
            simpleVision.displayTensorFlowDetections();
            simpleVision.identifyMineral(SimpleVision.MineralIdentificationLocation.CENTER);
        }
    }


    // Initialize vision in a separate thread to avoid init() hangups.
    class VisionLoader implements Runnable {
        public void run() {
        simpleVision = new SimpleVision(getVuforiaLicenseKey(),DiagnosticVision.this,
        false, true,true, true, true);
        }
    }

}
