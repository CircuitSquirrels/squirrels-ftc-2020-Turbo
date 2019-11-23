package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.SimpleVision;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;


/**
 * Created by Ashley on 12/14/2017.
 */

@TeleOp(name="Diagnostic Vision", group="Diagnostic")
public class DiagnosticVision extends DiagnosticOpMode {

    private Thread thread;
    Waypoints waypoints;

    @Override
    public void init() {
        super.init();
        waypoints = new Waypoints(Color.Ftc.RED);
        thread = new Thread(new VisionLoader());
        thread.start();
        telemetry.addData("Diagnostic Vision Mode ", " Initialized");
    }

    @Override
    public void start() {
        super.start();
        mecanumNavigation.setCurrentPosition(waypoints.loading.get(Waypoints.LocationLoading.INITIAL_POSITION));
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
            simpleVision.displayFormattedVumarkPose();
            telemetry.addData("VuForia Nav2D",simpleVision.getPositionAbsoluteNav2d());
            telemetry.addData("Vuforia Skystone Nav2D",simpleVision.getPositionSkystoneRelativeNav2d());
            telemetry.addData("Skystone Index: ", getSkystoneIndex(waypoints))
                    .addData("Skystone Index Rounded: ", Math.round(getSkystoneIndex(waypoints)));
        }
    }

    // Initialize vision in a separate thread to avoid init() hangups.
    class VisionLoader implements Runnable {
        public void run() {
        simpleVision = new SimpleVision(getVuforiaLicenseKey(),DiagnosticVision.this,
        true, false,true, true,
                SimpleVision.UseWebcamEnum.TRUE, SimpleVision.TensorFlowEnabled.FALSE);
        }
    }

}
