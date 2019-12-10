package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.firstinspires.ftc.teamcode.Vision.AveragingPipeline;
import org.firstinspires.ftc.teamcode.Vision.SkystoneDetector;


/**
 * Created by Ashley on 12/14/2017.
 */

@TeleOp(name="Diagnostic Vision", group="Diagnostic")
public class DiagnosticVision extends DiagnosticOpMode {

    private Thread thread;
    Waypoints waypoints;
    SkystoneDetector skystoneDetector;

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
        if (skystoneDetector == null) {
            telemetry.addData("Vision:", "LOADING...");
        } else {
            telemetry.addData("Vision:", "INITIALIZED");
        }
    }

    @Override
    public void loop() {
        super.loop();

        if (skystoneDetector != null) {
            telemetry.addData("Skystone Location",skystoneDetector.getSkystoneRelativeLocation())
                    .addData("Press A"," to save an image")
                    .addData("Image Number", skystoneDetector.ternarySkystonePipeline.lastInputImage);

            if(controller1.AOnce()) {
                skystoneDetector.ternarySkystonePipeline.saveInputImage("/sdcard/FIRST/images/");
            }
        }

    }

    // Initialize vision in a separate thread to avoid init() hangups.
    class VisionLoader implements Runnable {
        public void run() {
            skystoneDetector.init(new AveragingPipeline());
        }
    }

}
