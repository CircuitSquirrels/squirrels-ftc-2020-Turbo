package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.firstinspires.ftc.teamcode.Vision.AveragingPipeline;
import org.firstinspires.ftc.teamcode.Vision.SkystoneDetector;

import java.io.File;



/**
 * Created by Ashley on 12/14/2017.
 */

@TeleOp(name="Diagnostic Vision", group="Diagnostic")
public class DiagnosticVision extends DiagnosticOpMode {

    private Waypoints waypoints;
    private boolean useSettingsDirectory = false;
    private boolean settingsDirectoryTriedAndFailed = false;

    @Override
    public void init() {
        super.init();
        waypoints = new Waypoints(Color.Ftc.RED);
        new Thread(() -> loadVision(DiagnosticVision.this, Color.Ftc.BLUE)).start();
        telemetry.addData("Diagnostic Vision Mode ", " Initialized");
    }

    @Override
    public void start() {
        super.start();
        mecanumNavigation.setCurrentPosition(Waypoints.LocationLoading.INITIAL_POSITION.getNewNavigation2D());
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
                    .addData("Image Number", skystoneDetector.averagingPipeline.lastInputImage);
            try {
                File whatever = AppUtil.getInstance().getSettingsFile("whatever.txt");
                File settingsDirectory = new File(whatever.getParent());
                telemetry.addData("Settings Directory:", settingsDirectory.getAbsoluteFile());
                useSettingsDirectory = true;
                if(settingsDirectoryTriedAndFailed) {
                    telemetry.addData("Settings Write: ", "Tried and failed, attempting appData");
                }
            } catch (Exception e) {
                telemetry.addData("Exception: ", "Unable to get settings file directory");
            }


            if(controller1.AOnce()) {
                if (useSettingsDirectory && !settingsDirectoryTriedAndFailed) {
                    // Save into settings directory
                    try {
                        skystoneDetector.averagingPipeline.saveInputImage();
                    } catch (Exception e) {
                        settingsDirectoryTriedAndFailed = true;
                        skystoneDetector.averagingPipeline.saveInputImage(".");
                    }
                } else {
                    // Lack permissions for: "/sdcard/FIRST/images/" so this should save to appdata.
                    // attempting to save to the current folder, which may be appdata.
                    skystoneDetector.averagingPipeline.saveInputImage(".");
                }
            }
        }

    }
}
