package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utilities.TimingMonitor;


/**
 * Created by Ashley on 12/14/2017.
 */

@TeleOp(name="Diagnostic", group="Diagnostic")
public class DiagnosticOpMode extends Manual {

    IMUUtilities imuHelper;
    TimingMonitor timingMonitor;

    @Override
    public void init() {
        super.init();
        imuHelper = new IMUUtilities(this, "IMU_1");
        telemetry.addData("Diagnostic Mode ", " Initialized");
        timingMonitor = new TimingMonitor(this);
    }

    @Override
    public void loop() {
        timingMonitor.loopStart();
        super.loop();
        timingMonitor.checkpoint("Main Loop");
        if (imuHelper.imu != null) imuHelper.update();
        timingMonitor.checkpoint("IMU update");
        showDiagnosticTelemetry();
        telemetry.addLine();
        if (imuHelper.imu != null) imuHelper.displayTelemetry();
        telemetry.addData("IMU Rotation",imuHelper.heading);
        telemetry.addLine();
        timingMonitor.displayMaxTimes();
        telemetry.addLine();
        timingMonitor.checkpoint("Telemetry Displayed");
    }


    public void showDiagnosticTelemetry() {

        telemetry.addLine();
        telemetry.addData("Period Average (sec)", df_prec.format(getAveragePeriodSec()));
        telemetry.addData("Period Max (sec)", df_prec.format(getMaxPeriodSec()));

        // Show color sensor telemetry only if sensor is attached
//        if (colorSensorExists(ColorSensorName.MINERAL_COLOR)) {
//            displayColorSensorTelemetry();
//        }

        // Display all ODS sensor light levels
//        for (OpticalDistanceSensorName o : OpticalDistanceSensorName.values()) {
//            telemetry.addData(o.name(), df_prec.format(getOpticalDistanceSensorLightLevel(o)));
//        }

        telemetry.addLine(); // Create Space

        // Display all servo positions
        for (ServoName s : ServoName.values()) {
            telemetry.addData(s.name(), df.format(getAngle(s)));
        }

        telemetry.addLine(); // Create Space

        // Display all motor encoder values
        for (MotorName m : MotorName.values()) {
            telemetry.addData(m.name(), getEncoderValue(m));
        }
    }


}
