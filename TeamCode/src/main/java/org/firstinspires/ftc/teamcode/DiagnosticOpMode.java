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
        imuHelper = new IMUUtilities(this, "IMU_1", IMUUtilities.ImuMode.FAST_HEADING_ONLY);
        telemetry.addData("Diagnostic Mode ", " Initialized");
        timingMonitor = new TimingMonitor(this);
    }

    @Override
    public void loop() {
        timingMonitor.loopStart();
        super.loop();
        timingMonitor.checkpoint("Main Loop");
        showDiagnosticTelemetry();
        if (imuHelper.imu != null) {
            telemetry.addLine("****IMU Telemetry****");
            imuHelper.updateNow();
            timingMonitor.checkpoint("IMU update");
            imuHelper.displayTelemetry();
        }
        telemetry.addLine();
        timingMonitor.displayMaxTimes();
        telemetry.addLine();
        showPIDFTelemetry();
        timingMonitor.checkpoint("Telemetry Displayed");
    }


    private void showDiagnosticTelemetry() {
        telemetry.addLine();
        telemetry.addData("Period Average (sec)", df_prec.format(getAveragePeriodSec()));
        telemetry.addData("Period Max (sec)", df_prec.format(getMaxPeriodSec()));
        telemetry.addLine();

        // Display all servo positions
        for (ServoName s : ServoName.values()) {
            telemetry.addData(s.name(), df.format(getAngle(s)));
        }

        telemetry.addLine();

        // Display all motor encoder values
        for (MotorName m : MotorName.values()) {
            telemetry.addData(m.name(), getEncoderValue(m));
        }
    }

    private void showPIDFTelemetry() {
        telemetry.addLine()
                .addData("P", K_P)
                .addData("I", K_I)
                .addData("D", K_D)
                .addData("F", K_F);
    }
}
