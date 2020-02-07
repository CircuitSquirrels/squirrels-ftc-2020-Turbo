package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utilities.TimingMonitor;

import java.util.function.BinaryOperator;
import java.util.function.BooleanSupplier;
import java.util.function.IntBinaryOperator;


/**
 * Created by Ashley on 12/14/2017.
 */

@TeleOp(name="Diagnostic", group="Diagnostic")
public class DiagnosticOpMode extends Manual {

    IMUUtilities imuHelper;
    private TimingMonitor timingMonitor;

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

        // Control Tuning
        manualControlTuning();
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

    private void manualControlTuning() {
        // Controls Damping factor with dpad up and down.  Sets to 0.1 if at 0.
        // Every 10 presses of upDpad increases K_D by a factor of 10.
        double adjustmentFactor = Math.pow(10.0,1.0/10.0); // 10 steps per decade

        // Hold left trigger to modify the PIDF parameters.
        if(controller1.left_trigger > 0.5) {
            singlePIDFParameterTuning(ControlParameter.P, controller1.dpadUpOnce(), controller1.dpadDownOnce(),0.1, adjustmentFactor);
            singlePIDFParameterTuning(ControlParameter.I, controller1.dpadRightOnce(), controller1.dpadLeftOnce  (),0.1, adjustmentFactor);
            singlePIDFParameterTuning(ControlParameter.D, controller1.rightBumperOnce(), controller1.leftBumperOnce(),0.1, adjustmentFactor);
            singlePIDFParameterTuning(ControlParameter.F, controller1.YOnce(), controller1.XOnce(),0.1, adjustmentFactor);
        }
        telemetry.addData("Tune P:","Dpad Up/Down")
                .addData("Tune I:","Dpad Right/Left")
                .addData("Tune D:","Bumpers Right/Left")
                .addData("Tune F:","Increase Y / Decrease X");
    }


    private void singlePIDFParameterTuning(ControlParameter controlParameter,
                                           boolean upOp, boolean downOp,
                                           double minNonZeroValue, double adjustmentFactor) {
        double currentValue = getSingleDriveControlParameter(controlParameter);
        if(upOp) {
            if (currentValue == 0.0) {
                setSingleDriveControlParameter(controlParameter,minNonZeroValue);
            } else {
                changeDriveControlParameterByFactor(controlParameter,adjustmentFactor);
            }
        } else if (downOp) {
            if (currentValue <= minNonZeroValue) {
                setSingleDriveControlParameter(controlParameter,0.0);
            } else {
                changeDriveControlParameterByFactor(controlParameter,1.0/adjustmentFactor);
            }
        }
    }


    public interface BooleanFunction {
         boolean getBoolean();
    }
}
