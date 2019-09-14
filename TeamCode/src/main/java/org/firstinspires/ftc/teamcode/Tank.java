package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Tank POV", group = "Testing")
public class Tank extends RobotHardware {

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.left_stick_x;
        double leftPower  = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower = Range.clip(drive - turn, -1.0, 1.0) ;
        setDriveForTank(leftPower, rightPower);

        setPower(MotorName.FEEDER, gamepad1.right_trigger);
        setPower(MotorName.ARM, -gamepad1.right_stick_y);
        setPower(MotorName.WRIST, gamepad1.right_stick_x);

        // Telemetry
//        telemetry.addLine(); // Create Space
        // Display all motor encoder values
        for (MotorName m : MotorName.values()) {
            telemetry.addData(m.name(), getEncoderValue(m));
        }

    }


}
