package FakeHardware;

import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class FakeElapsedTime extends ElapsedTime {
    private RobotHardware opMode;
    private double offsetTime;

    FakeElapsedTime(RobotHardware opMode) {
        this.opMode = opMode;
        reset();
    }

    @Override public double time() {
        return opMode.getTime() - offsetTime;
    }

    @Override public void reset() {
            if(opMode == null) {
                offsetTime = 0.0;
            } else {
                offsetTime = opMode.getTime();
            }
    }
}
