package FakeHardware;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.IMUUtilities;

public class FakeIMUUtilities extends IMUUtilities {

    // Default to Fast, Heading only mode when not specified.
    public FakeIMUUtilities(RobotHardware opMode, String imu_name) {
        this(opMode,imu_name, ImuMode.FAST_HEADING_ONLY);
    }

    public FakeIMUUtilities(RobotHardware opMode, String imu_name, ImuMode imuMode) {
        super(opMode,imu_name,imuMode);

    }

//    public FakeIMUUtilities() {}

}
