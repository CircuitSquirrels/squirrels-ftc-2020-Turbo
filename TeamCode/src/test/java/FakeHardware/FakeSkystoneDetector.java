package FakeHardware;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Vision.SkystoneDetector;

public class FakeSkystoneDetector extends SkystoneDetector {
    private int skystoneIndex = 0;


    public FakeSkystoneDetector(RobotHardware opmode) {
        super(opmode);
    }

    public void setSkystoneIndex(int skystoneIndex) {
        this.skystoneIndex = skystoneIndex;
    }

    @Override
    public int getSkystoneIndex() {
        return skystoneIndex;
    }
}
