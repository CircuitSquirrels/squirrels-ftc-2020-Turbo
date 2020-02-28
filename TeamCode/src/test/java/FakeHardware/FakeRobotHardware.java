package FakeHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

// Mockito
//import org.mockito.Mockito;


import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.deadWheels.OdometryTicks;


import java.util.EnumMap;

public class FakeRobotHardware extends RobotHardware {

    //public double time; This already exists in RobotHardware, and should not be hidden.
    // Remember, Java doesn't allow subclasses to Override parent fields, only hide them, so
    // it is better not to declare 'time' here, lest we fail to modify the parent field.

    // Run to initialize fake internals
    public void initializeFakeOpMode() {
        initFakeMotors();
    }

    // call within a simulation loop, giving current sim time, to propagate simulated hardware state.
    public void updateAndIntegrateFakeOpMode(double time) {
        this.time = time;
        // update motor positions
        for(MotorName motorName: MotorName.values()) {
            fakeMotorMap.get(motorName).updateAndIntegratePosition(time);
        }
    }

    @Override
    public void setPower(MotorName motor, double power) {
        FakeDcMotor fakeDcMotor = fakeMotorMap.get(motor);
        fakeDcMotor.setPower(power);
        // No missing motor handling here
    }

    @Override
    public int getEncoderValue(MotorName motor) {
        return (int) fakeMotorMap.get(motor).getTicks();
    }

    @Override
    public void setDriveForMecanumWheels(Mecanum.Wheels wheels) {
        super.setDriveForMecanumWheels(wheels);
    }

    @Override
    public double getTime() {
        return this.time;
    }

    private double averagePeriodSec = 0.02;

    @Override
    public double getAveragePeriodSec() {
        return averagePeriodSec;
    }

    public void setAveragePeriodSec(double averagePeriodSec) {
        this.averagePeriodSec = averagePeriodSec;
    }

    public ElapsedTime getNewElapsedTime() {
        return new FakeElapsedTime(this);
    }


    // Initialize Fake stuff

    EnumMap<MotorName,FakeDcMotor> fakeMotorMap = new EnumMap<>(MotorName.class);
    private void initFakeMotors() {
        for(MotorName motorName: MotorName.values()) {
            fakeMotorMap.put(motorName, new FakeDcMotor(motorName));
        }
    }


    // set and get dead wheel encoders
    public void setDeadWheelEncoders(OdometryTicks odometryTicks) {
        fakeMotorMap.get(MotorName.LEFT_WHEEL).setTicks((int) odometryTicks.left);
        fakeMotorMap.get(MotorName.CENTER_WHEEL).setTicks((int) odometryTicks.center);
        fakeMotorMap.get(MotorName.RIGHT_WHEEL).setTicks((int) odometryTicks.right);
    }

    public OdometryTicks getDeadWheelEncoders() {
        return new OdometryTicks(
                fakeMotorMap.get(MotorName.LEFT_WHEEL).getTicks(),
                fakeMotorMap.get(MotorName.CENTER_WHEEL).getTicks(),
                fakeMotorMap.get(MotorName.RIGHT_WHEEL).getTicks());
    }

}
