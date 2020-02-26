package FakeHardware;// Mockito
//import org.mockito.Mockito;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoOpmode;
import org.firstinspires.ftc.teamcode.deadWheels.OdometryLocalizer;
import org.firstinspires.ftc.teamcode.DrivetrainControl.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.IMUUtilities;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;

import java.util.EnumMap;

public class FakeAutoOpmode extends AutoOpmode {

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

    public void setMecanumNavigation(MecanumNavigation mecanumNavigation) {
        this.mecanumNavigation = mecanumNavigation;
    }

    public void setOdometryLocalizer(OdometryLocalizer odometryLocalizer) {
        this.odometryLocalizer = odometryLocalizer;
    }

    public void setAutoDrive(AutoDrive autoDrive) {
        this.autoDrive = autoDrive;
    }

    public void setIMUUtilities(IMUUtilities imuUtilities) {
        this.imuUtilities = imuUtilities;
    }

    public void setDriveSpeed(double driveSpeed) {
        DriveSpeed.set(driveSpeed);
    }

    public void setSimpleAuto(boolean simpleAuto) {
        SimpleAuto.set(simpleAuto);
    }

    public void setSkystoneIndex(int skystoneIndex) {
        FakeSkystoneDetector fakeSkystoneDetector = (FakeSkystoneDetector) skystoneDetector;
        fakeSkystoneDetector.setSkystoneIndex(skystoneIndex);
    }

    public void setParkInner(boolean parkInner) {
        ParkInner.set(parkInner);
    }

    public void setDropStones(boolean dropStones) {
        DropStones.set(dropStones);
    }

    public void setMoveFoundation(boolean moveFoundation) {
        Foundation.set(moveFoundation);
    }

    public void setConservativeMode(boolean conservativeMode) {ConservativeRoute.set(conservativeMode); }

    @Override
    public void setAngle(ServoName servo, double position) {
        //do nothing
    }

    public ElapsedTime getNewElapsedTime() {
        return new FakeElapsedTime(this);
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

    //


    // Initialize Fake stuff

    EnumMap<MotorName,FakeDcMotor> fakeMotorMap = new EnumMap<>(MotorName.class);
    private void initFakeMotors() {
        for(MotorName motorName: MotorName.values()) {
            fakeMotorMap.put(motorName, new FakeDcMotor(motorName));
        }
    }
}
