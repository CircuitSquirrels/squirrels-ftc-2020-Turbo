import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

// Mockito
//import org.mockito.Mockito;


import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;


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

    //


    // Initialize Fake stuff

    EnumMap<MotorName,FakeDcMotor> fakeMotorMap = new EnumMap<>(MotorName.class);
    private void initFakeMotors() {
        for(MotorName motorName: MotorName.values()) {
            fakeMotorMap.put(motorName, new FakeDcMotor(motorName));
        }
    }


    // Does not extend DcMotor
    // Need to return a MotorConfigurationType object
    public static class FakeDcMotor{

        double power = 0.0;
        double ticks = 0;
        MotorName motorName;
        private double time = 0.0;

        FakeDcMotor(MotorName motorName) {
            this.motorName = motorName;
        }


        // Fake Physics - basic acceleration limit
        private double maxTicksPerSecond = 1000;
        private double maxTicksPerSecondPerSecond = 3000;
        private double currentTicksPerSecond = 0;



        public void updateAndIntegratePosition(double time) {
            double deltaTime = time - this.time;
            if(deltaTime == 0.0) return;

            // Limit Acceleration
            double desiredTicksPerSecond = power * maxTicksPerSecond;
            double desiredAcceleration_TicksPerSecPerSec = (desiredTicksPerSecond - currentTicksPerSecond) / deltaTime;
            if(Math.abs(desiredAcceleration_TicksPerSecPerSec) > maxTicksPerSecondPerSecond) {
                double accelerationSign = desiredAcceleration_TicksPerSecPerSec / Math.abs(desiredAcceleration_TicksPerSecPerSec);
                // Limit the acceleration
                currentTicksPerSecond += accelerationSign * maxTicksPerSecondPerSecond * deltaTime;
//                System.out.println("ACCELERATION LIMIT ACTIVATED");
            } else {
                currentTicksPerSecond = desiredTicksPerSecond;
            }

            // Limit Velocity
            if(Math.abs(currentTicksPerSecond) > maxTicksPerSecond) {
                currentTicksPerSecond *= Math.abs(maxTicksPerSecond/currentTicksPerSecond);
            }


            this.ticks += currentTicksPerSecond * deltaTime;
            this.time = time;
        }

        public void setPower(double power) {
            this.power = power;
        }

        public double getPower() {
            return power;
        }

        public void setTicks(int ticks) {
            this.ticks = ticks;
        }

        public double getTicks() {
            return ticks;
        }



    }
}
