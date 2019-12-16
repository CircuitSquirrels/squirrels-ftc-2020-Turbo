package FakeHardware;


import org.firstinspires.ftc.teamcode.RobotHardware;

// Does not extend DcMotor
// Need to return a MotorConfigurationType object
public class FakeDcMotor{

    double power = 0.0;
    double ticks = 0;
    RobotHardware.MotorName motorName;
    private double time = 0.0;

    FakeDcMotor(RobotHardware.MotorName motorName) {
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