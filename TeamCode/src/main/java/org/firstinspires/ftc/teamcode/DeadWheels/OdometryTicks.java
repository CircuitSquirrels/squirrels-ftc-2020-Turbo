package org.firstinspires.ftc.teamcode.DeadWheels;

public class OdometryTicks {

    public double left,center,right;

    public OdometryTicks(double left, double center, double right) {
        this.left = left;
        this.center = center;
        this.right = right;
    }

    public OdometryTicks subtractAndReturn(OdometryTicks ticks) {
        return new OdometryTicks(left - ticks.left, center - ticks.center, right - ticks.right);
    }

    public OdometryTicks addAndReturn(OdometryTicks ticks) {
        return new OdometryTicks(left + ticks.left, center + ticks.center, right + ticks.right);
    }
}
