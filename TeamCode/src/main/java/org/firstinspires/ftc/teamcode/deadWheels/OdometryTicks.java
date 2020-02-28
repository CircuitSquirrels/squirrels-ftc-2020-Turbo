package org.firstinspires.ftc.teamcode.deadWheels;

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

    public String toString() {
        String format_xy = "%6d";
        return "Left: " + String.format(format_xy,(int) left) + ",  " + "Center: " + String.format(format_xy,(int) center) + ", " + "Right: " + String.format(format_xy,(int) right) + "";
    }

}
