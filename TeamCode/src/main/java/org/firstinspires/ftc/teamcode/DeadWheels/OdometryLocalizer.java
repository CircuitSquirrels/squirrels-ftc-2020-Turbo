package org.firstinspires.ftc.teamcode.DeadWheels;

import java.util.EnumMap;

public class OdometryLocalizer {

    private OdometryConfig odometryConfig;
    // Positive center rotations are to the left
    enum Wheels {
        RIGHT,
        CENTER,
        LEFT
    }

    private EnumMap<Wheels, Integer> previousEncoderPositions = new EnumMap<>(Wheels.class);

    OdometryLocalizer(OdometryConfig odometryConfig) {
        this.odometryConfig = odometryConfig;
    }

    public void update(int right, int center, int left) {
        double forward_in = odometryConfig.inchesFromTicks((right + left) / 2.0);
        // 90
        double rotation_degrees_CCW = odometryConfig.inchesFromTicks((right - left) / 2.0) / (odometryConfig.wheelDiameter * Math.PI) * 360.0;
        double strafeCorrection_in = rotation_degrees_CCW * odometryConfig.strafeErrorPerDegrees;
        double strafe_in = odometryConfig.inchesFromTicks(center) - strafeCorrection_in;
        // 10
        double translationDistance = Math.sqrt(Math.pow(forward_in, 2) + Math.pow(strafe_in, 2));
        // Direction of travel relative to forward
        double translationAngle_degrees = Math.toDegrees(Math.atan2(forward_in, strafe_in));

        double pathRadius = 0;
    }
}
