package org.firstinspires.ftc.teamcode.Utilities;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Mecanum wheel drive calculations.
 * Input controls:
 *   V_d = desired robot speed.
 *   theta_d = desired robot velocity angle.
 *   V_theta = desired robot rotational speed.
 *
 *  Example:
 *    // Convert joysticks to wheel powers.
 *    Mecanum.Wheels wheels = Mecanum.motionToWheels(
 *        Mecanum.joystickToMotion(
 *            gamepad1.left_stick_x, gamepad1.left_stick_y,
 *            gamepad1.right_stick_x, gamepad1.right_stick_y));
 *    // Set power on the motors.
 *    frontLeftMotor.setPower(wheels.frontLeft);
 *
 * Thanks to Phillip Tischler  http://pmtischler-ftc-app.readthedocs.io/en/latest/
 */
public class Mecanum {
    /**
     * Mecanum wheels, used to get individual motor powers.
     */
    public static class Wheels {
        // The mecanum wheels.
        public double frontLeft;
        public double frontRight;
        public double backLeft;
        public double backRight;

        /**
         * Sets the wheels to the given values.
         */
        public Wheels(double frontLeft, double frontRight,
                      double backLeft, double backRight) {
            List<Double> powers = Arrays.asList(frontLeft, frontRight,
                                                backLeft, backRight);
            clampPowers(powers);

            this.frontLeft = powers.get(0);
            this.frontRight = powers.get(1);
            this.backLeft = powers.get(2);
            this.backRight = powers.get(3);
        }

        /**
         * Scales the wheel powers by the given factor.
         * @param scalar The wheel power scaling factor.
         */
        public Wheels scaleWheelPower(double scalar) {
            return new Wheels(frontLeft * scalar, frontRight * scalar,
                              backLeft * scalar, backRight * scalar);
        }

        public void coupledScaleToOne()
        {
            double eps = 1.0e-4;
            double maxFront = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
            double maxRear = Math.max(Math.abs(backLeft), Math.abs(backRight));
            double max = Math.max(Math.abs(maxFront), Math.abs(maxRear));

            if (max > eps)
            {
                frontLeft /= max;
                frontRight /= max;
                backLeft /= max;
                backRight /= max;
            }
        }
    }

    /**
     * Clamps the motor powers while maintaining power ratios.
     * @param powers The motor powers to clamp.
     */
    private static void clampPowers(List<Double> powers) {
      double minPower = Collections.min(powers);
      double maxPower = Collections.max(powers);
      double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

      if (maxMag > 1.0) {
        for (int i = 0; i < powers.size(); i++) {
          powers.set(i, powers.get(i) / maxMag);
        }
      }
    }

    /**
     * Calculates mecanum Wheels power using simplistic calculations.
     * @param leftStickX Unmodified Gamepad leftStickX inputs.
     * @param leftStickY Unmodified Gamepad leftStickY inputs.
     * @param rightStickX Unmodified Gamepad rightStickX inputs.
     * @param rightStickY Unmodified Gamepad rightStickY inputs.
     * @return Wheels object with calculated drive powers.
     */
    public static Wheels simpleJoystickToWheels(double leftStickX, double leftStickY,
                                                double rightStickX, double rightStickY) {
        double forward = -leftStickY;
        double leftStrafe = -leftStickX;
        double rotateCCW = -rightStickX;

        double frontLeft = forward - leftStrafe - rotateCCW;
        double frontRight = forward + leftStrafe + rotateCCW;
        double backLeft = forward + leftStrafe - rotateCCW;
        double backRight = forward - leftStrafe + rotateCCW;

        return new Wheels(frontLeft, frontRight,
                backLeft, backRight);
    }
}