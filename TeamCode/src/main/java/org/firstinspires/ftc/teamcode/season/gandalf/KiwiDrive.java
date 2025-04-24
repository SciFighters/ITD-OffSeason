package org.firstinspires.ftc.teamcode.season.gandalf;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class KiwiDrive {
    private final DcMotor wheel1;
    private final DcMotor wheel2;
    private final DcMotor wheel3;
    private double moveSpeedModifier;
    private final BNO055IMU imu;
    public double wheel1Power, wheel2Power, wheel3Power;

    public KiwiDrive(DcMotor wheel1, DcMotor wheel2, DcMotor wheel3, BNO055IMU imu) {
        this.wheel1 = wheel1;
        this.wheel2 = wheel2;
        this.wheel3 = wheel3;
        this.imu = imu;
        moveSpeedModifier = 0.2;
    }

    public void drive(double xv, double yv, double av) {

        double[] cords = rotateVector(xv, yv);
        xv = cords[0];
        yv = cords[1];

        wheel1Power = moveSpeedModifier * xv - 0.3 * av;
        wheel2Power = moveSpeedModifier * (-yv * -Math.sqrt(3) / 2 + xv * -1 / 2) - 0.3 * av;
        wheel3Power = moveSpeedModifier * (-yv * Math.sqrt(3) / 2 + xv * -1 / 2) - 0.3 * av;

        normalizeWheelSpeeds(wheel1Power, wheel2Power, wheel3Power);

        wheel1.setPower(wheel1Power);
        wheel2.setPower(wheel2Power);
        wheel3.setPower(wheel3Power);
    }

    public void setMoveSpeedModifier(double moveSpeedModifier) {
        this.moveSpeedModifier = 2 * moveSpeedModifier;
    }

    public void dpadMovement(boolean down, boolean up, boolean left, boolean right, double av) {
        if (down) {
            if (left) {
                drive(-0.6, 0.6, av);
            } else if (right) {
                drive(0.6, 0.6, av);
            } else {
                drive(0, 0.6, av);
            }
        } else if (up) {
            if (left) {
                drive(-0.6, -0.6, av);
            } else if (right) {
                drive(0.6, -0.6, av);
            } else {
                drive(-0.6, 0, av);
            }
        } else if (left) {
            drive(-0.6, 0, av);
        } else if (right) {
            drive(0.6, 0, av);
        }
    }

    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation();
        return (-orientation.firstAngle) % 360;
    }

    private double[] rotateVector(double a, double b) {
        double[] cords = {0, 0};
        double alpha = Math.toRadians(-getHeading());
        cords[0] = Math.cos(alpha) * a - Math.sin(alpha) * b;
        cords[1] = Math.sin(alpha) * a + Math.cos(alpha) * b;
        return cords;
    }

    public void setBoost(boolean left, boolean right) {

        if (left || right) {
            setMoveSpeedModifier(1);
        } else {
            setMoveSpeedModifier(0.35);
        }
    }

    private void normalizeWheelSpeeds(double wheel1Speed, double wheel2Speed, double wheel3Speed) {
        double highestSpeed = Math.abs(Math.max(wheel1Speed, Math.max(wheel2Speed, wheel3Speed)));
        if (highestSpeed > 1) {
            wheel1Power /= highestSpeed;
            wheel2Power /= highestSpeed;
            wheel3Power /= highestSpeed;
        }

    }
}