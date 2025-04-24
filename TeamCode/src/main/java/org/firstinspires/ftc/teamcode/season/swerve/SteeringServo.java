package org.firstinspires.ftc.teamcode.season.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;


@Config
public class SteeringServo {
    public static double noiseGain = 0.035;
    public static double noiseCuttoff = 2;
    public static double w = -0.7;
    public static double p = 0.25;

    //    private final double kp = 0.00334;

//    private final double kp = 0.0022;
//    private final double ki = 0.00000052;
//    private final double kd = 0.0148;

    //private final double kp = 0.0068;
    //private final double ki = 0.00000072;
    //private final double kd = 0.068;
    public static double PIDMod = 1;
    public static double kp = 0.225;
    public static double ki = 0.3;
    public static double kd = 0.015;
    public static double ks = 0;
    public double speed = 0;
    public static double minPower = 0.015;
    //    public static double kp = 0.45;
//    public static double ki = 0.3;
//    public static double kd = 0.02;
//    public static double ks = 0;
//    public double speed = 0;
//    public static double minPower = 0.03;//0.0335
    public static double tolerance = 0;

    private double lastError = 0.0;
    public double integral = 0.0;
    private double lastTime = 0;
    public double derivative;
    public double error;


    private boolean idle = false;
    public double power = 0;
    private CRServo servo;
    private AnalogInput encoder;
    private double angleOffset;
    double currentAngle;
    double targetAngle;

    double min = 0.007, max = 3.277;


    public SteeringServo(CRServo servo, AnalogInput encoder, double headingOffset) {
        this.servo = servo;
        this.encoder = encoder;
        this.angleOffset = headingOffset;
    }

    public void setPower(double power) {
        this.power = -power;
        servo.setPower(-power);
    }

    public void setTargetAngle(double target) {
        targetAngle = target;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    double getEncoderVoltage() {
        return encoder.getVoltage();
    }

    public double getCurrentAngle() {
        double v = getEncoderVoltage();
        currentAngle = ((v - min) / (max - min)) * 360 - angleOffset;
        return currentAngle;
    }

    public double getRawAngle() {
        double v = getEncoderVoltage();
        currentAngle = ((v - min) / (max - min)) * 360;
        return currentAngle;
    }

    public void setAngleOffset(double angle) {
        angleOffset = angle;
    }

    public double getAngleOffset() {
        return angleOffset;
    }

    public void zeroHeading() {
        angleOffset = 0;
        angleOffset = getCurrentAngle();
    }

    public double calcDeltaAngle(double target, double current) {
        double delta = target - current;
        if (delta > 180) {
            delta = delta - 360;
        } else if (delta < -180) {
            delta = 360 + delta;
        }
        return delta;
    }

    public void setSpeed(double S) {
        this.speed = S;
    }

    public void update() {
        double currentTime = (double) System.currentTimeMillis() / 1000;  // seconds
        double deltaTime;
        double currentAngle = getCurrentAngle();
        error = calcDeltaAngle(getTargetAngle(), currentAngle);
        power = 0;

        double errNorm = error / 100;
        if (lastTime != 0) {
            //double errCurved =

            if (Math.abs(error) < tolerance) {
                power = 0;
                integral = 0;
            } else {

                deltaTime = currentTime - lastTime;
                integral += (errNorm * deltaTime);
                if (Math.signum(integral) != Math.signum(error)) {
                    integral = 0;
                }
                derivative = (errNorm - lastError) / deltaTime;

                power = PIDMod * kp * errNorm +
                        PIDMod * ki * integral +
                        PIDMod * kd * derivative;

                power += PIDMod * minPower * Math.signum(power);
                power = Range.clip(power, -1, 1);
            }

            setPower(power);
        }

        lastError = errNorm;
        lastTime = currentTime;


        //telemetry.addData("deltaTime", deltaTime);
        //telemetry.addData("error", error);
        //telemetry.addData("getTargetAngle", getTargetAngle());
        //telemetry.addData("power", power);
        //telemetry.addData("ki * integral", ki * integral);

        //TelemetryPacket packet = new TelemetryPacket();


        //
        //if(Math.abs(error) > noiseCuttoff) {
        //    power += Math.signum(power) * Math.random() * noiseGain;
        //}
    }


}
