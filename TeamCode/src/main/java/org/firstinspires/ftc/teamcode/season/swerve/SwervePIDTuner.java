package org.firstinspires.ftc.teamcode.season.swerve;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Range;

import java.util.Random;

@TeleOp(name = "Swerve PID Test")
public class SwervePIDTuner extends LinearOpMode {

    private CRServo swerveServo;
    private AnalogInput encoder;

    private double kp = 0.003;
    private double ki = 0.0;
    private double kd = 0.02;

    double min = 0.007, max = 3.277;
    private double kChange = 0.00002;
    double minPower = 0.02;

    private double targetAngle = 0.0;
    private double lastError = 0.0;
    private double integral = 0.0;
    double deltaTime = 0;
    double temp;

    double lastTime;
    double derivative;
    double error;
    double power;

    private final long TARGET_INTERVAL = 3500;
    private long lastTargetChangeTime = 0;
    Random random = new Random();

    @Override
    public void runOpMode() throws InterruptedException {
        swerveServo = hardwareMap.get(CRServo.class, "fl_servo");
        encoder = hardwareMap.analogInput.get("fl_encoder");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {

            long currentTime = System.currentTimeMillis();

            double currentAngle = 0;
            double deltaTime = 0;
            if (lastTime != 0) {
                currentAngle = getServoAngle();
                error = calcDeltaAngle(targetAngle, currentAngle);
                deltaTime = currentTime - lastTime;
                integral += (error * deltaTime);
                derivative = (error - lastError) / deltaTime;

                power = kp * error + ki * integral + kd * derivative;

                if (power >= 0)
                    setServoPower(Range.clip(power, minPower, 1));
                else
                    setServoPower(Range.clip(power, -1, -minPower));
            }

            lastError = error;
            lastTime = currentTime;

//            // Adjust PID coefficients with dPad and X/A/B
//
//            // Move target angle by 90 degrees every few seconds
            if (System.currentTimeMillis() - lastTargetChangeTime > TARGET_INTERVAL) {
                targetAngle += random.nextInt(280) - 140;
                targetAngle %= 360; // Keep within 0-360
                lastTargetChangeTime = System.currentTimeMillis();
            }
//
//            // Calculate PID
//            double currentAngle = getServoAngle();
//            deltaTime = System.currentTimeMillis() - lastPIDTime;
//            if (deltaTime >= 20) {
//                temp = deltaTime;
//                lastPIDTime = System.currentTimeMillis();
//                error = calcDeltaAngle(targetAngle, currentAngle);
//                integral += error;
//                derivative = (error - lastError) / (deltaTime);
//
//
//                if (gamepad1.dpad_up) {
//                    if (gamepad1.x) kp += kChange * 0.75;
//                    else if (gamepad1.a) ki += kChange * 0.25;
//                    else if (gamepad1.b) kd += kChange * 5;
//                } else if (gamepad1.dpad_down) {
//                    if (gamepad1.x) kp -= kChange * 0.75;
//                    else if (gamepad1.a) ki -= kChange * 0.25;
//                    else if (gamepad1.b) kd -= kChange * 5;
//                }
//            }
//            //if (error < 20)
//            output = kp * error + ki * integral + kd * derivative;
//            //else
//            //    output = kp * error + kd * derivative;
//            if (output >= 0)
//                setServoPower(Range.clip(output, minPower, 1));
//            else
//                setServoPower(Range.clip(output, -1, -minPower));
//
//            lastError = error;

            // Send telemetry data to dashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Angle", targetAngle);
            packet.put("Current Angle", currentAngle);
            packet.put("Error", error);
            packet.put("power", power);
            packet.put("KP", kp);
            packet.put("KI", ki);
            packet.put("KD", kd);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Target Angle", targetAngle);
//            telemetry.addData("Current Angle", currentAngle);
            telemetry.addData("Error", error);
            telemetry.addData("power", power);
            telemetry.addData("KP", kp);
            telemetry.addData("KI", ki);
            telemetry.addData("KD", kd);
            telemetry.addData("delta time", temp);
            telemetry.update();
        }
    }

    private double getServoAngle() {
        double v = getEncoderVoltage();
        return ((v - min) / (max - min)) * 360;
    }

    double getEncoderVoltage() {
        return encoder.getVoltage();
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

    private void setServoPower(double power) {
        swerveServo.setPower(-power);
    }
}
