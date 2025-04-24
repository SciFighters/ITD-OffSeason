package org.firstinspires.ftc.teamcode.season.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoProgrammer extends LinearOpMode {
    Servo servo;
    double position = 0;

    //0.2 close 0.37 open
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "clawServo");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                position += 0.001;
            } else if (gamepad1.dpad_down) {
                position -= 0.001;
            }
            servo.setPosition(position);
            telemetry.addData("pos", position);
            telemetry.update();
        }
    }
}
