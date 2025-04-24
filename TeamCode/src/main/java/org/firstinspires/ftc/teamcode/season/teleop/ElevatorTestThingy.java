package org.firstinspires.ftc.teamcode.season.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ElevatorTestThingy extends LinearOpMode {
    DcMotorEx motor1;
    DcMotorEx motor2;
    double position = 0;

    //0.39 close 0.74 open
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotorEx.class, "dischargeMotor1");
//        motor2 = hardwareMap.get(DcMotorEx.class, "dischargeMotor2");
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            motor1.setPower(-0.8);
//            motor2.setPower(-0.8);
            telemetry.addData("pos", motor1.getCurrentPosition());
//            telemetry.addData("pos",motor1.getCurrentPosition());
            telemetry.update();
        }
    }
}
