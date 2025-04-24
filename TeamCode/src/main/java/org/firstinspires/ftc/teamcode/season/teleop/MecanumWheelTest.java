package org.firstinspires.ftc.teamcode.season.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumWheelTest extends CommandOpMode {
    DcMotor fl, fr, bl, br, cr;

    @Override
    public void initialize() {
        fl = hardwareMap.get(DcMotor.class, "fl_motor");
        fr = hardwareMap.get(DcMotor.class, "fr_motor");
        br = hardwareMap.get(DcMotor.class, "br_motor");
        bl = hardwareMap.get(DcMotor.class, "bl_motor");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        cr = fl;
    }

    @Override
    public void run() {
        super.run();
        if (gamepad1.y) {
            cr.setPower(0);
            cr = fl;
        }
        if (gamepad1.b) {
            cr.setPower(0);
            cr = bl;
        }
        if (gamepad1.a) {
            cr.setPower(0);
            cr = br;
        }
        if (gamepad1.x) {
            cr.setPower(0);
            cr = fr;
        }
        cr.setPower(gamepad1.left_stick_x);
        telemetry.addData("pos", cr.getCurrentPosition());
        telemetry.addData("", cr.getDirection());

//        if (0 < Math.random()&& Math.random() < 0.0001){
//            try {
//                throw new Exception("skill issue");
//            } catch (Exception e) {
//                throw new RuntimeException("skill issue");
//            }
//        }
    }

}
