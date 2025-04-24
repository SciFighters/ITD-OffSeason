package org.firstinspires.ftc.teamcode.season.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.utils.Utils;
import org.firstinspires.ftc.teamcode.season.swerve.SwerveModule;

@Disabled
@TeleOp(group = "tests")
public class WheelTest extends LinearOpMode {
    // bl: servo = 1, analog = 0, motor = 0
    // br: servo = 4, analog = 2,  motor = 3
    // fl: servo = 0, analog = 1, motor = 1
    // fr: servo = 3, analog = 3, motor = 2
    //0:350.64 1: 173 2:315 3: 54.83
    //br:53,bl:133.21,
    SwerveModule bl, br, fl, fr, cr;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    @Override
    public void runOpMode() {
        bl = new SwerveModule(hardwareMap.get(DcMotor.class, "bl_motor"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                hardwareMap.analogInput.get("bl_encoder"), 314.64);
        br = new SwerveModule(hardwareMap.get(DcMotor.class, "br_motor"),
                hardwareMap.get(CRServo.class, "br_servo"),
                hardwareMap.analogInput.get("br_encoder"), 54.93);
        fl = new SwerveModule(hardwareMap.get(DcMotor.class, "fl_motor"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                hardwareMap.analogInput.get("fl_encoder"), 356.5);
        fr = new SwerveModule(hardwareMap.get(DcMotor.class, "fr_motor"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                hardwareMap.analogInput.get("fr_encoder"), 346.9);
        cr = fl;
        // FLONG
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.b) {
                cr.setServoPower(0);
                cr.setPower(0);
                cr = fl;
            } else if (gamepad1.a) {
                cr.setServoPower(0);
                cr.setPower(0);
                cr = bl;
            } else if (gamepad1.y) {
                cr.setServoPower(0);
                cr.setPower(0);
                cr = fr;
            } else if (gamepad1.x) {
                cr.setServoPower(0);
                cr.setPower(0);
                cr = br;
            }
            double power = -gamepad1.left_stick_y;
            double wantedAngle = Math.toDegrees(Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y));

            power = Utils.signRoot(power);

            cr.setHeadingWithAngle(wantedAngle);
            cr.setPower(power);
//            cr.update();

            if (gamepad1.dpad_right) {
                cr.setServoPower(0.07);
            } else if (gamepad1.dpad_left) {
                cr.setServoPower(-0.07);
            } else if (gamepad1.right_stick_button) {
                cr.servo.setPower(0);
                cr.setHeadingWithAngle(0);
                cr.update();
            } else if (Math.pow(gamepad1.right_stick_y, 2) + Math.pow(gamepad1.right_stick_x, 2) <= 0.99) {
                cr.servo.setPower(0);
            } else {
                cr.servo.setPower(0);
                cr.update();
            }
            if (gamepad1.back) {
                cr.zeroHeading();
            }
            multipleTelemetry.addData("wanted angle: ", wantedAngle);
            multipleTelemetry.addData("X", gamepad1.right_stick_x);
            multipleTelemetry.addData("y", -gamepad1.right_stick_y);
            multipleTelemetry.addData("current angle: ", cr.getCurrentHeading());
            multipleTelemetry.addData("motor position: ", cr.getPosition());
            multipleTelemetry.addData("angle diff:", (wantedAngle - cr.getCurrentHeading()) % 180);
            multipleTelemetry.addData("offset", cr.servo.getAngleOffset());
            multipleTelemetry.addData("bl position", bl.getPosition());
            multipleTelemetry.addData("br position", br.getPosition());
            multipleTelemetry.addData("fl position", fl.getPosition());
            multipleTelemetry.addData("fr position", fr.getPosition());
            multipleTelemetry.update();

        }
    }
}
