package org.firstinspires.ftc.teamcode.season.teleop;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.season.auto.ChamberOnly;

@TeleOp
public class MoveMotor extends CommandOpMode {
    DcMotor motor;

    @Override
    public void initialize() {
        motor = hardwareMap.dcMotor.get("upper");
    }

    @Override
    public void run() {
        super.run();
        motor.setPower(-gamepad1.left_stick_y);
        telemetry.addData("power", motor.getPower());
        telemetry.addData("pos", motor.getCurrentPosition());
    }
}
