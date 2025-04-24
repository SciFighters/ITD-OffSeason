package org.firstinspires.ftc.teamcode.season.gandalf;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class ArmClass {
    private DcMotor armMotor;
    private HOB robot;
    private int minArmPos, maxArmPos;
    private double previousArmPosition = 0;
    public boolean isReturnRequested = false;

    public ArmClass(DcMotor armMotor, HOB robot) {
        this.armMotor = armMotor;
        this.robot = robot;
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        minArmPos = 0;
        maxArmPos = -100000;
    }

    public void move(double leftTrigger, double rightTrigger) {
        robot.telemetry.addData("current arm pos", armMotor.getCurrentPosition());

        if (Math.abs(rightTrigger - leftTrigger) <= 0.05) {
            if (isReturnRequested) {
                armMotor.setPower(-armMotor.getCurrentPosition() / 500.0);
            } else {
                angleCorrection();
            }
        } else {
            isReturnRequested = false;
            if (armMotor.getCurrentPosition() >= 0) {
                armMotor.setPower(Range.clip((leftTrigger - rightTrigger) * 0.5, -1, 0));
            } else {
                armMotor.setPower((leftTrigger - rightTrigger) * 0.5);
            }
        }
        previousArmPosition = armMotor.getCurrentPosition();
    }

    private void angleCorrection() {
        armMotor.setPower((previousArmPosition - armMotor.getCurrentPosition()) * 0.03);
    }

}
