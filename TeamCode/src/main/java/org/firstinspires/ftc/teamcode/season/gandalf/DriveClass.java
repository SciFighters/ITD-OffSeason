package org.firstinspires.ftc.teamcode.season.gandalf;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveClass {
    private final DcMotor leftWheel, rightWheel;
    private double speedModifier = 0, angularSpeedModifier = 0;
    private boolean isDrivingForward = true;

    public DriveClass(DcMotor leftWheel, DcMotor rightWheel) {
        this.leftWheel = leftWheel;
        this.rightWheel = rightWheel;
        this.rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double speed, double turn, boolean leftBumper, boolean rightBumper) {
        setBoost(leftBumper, rightBumper);
        leftWheel.setPower(speed * speedModifier - turn * angularSpeedModifier);
        rightWheel.setPower(speed * speedModifier + turn * angularSpeedModifier);

    }

    private void setBoost(boolean leftBumper, boolean rightBumper) {
        if (leftBumper && rightBumper) {
            speedModifier = 0.9;
            angularSpeedModifier = 0.4;
        } else if (leftBumper || rightBumper) {
            speedModifier = 0.6;
            angularSpeedModifier = 0.4;
        } else {
            speedModifier = 0.4;
            angularSpeedModifier = 0.25;
        }
    }

    public void swapDrivingDirection() {
        isDrivingForward = !isDrivingForward;
        if (isDrivingForward) {
            rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            rightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            leftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
}
