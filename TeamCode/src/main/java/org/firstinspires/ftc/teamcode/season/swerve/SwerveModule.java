package org.firstinspires.ftc.teamcode.season.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.season.utils.Utils;

public class SwerveModule {
    public SteeringServo servo;
    DcMotor motor;
    double motorPower = 0;
    double isMotorFlipped = 1;
    double headingOffset;
    double lastPos;
    private double motorsCounterServoPower = 0;
    final double motorsCounterModifier = 0.0;

    public SwerveModule(DcMotor motor, CRServo servo, AnalogInput encoder, double headingOffset) {
        this.motor = motor;
        this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.servo = new SteeringServo(servo, encoder, headingOffset);
        this.headingOffset = headingOffset;
        lastPos = getPosition();

    }

    public void zeroHeading() {
        isMotorFlipped = 1;
        servo.zeroHeading();
    }

    public void setHeadingWithAngle(double angle) {
        double delta = servo.calcDeltaAngle(angle, servo.getCurrentAngle());
        if (Math.abs(delta) > 90) {
            angle = (angle + 180) % 360;
            isMotorFlipped = -1;
        } else {
            isMotorFlipped = 1;
        }
        servo.setTargetAngle(angle);
    }

    public void setHeading(double angle, boolean angleOptimization) {
        if (angleOptimization) {
            angle = (angle + 180) % 360;
            isMotorFlipped = -1;
        } else {
            isMotorFlipped = 1;
        }
        servo.setTargetAngle(angle);
    }

    public double getDeltaAngle(double angle) {
        return Math.abs(Utils.calcDeltaAngle(angle, servo.getCurrentAngle()));
    }

    public double getAngle() {
        return servo.getRawAngle();
    }

    public double getAngleError() {
        return Utils.calcDeltaAngle(servo.targetAngle, servo.currentAngle);
    }

    public void setServoPower(double power) {
        servo.setPower(power);
    }

    public double getCurrentHeading() {
        return servo.getCurrentAngle();
    }

    public double getTargetHeading() {
        return servo.getTargetAngle();
    }

    public void setPower(double power) {
        motorPower = power;
    }

    public void updateMotor() {
        motor.setPower(motorPower * isMotorFlipped + motorsCounterServoPower);
    }

    //* Math.cos(Math.toRadians(getAngleError()))
    public double getPower() {
        return motor.getPower();
    }

    public double getPosition() {
        return (double) motor.getCurrentPosition();
    }

    public double getPositionDifference() {
        double pos = getPosition();
        double angleDiff = pos - lastPos;
        lastPos = pos;
        return angleDiff;
    }

    private void calcMotorsCounterServoPower(double servoPower) {
        motorsCounterServoPower = servoPower * motorsCounterModifier;
    }

    public void update() {
        servo.setSpeed(motorPower);
        servo.update();
        calcMotorsCounterServoPower(servo.power);
        updateMotor();
    }
}
