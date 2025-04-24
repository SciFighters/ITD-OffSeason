package org.firstinspires.ftc.teamcode.season.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SampleSubsystem extends SubsystemBase {
    private final DcMotorEx motor;
    //    private final Servo gearBoxServo, clawServo;
    double servoDischargePos = 0, servoClimbPos = 1;
    double clawServoHoldPos = 0.1, clawServoReleasePos = 0;
    MultipleTelemetry telemetry;
    int gearBoxRatio = 1;

    public SampleSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "dischargeMotor");
//        motor2 = hardwareMap.get(DcMotorEx.class,"dischargeMotor2");
//        gearBoxServo = hardwareMap.servo.get("gearBoxServo");
//        clawServo = hardwareMap.servo.get("dischargeServo");
//        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
        resetEncoders();
    }

    //    public void setPower(double power) {
//        telemetry.addData("tiiick", motor2.getCurrentPosition());
//        telemetry.addData("gearboxRatio",gearBoxRatio);
//        telemetry.addData("servoPos",gearBoxServo.getPosition());
//
//        telemetry.update();
//        if (motor2.getCurrentPosition() <= 0) {
//            motor1.setPower((Range.clip(power, 0, 1)));
//            motor2.setPower((Range.clip(power, 0, 1)));
//        }else if (motor2.getCurrentPosition() >= 1628 * gearBoxRatio) {
//            motor1.setPower((Range.clip(power, -1, 0)));
//            motor2.setPower((Range.clip(power, -1, 0)));
//        } else {
//            motor1.setPower(power);
//            motor2.setPower(power);
//        }
//    }
    public void setPosition(int position) {
        motor.setTargetPosition(position);
        motor.setPower(0.5);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motor2.setTargetPosition(position);
    }

    public void setPower(double power) {
        motor.setPower(power);
//        motor2.setPower(power);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void climbMode() {
//        gearBoxServo.setPosition(servoClimbPos);
        gearBoxRatio = 9;
    }

    public void dischargeMode() {
//        gearBoxServo.setPosition(servoDischargePos);
        gearBoxRatio = 1;
    }

    public int getGearBoxRatio() {
        return gearBoxRatio;
    }

    public void resetEncoders() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition(0);
//        motor2.setTargetPosition(0);
        motor.setTargetPositionTolerance(10);
//        motor2.setTargetPositionTolerance(10);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

//    public void holdSample() {
//        clawServo.setPosition(clawServoHoldPos);
//    }

//    public void releaseSample() {
//        clawServo.setPosition(clawServoReleasePos);
//    }
//    public double getClawServoPos(){
//        return clawServo.getPosition();
//    }

}
