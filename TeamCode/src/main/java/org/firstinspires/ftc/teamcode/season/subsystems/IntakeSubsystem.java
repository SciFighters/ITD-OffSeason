package org.firstinspires.ftc.teamcode.season.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx rMotor;
    private final DcMotorEx lMotor;
    private final Servo hServo; // claw up & down
    private final Servo rServo; // claw rotation
    private final Servo screwServo;
    MultipleTelemetry telemetry;

    int positionCorrection = 0;
    private double targetPos = -1;
    private final int maxArmLength = 2050;
    public int minSlidesPos = 10;

    public final int manualTicksPerSecond = 785;
    public final double slidesSpeed = 1;
    public final double slidesLowSpeed = 0.7;
    public final double slidesHalfSpeed = 0.8;
    public boolean end = false;
    TouchSensor leftTouch, rightTouch;
    ElapsedTime screwTimer = new ElapsedTime();
    public final double openScrewTime = 0.35;


    public void openScrew() {
        screwServo.setPosition(0);
        screwTimer.reset();
    }

    public void closeScrew() {
        screwServo.setPosition(1);
        screwTimer.reset();
    }

    public IntakeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        rMotor = hardwareMap.get(DcMotorEx.class, "leftIntakeMotor");
        lMotor = hardwareMap.get(DcMotorEx.class, "rightIntakeMotor");
        lMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hServo = hardwareMap.servo.get("heightServo");
        rServo = hardwareMap.servo.get("rotationServo");
        screwServo = hardwareMap.servo.get("screwServo");
        leftTouch = hardwareMap.touchSensor.get("leftTouch");
//        rightTouch = hardwareMap.touchSensor.get("rightTouch");
        screwTimer.reset();
        //resetEncoders();
        this.telemetry = telemetry;

    }    // make one button that extends the arm and lowers the claw while opening it

    public void setArmPower(double power) {
        if (rMotor.getCurrentPosition() <= 0 || lMotor.getCurrentPosition() <= 0) {
            rMotor.setPower(Range.clip(power, 0, 1));
            lMotor.setPower(Range.clip(power, 0, 1));
        } else if (rMotor.getCurrentPosition() >= maxArmLength || lMotor.getCurrentPosition() >= maxArmLength) {
            rMotor.setPower(Range.clip(power, -1, 0));
            lMotor.setPower(Range.clip(power, -1, 0));
        } else {
            rMotor.setPower(power);
            lMotor.setPower(power);
        }
    }

    public void setRawPower(double power) {
        rMotor.setPower(power);
        lMotor.setPower(power);
    }

    public double getTargetPos() {
        return targetPos;
    }

    public boolean isHome() {
        return leftTouch.isPressed();
    }//|| rightTouch.isPressed()

    public void armGoToTarget() {
        rMotor.setTargetPosition((int) targetPos);
        lMotor.setTargetPosition((int) targetPos);
    }

    public double getAveragePosition() {
        return ((double) rMotor.getCurrentPosition() + (double) rMotor.getCurrentPosition()) / 2 + positionCorrection;
    }

    public void setRotationServoPosition(double position) {
        rServo.setPosition(position);
    }

    public double getZServoPosition() {
        return rServo.getPosition();
    }

    public void setHServoPosition(double position) {
        hServo.setPosition(position);
    }

    public int getMotorPosition() {
        return rMotor.getCurrentPosition() + positionCorrection;
    }

    public int getMotor2Position() {
        return rMotor.getCurrentPosition() + positionCorrection;
    }

    public void setPositionCorrection(int pos) {
        positionCorrection = pos;
    }

    public double getXServoPosition() {
        return hServo.getPosition();
    }

    public void resetEncoders() {
        rMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runWithoutEncoders();
    }

    public void runWithoutEncoders() {
        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runWithEncoders() {
        lMotor.setPower(slidesSpeed);
        rMotor.setPower(slidesSpeed);
        rMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }

    public void changeTargetPos(double change) {
        targetPos += change;
        if (targetPos < minSlidesPos)
            targetPos = minSlidesPos;
        if (targetPos > maxArmLength)
            targetPos = maxArmLength;
    }

    public double getCurrent() {
        return (rMotor.getCurrent(CurrentUnit.AMPS) + rMotor.getCurrent(CurrentUnit.AMPS)) / 2;
    }

    public double getPower() {
        return (rMotor.getPower() + rMotor.getPower()) / 2;
    }


    // make second button that while pressing it it goes to half of height and pushes things
    // away and then lowers one more stage and picks up the sample
    // button 3: checks angle and returns to base position

}
