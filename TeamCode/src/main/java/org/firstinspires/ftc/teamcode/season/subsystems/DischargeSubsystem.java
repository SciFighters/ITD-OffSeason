package org.firstinspires.ftc.teamcode.season.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DischargeSubsystem extends SubsystemBase {
    private final DcMotorEx lowerMotor, upperMotor;
    private Servo gearBoxServo, clawServo;
    double servoDischargePos = 0, servoClimbPos = 1;
    final double clawServoHoldPos = 0.38, clawServoReleasePos = 0.58, clawServoHalfReleasePos = 0.45;
    private final TouchSensor touchSensor;
    MultipleTelemetry telemetry;
    public double timeUp = 0;

    public final double climbRatio = 9;
    public final double dischargeRatio = 15 / 11.0;
    double gearBoxRatio = dischargeRatio;

    private double liftPosInCM;
    private double lastMotorTicks;
    private double targetPosInTicks = -1;

    public final double maxLiftPos = 1810;
    public double minLiftPos = 20;
    public final double minClimbLiftPos = 170;

    public final int highChamberHeight = 720;//880
    public final int lowChamberHeight = 350;
    public final int chamberReleaseDeltaSlides = 100;

    public final int highBasketHeight = 1635;
    public final int lowBasketHeight = 500;
    public final int BasketReleaseDeltaDrive = 0;

    public final int manualTicksPerSecond = 1200;
    public final double slidesSpeed = 1;
    public final double slidesHalfSpeed = 0.675;
    public final double slidesLowSpeed = 0.5;//0.7


    public DischargeSubsystem(HardwareMap hardwareMap, MultipleTelemetry telemetry) {
        lowerMotor = hardwareMap.get(DcMotorEx.class, "lower");
        upperMotor = hardwareMap.get(DcMotorEx.class, "upper");
//        gearBoxServo = hardwareMap.servo.get("gearBoxServo");
        clawServo = hardwareMap.servo.get("clawServo");
        touchSensor = hardwareMap.get(TouchSensor.class, "dischargeTs");
        lowerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        gearBoxServo.setPosition(servoDischargePos);
        resetEncoders();
        lowerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;
    }

    //fix: use gearbox ticks, not 1600
    public void setPower(double power) {
        setRawPower(calcPowerValue(power));
    }

    public void setRawPower(double power) {
        lowerMotor.setPower(power);
        upperMotor.setPower(power);
    }

    public double calcPowerValue(double power) {
        double currentPosInCM = getLiftPosInCM();
        if (currentPosInCM <= (gearBoxRatio == 1 ? minLiftPos : minClimbLiftPos)) {
            return (Range.clip(power, 0, 1));
        } else if (currentPosInCM >= maxLiftPos) {
            return ((Range.clip(power, -1, 0)));
        } else {
            return (power);
        }
    }

    public void climbMode() {
//        gearBoxServo.setPosition(servoClimbPos);
//        gearBoxRatio = climbRatio;
//        getLiftPosInCM();
    }

    public void dischargeMode() {
//        gearBoxServo.setPosition(servoDischargePos);
//        gearBoxRatio = dischargeRatio;
//        getLiftPosInCM();
    }

    public double getGearBoxRatio() {
        return gearBoxRatio;
    }

    public void resetEncoders() {
        lowerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftPosInCM = 0;
        lastMotorTicks = 0;
        runWithoutEncoders();
    }

    public void goToTarget() {
        lowerMotor.setTargetPosition((int) targetPosInTicks);
        upperMotor.setTargetPosition((int) targetPosInTicks);
    }

    public int getPosition() {
        return upperMotor.getCurrentPosition();
    }

    public int getPosition2() {
        return lowerMotor.getCurrentPosition();
    }

    public void holdSample() {
        clawServo.setPosition(clawServoHoldPos);
    }

    public void releaseSample() {
        clawServo.setPosition(clawServoReleasePos);
    }

    public void halfReleaseSample() {
        clawServo.setPosition(clawServoHalfReleasePos);
    }

    public void testClaw(double clawPos) {
        clawServo.setPosition(clawPos);
    }

    public double getClawServoPosition() {
        return clawServo.getPosition();
    }

    public void runWithoutEncoders() {
        lowerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runWithEncoders() {
        lowerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void runToPosition() {
        lowerMotor.setPower(slidesSpeed);
        upperMotor.setPower(slidesSpeed);
        lowerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        upperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getLiftPosInCM() {
        int ct = getPosition();
        liftPosInCM += (ct - lastMotorTicks) / gearBoxRatio;
        lastMotorTicks = ct;
        return liftPosInCM;
    }

    public void setTargetPosInTicks(double newTargetPosInCM) {
        newTargetPosInCM = Range.clip(newTargetPosInCM, minLiftPos, maxLiftPos);
        this.targetPosInTicks = (int) (getPosition() + (newTargetPosInCM - getLiftPosInCM()) * gearBoxRatio);
    }

    public double getTargetPosInTicks() {
        return targetPosInTicks;
    }


    public String getMode() {
        return lowerMotor.getMode().toString();
    }

    public void changeTargetPos(double change) {
        setTargetPosInTicks(getLiftPosInCM() + change);
    }

    public boolean isHome() {
        return touchSensor.isPressed();
    }

    public PIDFCoefficients getPIDFCoefficients() {
        return lowerMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getCurrent() {
        return (lowerMotor.getCurrent(CurrentUnit.AMPS) + upperMotor.getCurrent(CurrentUnit.AMPS)) / 2;
    }

    public int getAveragePosition() {
        return (int) ((getPosition() + getPosition2()) / 2);
    }
}
