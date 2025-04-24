package org.firstinspires.ftc.teamcode.season.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ClimbSubsystem extends SubsystemBase {
    private CRServo rWinchServo, lWinchServo;
    double rLastPower = 0;
    double lLastPower = 0;

    public ClimbSubsystem(HardwareMap hardwareMap) {
        rWinchServo = hardwareMap.crservo.get("rWinchServo");
        lWinchServo = hardwareMap.crservo.get("lWinchServo");
    }


    public void setRServoPower(double power) {
        rLastPower = power;
        rWinchServo.setPower(power);
    }

    public void setLServoPower(double power) {
        lLastPower = power;
        lWinchServo.setPower(power);
    }

    public double getlLastPower() {
        return lLastPower;
    }

    public double getrLastPower() {
        return rLastPower;
    }
}
