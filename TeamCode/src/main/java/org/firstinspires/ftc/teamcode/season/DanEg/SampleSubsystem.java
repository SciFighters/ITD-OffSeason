package org.firstinspires.ftc.teamcode.season.DanEg;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SampleSubsystem extends SubsystemBase {
    DcMotor motor;

    public SampleSubsystem(HardwareMap map, String motorName) {
        motor = map.get(DcMotor.class, motorName);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }
}
