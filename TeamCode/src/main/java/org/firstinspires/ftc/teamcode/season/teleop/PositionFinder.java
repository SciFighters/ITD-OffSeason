package org.firstinspires.ftc.teamcode.season.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.subsystems.MecanumDrive;
import org.opencv.core.Point;

@TeleOp
public class PositionFinder extends CommandOpMode {
    MecanumDrive mecanumDrive;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    @Override
    public void initialize() {
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(1.8, 0.2), 0, this);


    }

    @Override
    public void run() {
        super.run();
        multipleTelemetry.addData("x", mecanumDrive.getPosition().x);
        multipleTelemetry.addData("y", mecanumDrive.getPosition().y);
        multipleTelemetry.addData("angleAdjusted", mecanumDrive.getAdjustedHeading());
        multipleTelemetry.addData("angle", mecanumDrive.getHeading());
        multipleTelemetry.update();
    }

}
