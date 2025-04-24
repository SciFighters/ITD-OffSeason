package org.firstinspires.ftc.teamcode.season.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.MecanumDrive;
import org.opencv.core.Point;

//@Disabled
@Autonomous(group = "park")
public class AutoPark extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(2.4, 0.2), 0, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem);
        AutoUtils.initCommands(this,dischargeSubsystem,intakeSubsystem);



        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
        }


        schedule(new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.4, 0.2, 0, 0.05, 0.5, true));//
    }
}
