package org.firstinspires.ftc.teamcode.season.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.MecanumDrive;
import org.opencv.core.Point;

@Autonomous
public class AutonomousTest extends CommandOpMode {
    private MecanumDrive mecanumDrive;
    GamepadEx gamepad;
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    MultipleTelemetry multipleTelemetry;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        GamepadEx gamepad = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(1.8, 0.2), 0, this);

        register(mecanumDrive, dischargeSubsystem, intakeSubsystem);

//        schedule(new SequentialCommandGroup(
//                new GotoCmd(telemetry, swerveDrive, -0.66,0.0,0.0, 0.01,0.2),
//                new GotoCmd(telemetry, swerveDrive, 0,0.0,0.0, 0.01,0.2)));
//        schedule(new GotoCmd(telemetry, swerveDrive, 0.67, 0.21, 0, 0.03, 0.2));
        schedule(
                new MecanumCommands.SplineGotoCmd(mecanumDrive,
                        new Point(1.8, 0.2), new Point(0.3, 0.2), new Point(0.3, 1.5),
                        0.5, 0.03)
        );

    }


}
