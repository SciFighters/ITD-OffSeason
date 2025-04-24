package org.firstinspires.ftc.teamcode.season.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.DanEg.SetPowerCommand;
import org.firstinspires.ftc.teamcode.season.DanEg.SampleSubsystem;

@TeleOp
public class WhenPressedTest extends CommandOpMode {
    GamepadEx systemGamepad;
    private SampleSubsystem sampleSubsystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    @Override
    public void initialize() {
        systemGamepad = new GamepadEx(gamepad2);
        sampleSubsystem = new SampleSubsystem(hardwareMap, "motor");
        register(sampleSubsystem);
//        schedule(new SetPowerCommand(sampleSubsystem,0.5),
//                new DischargeGotoCmd(sampleSubsystem,400,10,telemetry));
        Button a = new GamepadButton(systemGamepad, GamepadKeys.Button.A);
        a.whenPressed(new SetPowerCommand(sampleSubsystem, 0.5));
        Button b = new GamepadButton(systemGamepad, GamepadKeys.Button.B);
        b.whenPressed(new SetPowerCommand(sampleSubsystem, -0.5));
        Button x = new GamepadButton(systemGamepad, GamepadKeys.Button.X);
        x.whenPressed(new SetPowerCommand(sampleSubsystem, 0));
    }

    @Override
    public void run() {
//        dischargeSubsystem.setPosition(400);
        super.run();
    }
}
