package org.firstinspires.ftc.teamcode.season.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class NormalLimelight extends LinearOpMode {

    GamepadEx gamepad;
    private Limelight3A limelight;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new GamepadEx(gamepad1);
        GamepadButton A = new GamepadButton(gamepad, GamepadKeys.Button.A);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(7);
        /*
         * Starts polling for data.
         */
        limelight.start();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();


            multipleTelemetry.addData("connected", limelight.isConnected());
            multipleTelemetry.addData("time", limelight.getTimeSinceLastUpdate());
            multipleTelemetry.addData("0", result.getPythonOutput()[0]);
            multipleTelemetry.addData("1", result.getPythonOutput()[1]);
            multipleTelemetry.addData("2", result.getPythonOutput()[2]);
            multipleTelemetry.addData("3", result.getPythonOutput()[3]);
            multipleTelemetry.addData("4", result.getPythonOutput()[4]);
            multipleTelemetry.addData("5", result.getPythonOutput()[5]);
            multipleTelemetry.addData("6", result.getPythonOutput()[6]);

            multipleTelemetry.addData("running", limelight.isRunning());
            A.whenPressed(() -> limelight.captureSnapshot("test1"));
            CommandScheduler.getInstance().run();
            multipleTelemetry.update();


        }
//        limelight.pause();

    }
}
