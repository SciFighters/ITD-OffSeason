package org.firstinspires.ftc.teamcode.season.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.subsystems.Pipelines;

@Config
@TeleOp(group = "limelight")
public class LimeLightOpMode extends LinearOpMode {
    public static int pipeLine = 5;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Ensure telemetry is transmitted at regular intervals
        telemetry.setMsTransmissionInterval(11);

        // Initialize Limelight camera and ensure it's started
//        limelight.shutdown(); // Stop the Limelight to reset it
        // Start it again for the current run
        limelight.pipelineSwitch(7);
        limelight.start();
//        limelight.pipelineSwitch(Pipelines.BLUE.PIPELINE); // Switch to the blue pipeline
        sleep(100); // Give the Limelight time to switch pipeline

        waitForStart(); // Wait for the start of the teleop period

        while (opModeIsActive()) {

            sleep(100); // Give a small delay after each pipeline switch if switching frequently

            LLResult result = limelight.getLatestResult();

            if (result != null) {
                multipleTelemetry.addData("0", result.getPythonOutput()[0]);
                multipleTelemetry.addData("1", result.getPythonOutput()[1]);
                multipleTelemetry.addData("2", result.getPythonOutput()[2]);
                multipleTelemetry.addData("3", result.getPythonOutput()[3]);
                multipleTelemetry.addData("4", result.getPythonOutput()[4]);
                multipleTelemetry.addData("valid result", result.isValid());
                multipleTelemetry.addData("limelight connected", limelight.isConnected());
                multipleTelemetry.update();
            }

            dashboardTelemetry.update();
            telemetry.update(); // Update robot's telemetry
        }
    }
}
