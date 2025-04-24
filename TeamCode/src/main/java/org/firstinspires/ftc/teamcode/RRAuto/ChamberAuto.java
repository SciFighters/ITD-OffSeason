package org.firstinspires.ftc.teamcode.RRAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Actions.SeasonActions;
import org.firstinspires.ftc.teamcode.Actions.SeasonActions.*;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;

@Autonomous
public class ChamberAuto extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);


    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        Pose2d startPos = new Pose2d(new Vector2d(0, -64), 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPos);

        TrajectoryActionBuilder moveToChamber = drive.actionBuilder(startPos)
                .setTangent(Math.PI / 2)
                .lineToYConstantHeading(-34);
        TrajectoryActionBuilder moveSamplesToObs = drive.actionBuilder(drive.localizer.getPose())

                .setTangent(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(12, -48), 0)
                .splineToSplineHeading(new Pose2d(32, -28, Math.PI / 2), Math.PI / 2)
                .splineToConstantHeading(new Vector2d(41, -19), 0)

                .splineToConstantHeading(new Vector2d(45, -48), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(52, -20), 0)
                .splineToConstantHeading(new Vector2d(56, -48), -Math.PI / 2)
                .setTangent(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(60, -20), 0)
                .splineToConstantHeading(new Vector2d(64, -48), -Math.PI / 2);
        TrajectoryActionBuilder moveToCyclePosition = drive.actionBuilder(drive.localizer.getPose())

                .setTangent(Math.PI)
                .splineToLinearHeading(new Pose2d(26, -52, Math.PI / 4), Math.PI);
        TrajectoryActionBuilder toChamberCycle = drive.actionBuilder(drive.localizer.getPose())

                .setTangent(Math.PI * 3 / 4)
                .splineToLinearHeading(new Pose2d(0, -36,0), Math.PI / 2);

        TrajectoryActionBuilder fromChamberCycle = drive.actionBuilder(drive.localizer.getPose())
                .setTangent(-Math.PI / 2)
                .splineTo(new Vector2d(26, -52), -Math.PI / 4);


//                .setTangent(Math.PI*3/4)
//                .splineTo(new Vector2d(0,-36),Math.PI/2)
//                .setTangent(-Math.PI/2)
//                .splineTo(new Vector2d(26,-52),-Math.PI/4)
//
//                .setTangent(Math.PI*3/4)
//                .splineTo(new Vector2d(0,-36),Math.PI/2)
//                .setTangent(-Math.PI/2)
//                .splineTo(new Vector2d(26,-52),-Math.PI/4)
//
//                .setTangent(Math.PI*3/4)
//                .splineTo(new Vector2d(0,-36),Math.PI/2)
//                .setTangent(-Math.PI/2)
//                .splineTo(new Vector2d(26,-52),-Math.PI/4);
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(new ParallelAction(
                new MotorControl(dischargeSubsystem, () -> 0.0, false, telemetry),
                new SequentialAction(
                        new ParallelAction(
                                moveToChamber.build(),
                                new GoToTargetWait(dischargeSubsystem,dischargeSubsystem.highChamberHeight)
                        ),
                        SeasonActions.chamberDischarge(dischargeSubsystem,telemetry),
                        new SleepAction(1),
                        new ParallelAction(
                                moveSamplesToObs.build(),
                                new GoHome(dischargeSubsystem)
                        ),
                        new ParallelAction(
                                moveToCyclePosition.build(),
                                new SequentialAction(
                                        new SleepAction(0.2),
                                        SeasonActions.slideUntil(intakeSubsystem,500,0.7,true)
                                )
                        ),
                        SeasonActions.startIntake(intakeSubsystem,1500),
                        SeasonActions.SampleSubmIntakeCmd(intakeSubsystem),
                        new ParallelAction(
                                toChamberCycle.build(),
                                new SequentialAction(
                                        SeasonActions.Transfer(intakeSubsystem,dischargeSubsystem),
                                        new GoToTargetWait(dischargeSubsystem,dischargeSubsystem.highChamberHeight)
                                )

                        )
                )



        ));


    }
}
