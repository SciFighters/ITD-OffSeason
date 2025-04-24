package org.firstinspires.ftc.teamcode.season.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.season.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.season.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.MecanumDrive;
import org.opencv.core.Point;

@Autonomous
public class BasketOnlyOld extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limelightSubsystem;

    @Override
    public void initialize() {
        limelightSubsystem = new LimelightSubsystem(hardwareMap, multipleTelemetry);

        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(0.8, 0.22), 180, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem, limelightSubsystem);
        mecanumDrive.setHeading(0);
//        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
        while (opModeInInit()) {
            super.run();
        }

        schedule(new SequentialCommandGroup(

                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight),
                new WaitCommand(700),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.31, 0.27, 225, 0.03, 0.8)
                                /*new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.31, 0.31, 225, 0.03, 0.8)*/),
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1800)),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),


                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.605, 0.625, 180, 0.015, 0.75),
                                new WaitCommand(100),
                                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem)),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem))),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                                new ParallelCommandGroup(
                                        new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight),
                                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1800))

                        ),
                        new SequentialCommandGroup(new WaitCommand(1200),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.4, 0.45, 225, 0.03, 1)
                        )

                ),

                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.34, 0.285, 225, 0.03, 1.2),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.355, 0.625, 180, 0.015, 0.75),
                                new WaitCommand(100),
                                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem)),
                        new SequentialCommandGroup(new WaitCommand(800), new DischargeCommands.GoHomeCmd(dischargeSubsystem))),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
//                                new ParallelCommandGroup(
                                new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight)),
//                   בםגק שכאקר צקקאופ 'ןאי                      new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1200))),
                        new SequentialCommandGroup(
                                new WaitCommand(1200),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.4, 0.45, 225, 0.03, 1))
                ),

                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.34, 0.285, 225, 0.03, 1.2),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new ParallelCommandGroup(
                        new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)),
                        new SequentialCommandGroup(
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.23, 0.8, 180, 0.03, 1.2),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.4, 0.8, 180, 0.03, 1.2)
                        )
                        ,
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        ),
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1800)
                ),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.285, 0.625, 180, 0.015, 0.75),
                new WaitCommand(100),
                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
                                new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1200),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.4, 0.45, 225, 0.03, 1)
                        )

                ),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.34, 0.285, 225, 0.03, 1.2),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new ParallelCommandGroup(
                        new InstantCommand(() -> mecanumDrive.setMoverServo(0)),
                        new SequentialCommandGroup(
                                new MecanumCommands.TwoSpeedsGotoCmd(telemetry, mecanumDrive, 0.8, 1.6, 270, 0.15, 0.8, 0.65, 0.2),
                                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.2, 1.8, 270, 0.1, 2)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(1500),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        )
                )


//                new ParallelCommandGroup(
//                    new SequentialCommandGroup(
//                                new WaitCommand(800),
//                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)),
//                    new SequentialCommandGroup(
//                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.6, 0.285, 270, 0.1,1),
//                        new MecanumCommands.GotoCmd(telemetry,mecanumDrive,0.6,1.5,270,0.1,0.6))
//                ),
//                new MecanumCommands.GotoCmd(telemetry,mecanumDrive,1.21,1.5,270,0.03,0.7),
//                new LimelightCommands.LimelightIntake(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive),
//                new ParallelCommandGroup(
//                    new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.6, 1.5, 270, 0.2,1),
//                    new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight)),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.34, 0.285, 225, 0.1, 1),
//                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem)
        ));


    }

    @Override
    public void run() {
        AutoUtils.savePosition(mecanumDrive);
        super.run();
        multipleTelemetry.addData("posm1", intakeSubsystem.getMotorPosition());
        multipleTelemetry.addData("posm2", intakeSubsystem.getMotor2Position());
        multipleTelemetry.addData("limelight x", limelightSubsystem.getXDistance());
        // multipleTelemetry.addData("lx", limelightSubsystem.getXDistance());
        // multipleTelemetry.addData("yx", limelightSubsystem.getYDistance());

        //multipleTelemetry.addData("currentIntake", intakeSubsystem.getCurrent());
        //multipleTelemetry.addData("isTouching", dischargeSubsystem.isHome());
        //multipleTelemetry.addData("discharge current command", dischargeSubsystem.getCurrentCommand().getName());
        //multipleTelemetry.addData("intake current command", intakeSubsystem.getCurrentCommand().getName());
        //multipleTelemetry.addData("current", dischargeSubsystem.getCurrent());
    }
}