package org.firstinspires.ftc.teamcode.season.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.season.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.season.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.season.subsystems.Pipelines;
import org.opencv.core.Point;

@Autonomous(group = "chamber")
public class LimelightChamber extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limelightSubsystem;
    Point pos = new Point(0, 0);

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(1.8, 0.2), 0, this);
        limelightSubsystem = new LimelightSubsystem(hardwareMap, multipleTelemetry);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem);
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry),
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
//        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
        limelightSubsystem.setPipeline(Pipelines.BLUE);
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
        while (opModeInInit()) {
            super.run();
        }
//

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 0.955, 0, 0.05, 1),
                        new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),
                new WaitCommand(300),

                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry),
                new WaitCommand(300),

//                new InstantCommand(() -> mecanumDrive.resetPos(new Point(mecanumDrive.getPosition().x, 0.935))),
                new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, 1.8, 0.8, -90, 0.05, 0.8),
                new WaitCommand(100),
//                new InstantCommand(() -> this.pos = new Point(mecanumDrive.getPosition().x,mecanumDrive.getPosition().y)),
//                new InstantCommand(() -> mecanumDrive.resetPos(pos)),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3, 0.7, 180, 0.02, 1),
                new LimelightCommands.LimelightCompleteSubIntake(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive),
                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.lowChamberHeight),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem),

                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.3, 0.7, 180, 0.02, 1),
                new LimelightCommands.LimelightCompleteSubIntake(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive),
                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.lowChamberHeight),
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem),

                new MecanumCommands.SetRotationCmd(mecanumDrive, 0).withTimeout(500),
                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.3, 0.5, 180, 0.02, 1),
                new LimelightCommands.LimelightCompleteSubIntake(limelightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive)
        ));
//        schedule(new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 1.02, 0, 0.05, 0.9, true),
//                        new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight, telemetry)),
//                new WaitCommand(200),
//                new ParallelCommandGroup(
//                        new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)),
//                new WaitCommand(200),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, 0.8, 0, 0.14, 1),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.7, 0.8, 0, 0.1, 0.8),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.7, 1.55, 0, 0.04, 0.8),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.93, 1.55, 0, 0.05, 0.9),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.93, 0.4, 0, 0.05, 0.9, true),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3, 0.73, 0, 0.01, 1),
//                new WaitCommand(1400),
//                new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1350),
//                new WaitCommand(300),
//                new IntakeCommands.SampleIntakeCmd(intakeSubsystem),
//                new WaitCommand(300),
//
//                new ParallelCommandGroup(
//                        new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
//                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.75, 0.6, 0, 0.06, 1)
//                ),
//
//                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight + 50, telemetry),
//                new WaitCommand(900),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.65, 1.05, 0, 0.05, 0.9),
//                new ParallelCommandGroup(
//                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1200),
//                        new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.75, 0.655, 0, 0.02, 1).withTimeout(150),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3, 0.9, 0, 0.015, 0.7),
//                new IntakeCommands.SlideGotoCmd(intakeSubsystem, 1350),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3, 0.7, 0, 0.01, 0.7),
//                new WaitCommand(900),
//                new IntakeCommands.SampleIntakeCmd(intakeSubsystem),
//                new WaitCommand(200),
//                new ParallelCommandGroup(
//                        new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem),
//                        new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.9, 0.6, 0, 0.06, 1)
//                ),
//
//                new DischargeCommands.DischargeGotoCmd(dischargeSubsystem, dischargeSubsystem.highChamberHeight + 50, telemetry),
//                new WaitCommand(900),
//                new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.92, 1.05, 0, 0.05, 0.9),
//                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)));
    }

    private CommandBase goToConstVelocity(double x, double y,
                                          double wantedAngle, double sensitivity, double speed) {
        return new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed);
//        return new MecanumCommands.GotoCmd()

    }

    @Override
    public void run() {
        AutoUtils.savePosition(mecanumDrive);
        telemetry.addData("x", mecanumDrive.getPosition().x);
        telemetry.addData("y", mecanumDrive.getPosition().y);

        telemetry.update();
        super.run();
    }
}