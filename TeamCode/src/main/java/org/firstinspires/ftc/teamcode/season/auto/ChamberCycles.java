package org.firstinspires.ftc.teamcode.season.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.season.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.season.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.MecanumDrive;
import org.opencv.core.Point;

@Autonomous(group = "chamber")
public class ChamberCycles extends CommandOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    Point pos = new Point(0, 0);

    @Override
    public void initialize() {
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(1.8, 0.88), 0, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem);

//                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
//        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
//        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
//        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
//        schedule(new  IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true));
//        while (opModeInInit()) {
//            super.run();
//        }
//

        schedule(new SequentialCommandGroup(
//                chamberDischarge(1.8, 2),
                hpIntake(),
                chamberDischarge(1.8, 1),
                hpIntake(),
                chamberDischarge(1.8, 1),
                hpIntake(),
                chamberDischarge(1.8, 1),
                hpIntake(),
                chamberDischarge(1.8, 1)
        ));
    }

    public CommandBase chamberDischarge(double dischargeX, int transferMode) {

        if (transferMode == 0) {
            return new SequentialCommandGroup(

                    new ParallelDeadlineGroup(
                            new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                            goTo(dischargeX, 0.88, 0, 0.05, 01.5)),

                    dischargeAction(dischargeX)

            );
        }

        if (transferMode == 1) {
            return new SequentialCommandGroup(

                    new ParallelCommandGroup(
//                            new SequentialCommandGroup(
//                                    new IntakeCommands.AutoTransfer(intakeSubsystem, dischargeSubsystem),
//                                    new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),
                            new SequentialCommandGroup(
                                    wait(700),
                                    goTo(dischargeX, 0.86, -10, 0.06, 1.3))),

//                    new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                    dischargeAction(dischargeX)

            );
        } else {
            return new SequentialCommandGroup(

                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.AutoTransfer(intakeSubsystem, dischargeSubsystem),
                                    new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),
                            new SequentialCommandGroup(
                                    wait(500),
                                    goTo(dischargeX, 0.76, 0, 0.08, 1.3),
                                    moverGoGo(0.08),
                                    goTo(dischargeX, 0.86, 0, 0.05, 1.3))),


                    new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                    dischargeAction(dischargeX)
            );
        }
    }

    public CommandBase dischargeAction(double dischargeX) {//Todo: add intake/discharge back
        return new ParallelDeadlineGroup(
                new DischargeCommands.SequentialRaceWrapper( // Custom wrapper for treating the sequence as a deadline
                        //new MecanumCommands.chamberWait(mecanumDrive),
                        new WaitCommand(600)/*,*///Todo: make time the minimum possible
//                        new DischargeCommands.AutoChamberDischargeCmd(dischargeSubsystem, telemetry)
                ),
                goToConstVelocity(dischargeX, 1.5, 0, 0.03, 0.5).withTimeout(5000)
        );
    }

    public CommandBase hpIntake() {//Todo: add intake/discharge back
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
//                        new DischargeCommands.GoHomeCmd(dischargeSubsystem),
//                        goTo(2.5, 0.55, -45, 0.03, 1),
                        new SequentialCommandGroup(
                                goTo(2.5, 0.55, -45, 0.03, 1)
                        )/*,*/
//                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1650)),
//                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem)

                ));
    }

    private CommandBase goToConstVelocity(double x, double y,
                                          double wantedAngle, double sensitivity, double speed) {
        return new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed);
//        return new MecanumCommands.GotoCmd()

    }

//    private CommandBase goPastConstVelocity(double x, double y,
//                                            double wantedAngle, double sensitivity, double speed) {
//        return new MecanumCommands.ConstantVelocityGoPastCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed);
////        return new MecanumCommands.GotoCmd()
//
//    }

    private CommandBase goToTwoSpeeds(double x, double y,
                                      double wantedAngle, double sensitivity, double speed1, double speed2, double swap) {
        return new MecanumCommands.TwoSpeedsGotoCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed1, speed2, swap);

    }


    private CommandBase goTo(double x, double y, double angle, double sensitivity, double speed) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, x, y, angle, sensitivity, speed);
    }

    private CommandBase wait(int mills) {
        return new WaitCommand(mills);
    }

    private CommandBase moverGoGo(double moveTo) {
        return new MecanumCommands.MoverServoCmd(mecanumDrive, moveTo);
    }

    @Override
    public void run() {
        AutoUtils.savePosition(mecanumDrive);
        super.run();
        telemetry.addData("x", mecanumDrive.getPosition().x);
        telemetry.addData("y", mecanumDrive.getPosition().y);
        telemetry.addData("intake pos", intakeSubsystem.getMotorPosition());
        telemetry.addData("elevator pos", dischargeSubsystem.getPosition());
        telemetry.update();
    }
}
