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
public class ChamberOnly extends CommandOpMode {
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
        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(1.8, 0.2), 0, this);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem);

//                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));
        AutoUtils.initCommands(this, dischargeSubsystem, intakeSubsystem);
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
//        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, true, telemetry));
//        schedule(new  IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true));
        while (opModeInInit()) {
            super.run();
        }
//

        schedule(new SequentialCommandGroup(

                chamberDischarge(1.78, 0),

                new ParallelCommandGroup(

                        new SequentialCommandGroup(

                                goToTwoSpeeds(1.84, 0.82, 0, 0.05, 0.65, 0.65, 0),

                                goToTwoSpeeds(2.5, 0.82, -90, 0.06, 0.8, 0.6, 0.15),

                                goToTwoSpeeds(2.92, 1.4, -90, 0.05, 0.65, 0.5, 0.15), //behind sample
                                goToTwoSpeeds(2.94, 0.7, -90, 0.08, 0.8, 0.65, 0.2), //observation

                                goToTwoSpeeds(3, 1.3, -90, 0.1, 0.8, 0.34, 0.15) //go back fast


//                                goPast2Speeds(1.8, 0.83, 0, 0.01, 0.65, 0.65, 0),
//
//                                goPast2Speeds(2.4, 0.83, 0, 0.01, 0.9, 0.6, 0.15),
//
//
//                                goTo(2.69, 1.14, -90, 0.035, 0.7),
//                                moverWentWent(0.45),
//                                wait(375),
//
//                                //goPast2Speeds(2.8, 0.72, -90, 0.02, 0.8, 0.65, 0.2), //observation
//                                goToTwoSpeeds(2.72, 0.61, -90, 0.08, 0.8, 0.7, 0.2), //observation
//                                moverWentWent(1),
//
//                                goToTwoSpeeds(3.02, 1.3, -90, 0.1, 0.8, 0.35, 0.15) //go back fast


//                                goToTwoSpeeds(2.84, 0.94, -90, 0.01, 0.8, 0.5, 0.2), //go back fast
//
//                                new ParallelCommandGroup(
//                                        new SequentialCommandGroup(
//                                                moverWentWent(0.5),
//                                                wait(500))
//                                ),
//
//                                //goPast2Speeds(2.9, 0.72, -90, 0.02, 0.8, 0.65, 0.2), //observation
//                                goToTwoSpeeds(2.82, 0.7, -90, 0.08, 0.7, 0.65, 0.2), //observation
//                                moverWentWent(1)
//                                goTo(2.97, 1.08, -90, 0.03, 0.7), //go back fast
//
//                                new SequentialCommandGroup(
//                                    moverWentWent(0.5),
//                                    wait(500)
//                                ),
//
//                                goPast2Speeds(2.97, 0.5, -90, 0.02, 0.8, 0.45, 0.15),
//                                moverWentWent(0)
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        )

                ),

                new SequentialCommandGroup(//special movement for fast intake here:

                        moverGoGo(0.435),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        wait(500), moverGoGo(0.375),
                                        wait(500), moverGoGo(0.3)),
                                goTo(2.6, 0.22, -90, 0.04, 1), //intake position
                                new SequentialCommandGroup(
                                        wait(1000),
                                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 2050))),
                        //goTo(2.7, 0.22, -90, 0.04, 1),
                        new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem),
                        new WaitCommand(100),
                        moverGoGo(0.5)

                ),

//                new SequentialCommandGroup(//special movement for fast intake here:
//                        new ParallelCommandGroup(
//                                goTo(2.7, 0.22, -90, 0.04, 1),//intake position
//                                new SequentialCommandGroup(
//                                        wait(400),
//                                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1500))
//                        ),
//                        new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem)
//
//                ),


                chamberDischarge(1.83, 2),
                hpIntake(),
                chamberDischarge(1.83, 1),
                hpIntake(),
                chamberDischarge(1.83, 1),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        ),
                        goToTwoSpeeds(3, 0.7, -90, 0.08, 1, 0.4, 0.15)) //go back fast

        ));
    }

    public CommandBase chamberDischarge(double dischargeX, int transferMode) {

        if (transferMode == 0) {
            return new SequentialCommandGroup(

                    new ParallelDeadlineGroup(
                            new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                            goTo(dischargeX, 0.88, 0, 0.05, 1.5)),

                    dischargeAction(dischargeX)

            );
        }

        if (transferMode == 1) {
            return new SequentialCommandGroup(

                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.AutoTransfer(intakeSubsystem, dischargeSubsystem),
                                    new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),

                            goTo(dischargeX, 0.87, 0, 0.05, 1.3)),
                    //goPast2Speeds(dischargeX+0.03, 0.88, 0, 0.01, 1, 0.7, 0.15)),

                    //new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                    dischargeAction(dischargeX)

            );
        } else {
            return new SequentialCommandGroup(

                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new IntakeCommands.AutoTransfer(intakeSubsystem, dischargeSubsystem),
                                    new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),
                            new SequentialCommandGroup(
                                    wait(300),
                                    goTo(dischargeX, 0.87, 0, 0.08, 1.3),
                                    moverGoGo(0))),


                    //new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                    dischargeAction(dischargeX)
            );
        }
    }

    public CommandBase dischargeAction(double dischargeX) {
        return new ParallelDeadlineGroup(
                new DischargeCommands.SequentialRaceWrapper( // Custom wrapper for treating the sequence as a deadline
                        //new MecanumCommands.chamberWait(mecanumDrive),
                        new WaitCommand(600),//Todo: make time the minimum possible
                        new DischargeCommands.AutoChamberDischargeCmd(dischargeSubsystem, telemetry)
                ),
                goToConstVelocity(dischargeX, 3, 0, 0.03, 0.5).withTimeout(5000)
        );
    }

    public CommandBase hpIntake() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
                        ),
                        goTo(2.408, 0.656, -45, 0.03, 0.9),
                        new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 2120)),
                new IntakeCommands.SampleGroundIntakeCmd(intakeSubsystem)

        );
    }

    private CommandBase goToConstVelocity(double x, double y,
                                          double wantedAngle, double sensitivity, double speed) {
        return new MecanumCommands.ConstantVelocityGotoCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed);
//        return new MecanumCommands.GotoCmd()

    }

    private CommandBase goPast2Speeds(double x, double y,
                                      double wantedAngle, double sensitivity, double speed1, double speed2, double swapDistance) {
        return new MecanumCommands.ConstantVelocityGoPastCmd(telemetry, mecanumDrive, x, y, wantedAngle, sensitivity, speed1, speed2, swapDistance);
//        return new MecanumCommands.GotoCmd()

    }

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

    private CommandBase moverWentWent(double moveTo) {
        return new MecanumCommands.MoverWentWentCmd(mecanumDrive, moveTo);
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
