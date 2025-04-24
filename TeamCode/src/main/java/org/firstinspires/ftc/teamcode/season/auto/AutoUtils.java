package org.firstinspires.ftc.teamcode.season.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.SavedVariables;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.season.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.season.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.ArmsStages;
import org.firstinspires.ftc.teamcode.season.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.MecanumDrive;

public class AutoUtils {
    public static void initCommands(CommandOpMode commandOpMode, DischargeSubsystem dischargeSubsystem, IntakeSubsystem intakeSubsystem) {
        commandOpMode.schedule(new SequentialCommandGroup(

//                new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem),
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                //new IntakeCommands.Wait(intakeSubsystem, 1),
                new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true),
                new DischargeCommands.MotorControl(dischargeSubsystem, () -> 0.0, false, commandOpMode.telemetry),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
        ));
    }

    public static CommandBase inwardsPark(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 3.4, 0.2, 0, 0.04, 0.5, true);
    }

    public static CommandBase outwardsPark(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.8, 0.2, 0, 0.04, 0.5, true);
    }

    public static CommandBase dischargeGotoChamber(DischargeSubsystem dischargeSubsystem) {
        return new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight);
    }

    public static CommandBase dischargeGotoBasket(DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
        return new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight);
    }

    public static CommandBase chamberGoto(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.88, 0.88, 0, 0.05, 1.5);
    }

    public static CommandBase inFrontOfChamberGoto(MecanumDrive mecanumDrive, Telemetry telemetry, double y) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 1.8, y, 0, 0.03, 0.8);
    }

    public static CommandBase chamberDischarge(DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
        return new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry);
    }

    public static CommandBase nextToBasketGoto(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.2, 0.775, 180, 0.05, 0.5);
    }

    public static CommandBase basketDischargePositionGoto(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.31, 0.27, 225, 0.03, 0.8);
    }

    public static ParallelCommandGroup driverBasketPrep(MecanumDrive mecanumDrive, DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
        return new ParallelCommandGroup(basketDischargePositionGoto(mecanumDrive, telemetry),
                new DischargeCommands.GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highBasketHeight));
    }

    public static ParallelCommandGroup driverChamberPrep(MecanumDrive mecanumDrive, DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
        return new ParallelCommandGroup(chamberGoto(mecanumDrive, telemetry),
                dischargeGotoChamber(dischargeSubsystem));
    }

//    public static SequentialCommandGroup goToHPFromSub(MecanumDrive mecanumDrive, DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
//        return new SequentialCommandGroup(new MecanumCommands.ConstantVelocityGoPastCmd(telemetry, mecanumDrive, 3, 1.5, 0, 0.01, 1, true),
//                new MecanumCommands.ConstantVelocityGoPastCmd(telemetry, mecanumDrive, 3, 0.7, 0, 0.01, 1, true),
//                new MecanumCommands.SetRotationCmd(mecanumDrive, 0),
//                new DischargeCommands.HpDischarge(dischargeSubsystem));
//    }

    public static CommandBase goToHPFromChamber(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 2.5, 0.55, -45, 0.03, 1);
    }

    public static CommandBase basketDischarge(DischargeSubsystem dischargeSubsystem) {
        return new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem);
    }


    public static CommandBase dischargeGoHome(DischargeSubsystem dischargeSubsystem) {
        return new DischargeCommands.GoHomeCmd(dischargeSubsystem);
    }

    public static CommandBase secondYellowGoto(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.3, 0.775, 180, 0.01, 0.75);
    }

    public static CommandBase thirdYellowGoto(MecanumDrive mecanumDrive, Telemetry telemetry) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, 0.58, 0.775, 180, 0.01, 0.75);
    }

    public static CommandBase backupFromChamber(MecanumDrive mecanumDrive, Telemetry telemetry, double x) {
        return new MecanumCommands.GotoCmd(telemetry, mecanumDrive, x, 0.8, 0, 0.14, 1);
    }

    public static CommandBase startIntakeForSecondYellow(IntakeSubsystem intakeSubsystem) {
        return new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1400);
    }

    public static CommandBase startIntakeForThirdYellow(IntakeSubsystem intakeSubsystem) {
        return new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, 1440);
    }

    public static CommandBase sampleIntake(IntakeSubsystem intakeSubsystem) {
        return new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem);
    }

    public static CommandBase transfer(DischargeSubsystem dischargeSubsystem, IntakeSubsystem intakeSubsystem) {
        return new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem);
    }

    public static void savePosition(MecanumDrive mecanumDrive) {
        SavedVariables.angle = mecanumDrive.getAdjustedHeading();
        SavedVariables.x = mecanumDrive.getPosition().x;
        SavedVariables.y = mecanumDrive.getPosition().y;

    }
}
