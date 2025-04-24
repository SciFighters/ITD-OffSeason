package org.firstinspires.ftc.teamcode.season.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands.DischargeReleaseCmd;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands.DischargeGrabCmd;

@Config
@TeleOp
public class BasicDischargeTest extends CommandOpMode {
    GamepadEx systemGamepad;
    private DischargeSubsystem dischargeSubsystem;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    public static boolean gotoo = false, home = false;
    boolean wasGoto = false, washome = false;

    @Override
    public void initialize() {

        systemGamepad = new GamepadEx(gamepad2);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        register(dischargeSubsystem);
//        dischargeSubsystem.setDefaultCommand(new DischargeManualGotoCmd(() -> systemGamepad.getLeftY(), dischargeSubsystem, telemetry));
        Button dPadUp = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_UP);
        Button dPadDown = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_DOWN);
        Button dPadRight = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_RIGHT);
        Button dPadLeft = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_LEFT);
        Button leftBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_BUMPER);
        Button rightBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        Button A = new GamepadButton(systemGamepad, GamepadKeys.Button.A);
        Button X = new GamepadButton(systemGamepad, GamepadKeys.Button.X);
        Button Y = new GamepadButton(systemGamepad, GamepadKeys.Button.Y);
        Button B = new GamepadButton(systemGamepad, GamepadKeys.Button.B);


        dPadUp.whenPressed(new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight));
        dPadDown.whenPressed(new DischargeCommands.GoHomeCmd(dischargeSubsystem));
        dPadLeft.whenPressed(new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight));
        leftBumper.whenPressed(new DischargeReleaseCmd(dischargeSubsystem));
        rightBumper.whenPressed(new DischargeGrabCmd(dischargeSubsystem));
        X.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));
        Y.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));
        B.whenPressed(new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight - 210));
        A.whenPressed(new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight));
        //schedule(new DischargeCommands.GoHomeCmd(dischargeSubsystem));

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
        }

        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, systemGamepad::getRightY, false, telemetry));
    }

    @Override
    public void run() {
        if (gotoo && !wasGoto) {
            schedule(new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight));
            wasGoto = true;
            washome = false;
            home = false;
        }
        if (home && !washome) {
            schedule(new DischargeCommands.GoHomeCmd(dischargeSubsystem));
            wasGoto = false;
            washome = true;
            gotoo = false;
        }

//        if(lp != p|| li != i || ld != d || lf != f){
//            PIDFCoefficients pidf = new PIDFCoefficients(p,i,d,f);
//        }
//
//        lp = p;
//        li = i;
//        ld = d;
//        lf = f;
        super.run();
        //telemetry.addData("yPower", systemGamepad.getLeftY() * 0.75);
        //telemetry.addData("posInCM", dischargeSubsystem.getLiftPosInCM());
        //telemetry.addData("pos", dischargeSubsystem.getPosition());
        //telemetry.addData("pos2", dischargeSubsystem.getPosition2());
        //telemetry.addData("gearRatio", dischargeSubsystem.getGearBoxRatio());
        //telemetry.addData("timeUp", dischargeSubsystem.timeUp);
//
//        telemetry.addData("position", dischargeSubsystem.getPosition());
//        telemetry.addData("Target Pos", dischargeSubsystem.getTargetPosInTicks());
//        telemetry.addData("touch", dischargeSubsystem.isHome());
        //telemetry.addData("mode", dischargeSubsystem.getMode());
        //String commandName = dischargeSubsystem.getCurrentCommand().getName();
        //telemetry.addData("command", commandName==null ? "null" : commandName);

        multipleTelemetry.addData("p", dischargeSubsystem.getPIDFCoefficients().p);
        multipleTelemetry.addData("i", dischargeSubsystem.getPIDFCoefficients().i);
        multipleTelemetry.addData("d", dischargeSubsystem.getPIDFCoefficients().d);
        multipleTelemetry.addData("f", dischargeSubsystem.getPIDFCoefficients().f);
        multipleTelemetry.update();
//        telemetry.update();
    }
}