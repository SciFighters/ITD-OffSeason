package org.firstinspires.ftc.teamcode.season.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;

@TeleOp
public class BasicIntakeTest extends CommandOpMode {
    GamepadEx systemGamepad;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
    IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        systemGamepad = new GamepadEx(gamepad2);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        register(intakeSubsystem);
        Button dPadUp = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_UP);
        Button dPadDown = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_DOWN);
        Button dPadRight = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_RIGHT);
        Button dPadLeft = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_LEFT);
        Button leftBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_BUMPER);
        Button rightBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        Button leftStickButton = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON);
        Button rightStickButton = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        Button A = new GamepadButton(systemGamepad, GamepadKeys.Button.A);
        Button Y = new GamepadButton(systemGamepad, GamepadKeys.Button.Y);
        Button B = new GamepadButton(systemGamepad, GamepadKeys.Button.B);
        Button X = new GamepadButton(systemGamepad, GamepadKeys.Button.X);
        Button back = new GamepadButton(systemGamepad, GamepadKeys.Button.BACK);
        //important stuff
        B.whenPressed(new IntakeCommands.RotateBackCmd(intakeSubsystem));
        Y.whenPressed(new IntakeCommands.SlideGotoCmd(intakeSubsystem, 1200));
        A.whenPressed(new IntakeCommands.SlideHomeCmd(intakeSubsystem, false));
        X.whenPressed(new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, false));
        dPadUp.whenPressed(new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.LOWER));
        dPadDown.whenPressed(new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.UPPER));
        dPadRight.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 1));
        dPadLeft.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.5));
        back.whenPressed(new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem));
        leftBumper.whenPressed(new IntakeCommands.OpenScrewCmd(intakeSubsystem, false));
        rightBumper.whenPressed(new IntakeCommands.CloseScrewCmd(intakeSubsystem, false));
        leftStickButton.whenPressed(new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.ROTATE_HEIGHT));


        //schedule(new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true));

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();
        }

        intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem, () -> systemGamepad.getLeftY()));
        //X.whenPressed(new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, false));
//        A.whenPressed(new IntakeCommands.SampleIntakeCmd(intakeSubsystem));
        //B.whenPressed(new IntakeCommands.StartIntakeCmd(intakeSubsystem));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("motor pos", intakeSubsystem.getAveragePosition());
        telemetry.addData("current", intakeSubsystem.getCurrent());
        telemetry.update();
    }
}
