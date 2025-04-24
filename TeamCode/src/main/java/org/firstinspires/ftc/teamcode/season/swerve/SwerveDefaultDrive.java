package org.firstinspires.ftc.teamcode.season.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class SwerveDefaultDrive extends CommandOpMode {

    @Override
    public void initialize() {
        SwerveDrive swerveDrive;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);
        GamepadEx gamepad = new GamepadEx(gamepad1);
        swerveDrive = new SwerveDrive(hardwareMap, multipleTelemetry, this, true);
        register(swerveDrive);
        swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(
                telemetry, swerveDrive, gamepad::getLeftX, gamepad::getLeftY,
                gamepad::getRightX, () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));


    }
}
