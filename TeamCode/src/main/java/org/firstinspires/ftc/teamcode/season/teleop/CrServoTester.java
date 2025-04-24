package org.firstinspires.ftc.teamcode.season.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class CrServoTester extends CommandOpMode {

    CRServo rightServo, leftServo;
    ElapsedTime time = new ElapsedTime();

    double duration = 0;
    GamepadEx driverGamepad;
    GamepadEx systemGamepad;
    com.arcrobotics.ftclib.command.button.Button systemA, driverA;
    com.arcrobotics.ftclib.command.button.Button systemB, driverB;
    com.arcrobotics.ftclib.command.button.Button systemY, driverY;
    com.arcrobotics.ftclib.command.button.Button systemX, driverX;
    com.arcrobotics.ftclib.command.button.Button systemDPadDown, driverDPadDown;
    com.arcrobotics.ftclib.command.button.Button systemDPadUp, driverDPadUp;
    com.arcrobotics.ftclib.command.button.Button systemDPadRight, driverDPadRight;
    com.arcrobotics.ftclib.command.button.Button systemDPadLeft, driverDPadLeft;
    com.arcrobotics.ftclib.command.button.Button systemRightBumper, driverRightBumper;
    com.arcrobotics.ftclib.command.button.Button systemLeftBumper, driverLeftBumper;
    com.arcrobotics.ftclib.command.button.Button driverStart;
    com.arcrobotics.ftclib.command.button.Button systemBack, driverBack;
    com.arcrobotics.ftclib.command.button.Button systemPs;
    com.arcrobotics.ftclib.command.button.Button systemStart;
    com.arcrobotics.ftclib.command.button.Button systemLeftStick, systemRightStick;
    com.arcrobotics.ftclib.command.button.Button driverLeftStick, driverRightStick;

    @Override
    public void initialize() {
        systemGamepad = new GamepadEx(gamepad2);
        systemA = new GamepadButton(systemGamepad, GamepadKeys.Button.A);
        systemB = new GamepadButton(systemGamepad, GamepadKeys.Button.B);
        systemY = new GamepadButton(systemGamepad, GamepadKeys.Button.Y);
        systemRightBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        systemLeftBumper = new GamepadButton(systemGamepad, Button.LEFT_BUMPER);
        systemY = new GamepadButton(systemGamepad, GamepadKeys.Button.Y);
        rightServo = hardwareMap.crservo.get("rWinchServo");
        leftServo = hardwareMap.crservo.get("lWinchServo");
    }

    @Override
    public void run() {
        super.run();
        systemA.whenPressed(new Runnable() {
            @Override
            public void run() {
                time.reset();
                rightServo.setPower(1);
                leftServo.setPower(1);
            }
        });
        systemY.whenPressed(new Runnable() {
            @Override
            public void run() {
                time.reset();
                rightServo.setPower(-1);
                leftServo.setPower(-1);

            }
        });
        systemB.whenPressed(() -> {
            duration = time.seconds();
            rightServo.setPower(0);
            leftServo.setPower(0);

        });


        telemetry.addData("duration", duration);
        telemetry.addData("time", time.seconds());
        telemetry.update();

    }

    private void initButtons() {
        systemA = new GamepadButton(systemGamepad, GamepadKeys.Button.A);
        systemB = new GamepadButton(systemGamepad, GamepadKeys.Button.B);
        systemY = new GamepadButton(systemGamepad, GamepadKeys.Button.Y);
        systemX = new GamepadButton(systemGamepad, GamepadKeys.Button.X);
        systemDPadDown = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_DOWN);
        systemDPadUp = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_UP);
        systemDPadRight = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_RIGHT);
        systemDPadLeft = new GamepadButton(systemGamepad, GamepadKeys.Button.DPAD_LEFT);
        systemRightBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        systemLeftBumper = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_BUMPER);
        systemBack = new GamepadButton(systemGamepad, GamepadKeys.Button.BACK);
        systemStart = new GamepadButton(systemGamepad, GamepadKeys.Button.START);
        driverA = new GamepadButton(driverGamepad, GamepadKeys.Button.A);
        driverB = new GamepadButton(driverGamepad, GamepadKeys.Button.B);
        driverY = new GamepadButton(driverGamepad, GamepadKeys.Button.Y);
        driverX = new GamepadButton(driverGamepad, GamepadKeys.Button.X);
        driverDPadDown = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN);
        driverDPadUp = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP);
        driverDPadRight = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT);
        driverDPadLeft = new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT);
        driverRightBumper = new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        driverLeftBumper = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER);
        driverStart = new GamepadButton(driverGamepad, GamepadKeys.Button.START);
        systemLeftStick = new GamepadButton(systemGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON);
        systemRightStick = new GamepadButton(systemGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        driverLeftStick = new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON);
        driverRightStick = new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        driverBack = new GamepadButton(driverGamepad, GamepadKeys.Button.BACK);
//        systemPs = new GamepadButton(systemGamepad, () -> driverGamepad.gamepad.ps);


    }
}
