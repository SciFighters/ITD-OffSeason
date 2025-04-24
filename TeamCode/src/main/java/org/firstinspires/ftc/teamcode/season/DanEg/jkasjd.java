package org.firstinspires.ftc.teamcode.season.DanEg;//package org.firstinspires.ftc.teamcode.shtrungul;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.button.GamepadButton;
//import com.arcrobotics.ftclib.command.button.Trigger;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//
//public class jkasjd extends CommandOpMode {
//    SampleSubsystem sub = new SampleSubsystem(hardwareMap, "3");
//    GamepadEx driver = new GamepadEx(gamepad1);
//
//    @Override
//    public void initialize() {
//        register(sub);
//        sub.setDefaultCommand(new SetPowerCommand(sub, driver::getLeftX));
//        GamepadButton b = new GamepadButton(driver, GamepadKeys.Button.A);
//        Trigger t = new Trigger(() -> driver.getLeftX() > 0.05).whileActiveContinuous(new SetPowerCommand(sub, driver::getLeftX));
//    }
//}
