package org.firstinspires.ftc.teamcode.season.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.SavedVariables;
import org.firstinspires.ftc.teamcode.season.commands.ClimbCommands;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands;
import org.firstinspires.ftc.teamcode.season.commands.IntakeCommands;
import org.firstinspires.ftc.teamcode.season.commands.LimelightCommands;
import org.firstinspires.ftc.teamcode.season.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.season.commands.SetStateCommands;
import org.firstinspires.ftc.teamcode.season.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.season.subsystems.Pipelines;
import org.firstinspires.ftc.teamcode.season.subsystems.RobotState;
import org.opencv.core.Point;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

@TeleOp

public class Echo extends CommandOpMode {

    MecanumDrive mecanumDrive;
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limeLightSubsystem;
    ClimbSubsystem climbSubsystem;

    Supplier<Double> mecanumX, mecanumY, mecanumR;

    static RobotState robotState;
    static RobotState controllersState;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    MultipleTelemetry multipleTelemetry = new MultipleTelemetry(telemetry, dashboardTelemetry);


    GamepadEx driverGamepad;
    GamepadEx systemGamepad;
    Button systemA, driverA;
    Button systemB, driverB;
    Button systemY, driverY;
    Button systemX, driverX;
    Button systemDPadDown, driverDPadDown;
    Button systemDPadUp, driverDPadUp;
    Button systemDPadRight, driverDPadRight;
    Button systemDPadLeft, driverDPadLeft;
    Button systemRightBumper, driverRightBumper;
    Button systemLeftBumper, driverLeftBumper;
    Button driverStart;
    Button systemBack, driverBack;
    Button systemPs;
    Button systemStart;
    Button systemLeftStick, systemRightStick;
    Button driverLeftStick, driverRightStick;
    Trigger systemRTrigger;
    Trigger systemLTrigger;
    Queue<CommandBase> queue = new LinkedList<>();
    Runnable pullQueue;
    boolean queueable = false;
    BooleanSupplier queueableSup = () -> !queueable;
    long swapTime = 0;
    public Pipelines pipeline;


    @Override
    public void initialize() {

//        pullQueue = new Runnable() {
//            @Override
//            public void run() {
//                if (queue.size() > 0 && queueable) {
//                    schedule(queue.remove());
//                    queueable = false;
//                }
//
//            }
//        };
        driverGamepad = new GamepadEx(gamepad1);
        systemGamepad = new GamepadEx(gamepad2);

        mecanumDrive = new MecanumDrive(multipleTelemetry, hardwareMap, new Point(SavedVariables.x, SavedVariables.y), SavedVariables.angle, this);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap, multipleTelemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, multipleTelemetry);
        limeLightSubsystem = new LimelightSubsystem(hardwareMap, multipleTelemetry);
        climbSubsystem = new ClimbSubsystem(hardwareMap);
        register(mecanumDrive, dischargeSubsystem, intakeSubsystem, limeLightSubsystem, climbSubsystem);
        initButtons();
        robotState = RobotState.NONE;
        controllersState = null;

        //init commands
//        schedule(new SequentialCommandGroup(
//                new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem),
//                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
//                new IntakeCommands.ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
//                //new IntakeCommands.Wait(intakeSubsystem, 1),
//                new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true),
//                new IntakeCommands.SetArmsStageCmd(intakeSubsystem, ArmsStages.TRANSFER),
//                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));
//        IntakeCommands.IntakeManualGoToCmd.setEnabled(true);

        schedule(new DischargeCommands.DischargeGrabCmd(dischargeSubsystem));

        while (opModeInInit()) {
            super.run();
        }
        limeLightSubsystem.startLimelight();
        schedule(new IntakeCommands.ReturnArmForTransferCmd(intakeSubsystem, true));
        schedule(new DischargeCommands.MotorControl(dischargeSubsystem, systemGamepad::getRightY, true, telemetry));
        mecanumDrive.setHeading(SavedVariables.angle);

        mecanumX = () -> driverGamepad.getLeftX();
        mecanumY = () -> driverGamepad.getLeftY();
        mecanumR = () -> driverGamepad.getRightX();

//        swerveDrive.setDefaultCommand(new SwerveCommands.PowerCmd(telemetry, swerveDrive,
//                driverGamepad::getLeftX, driverGamepad::getLeftY, driverGamepad::getRightX,
//                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), true));
//
    }


    @Override
    public void run() {

//        AutoUtils.savePosition(mecanumDrive);
        if (controllersState != robotState) {
            CommandScheduler.getInstance().clearButtons();

//            IntakeCommands.IntakeManualGoToCmd.endCommand();
            CommandScheduler.getInstance().cancel(mecanumDrive.getDefaultCommand());
            CommandScheduler.getInstance().cancel(dischargeSubsystem.getDefaultCommand());
            CommandScheduler.getInstance().cancel(intakeSubsystem.getDefaultCommand());
            CommandScheduler.getInstance().setDefaultCommand(mecanumDrive, new MecanumCommands.NoOpCommand(mecanumDrive));
            CommandScheduler.getInstance().setDefaultCommand(intakeSubsystem, new IntakeCommands.NoOpCommand(intakeSubsystem));
            CommandScheduler.getInstance().setDefaultCommand(dischargeSubsystem, new DischargeCommands.NoOpCommand(dischargeSubsystem));
//            driverA.whenHeld(new SwerveCommands.SetRotationCmd(swerveDrive, 0))
//                    .and(new Trigger(() -> !driverStart.get()));

            systemLeftStick.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));
            systemRightStick.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));


            controllersState = robotState;
            switch (robotState) {
                case NONE:
                    noneBindings();
                    break;
                case INTAKE:
                    intakeBindings();
                    break;
                case BASKET:
                    basketBindings();
                    break;
                case CHAMBER:
                    chamberBindings();
                    break;
                case CLIMB:
                    climbBindings();
                    break;
                case AUTOINTAKE:
                    autoIntakeBindings();
                    break;


            }
        }
        if (driverX.get() && driverStart.get()) {
            mecanumDrive.resetHeading();
            mecanumDrive.setHeading(90);
        }
        if (robotState == RobotState.CLIMB && System.currentTimeMillis() > swapTime + 500 && gamepad2.ps) {
            robotState = RobotState.NONE;
            swapTime = System.currentTimeMillis();
        } else if (robotState == RobotState.NONE && System.currentTimeMillis() > swapTime + 500 && gamepad2.ps) {
            robotState = RobotState.CLIMB;
            mecanumDrive.setMoverServo(0.18);
            swapTime = System.currentTimeMillis();
        }
        if (robotState == RobotState.CLIMB)
            DischargeCommands.MotorControl.setMode(DischargeCommands.MotorControl.Mode.DO_NOTHING);
        if (systemGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            if (pipeline != null) {
                if (limeLightSubsystem.getCurrentPipeline() != pipeline.PIPELINE)
                    limeLightSubsystem.setPipeline(pipeline);
            }
        }

        if (systemGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            if (limeLightSubsystem.getCurrentPipeline() != Pipelines.YELLOW.PIPELINE)
                limeLightSubsystem.setPipeline(Pipelines.YELLOW);
        }


        //if (systemA.get() && controllersState == RobotState.INTAKE)
        //    systemX.whenPressed(new SequentialCommandGroup(
        //            new SetStateCommands.NoneStateCmd(),
        //            new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem)));
        super.run();
        telemetries();

    }

    public void chamberBindings() {
        intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem, systemGamepad::getLeftY));
        systemB.whenPressed(new IntakeCommands.StartIntakeCmd(intakeSubsystem));


        driverB.whenPressed(new ClimbCommands.moveWinchByTime(climbSubsystem, 1, "both", 5000));

        systemRightStick.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));

        driverDPadDown.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> -0.25, () -> 0.0,
                () -> 1.0, true));
        driverDPadUp.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> 0.25, () -> 0.0,
                () -> 1.0, true));
        driverDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.25, () -> 0.0, () -> 0.0,
                () -> 1.0, true));
        driverDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.25, () -> 0.0, () -> 0.0,
                () -> 1.0, true));


        mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                mecanumX, mecanumY, mecanumR,
                () -> Math.max(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * 14 + 0.5, true));


        systemRightBumper.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.NoneStateCmd(),
                new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry)));


        systemLeftBumper.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.NoneStateCmd(),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));


        systemA.whenPressed(new SequentialCommandGroup(
                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight))); //go to chamber

        systemY.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.BasketStateCmd(), //change to basket state
                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight))); //go to high basket

        systemBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));

        driverBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));
        driverRightBumper.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setWentWentServo(0.5)), new InstantCommand(() -> mecanumDrive.setWentWentServo(1)));
        driverLeftBumper.whenPressed(new DischargeCommands.ChamberDischargeCmd(dischargeSubsystem, telemetry));

    }

    public void basketBindings() {

        driverB.whenPressed(new ClimbCommands.moveWinchByTime(climbSubsystem, 1, "both", 5000));

//        driverA.whenPressed(new MecanumCommands.SetExtraRotationCmd(mecanumDrive, 225));

        driverDPadDown.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> -0.25, () -> 0.0,
                () -> 1.0, true));
        driverDPadUp.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> 0.25, () -> 0.0,
                () -> 1.0, true));
        driverDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.25, () -> 0.0, () -> 0.0,
                () -> 1.0, true));
        driverDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.25, () -> 0.0, () -> 0.0,
                () -> 1.0, true));

        mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                mecanumX, mecanumY, mecanumR,
                () -> Math.max(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * 0.5 + 0.5, true));


        systemRightBumper.whenPressed(new SequentialCommandGroup(
                new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem)));

        systemLeftBumper.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.NoneStateCmd(),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));

        systemA.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.ChamberStateCmd(), //change to chamber state
                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)));
//        systemA.whenPressed(
//                new ConditionalCommand(
//                        new SequentialCommandGroup(
//                                new SetStateCommands.ChamberStateCmd(), //change to chamber state
//                                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
//                                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),
//                        new SequentialCommandGroup(
//                                new SetStateCommands.ChamberStateCmd(),
//                                new InstantCommand( () -> queue.add(new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)))
//                        ),
//                        queueableSup)); //go to chamber
        systemY.whenPressed(new SequentialCommandGroup(
                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight))); //go to high basket

        systemBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));

        driverBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));
        driverLeftBumper.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));

    }

    public void intakeBindings() {
        driverB.whenPressed(new ClimbCommands.moveWinchByTime(climbSubsystem, 1, "both", 5000));


        driverLeftStick.whenPressed(() -> limeLightSubsystem.limelight.captureSnapshot("la shot"));
//        mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
//                systemGamepad::getRightX, mecanumY, () -> 0.0,
//                () -> systemGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5 + 0.2, false));

        mecanumDrive.setDefaultCommand(new MecanumCommands.IntakePowerCmd(telemetry, mecanumDrive,
                systemGamepad::getRightX, () -> 0.0, () -> 0.0,
                mecanumX, mecanumY, mecanumR,
                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.5 + 0.3));

        intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem,
                () -> systemGamepad.getLeftY() * (1 - 0.6 * systemGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER))));

        systemA.whenPressed(new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem));/*.and(new Trigger(() -> IntakeCommands.Transfer.transferring))*/

        systemB.whenPressed(new IntakeCommands.RestartIntakeCmd(intakeSubsystem)).and(new Trigger(() -> !systemStart.get()));

//                    systemY.whenPressed(new SequentialCommandGroup(
//                            new IntakeCommands.ReturnArmForHMCmd(intakeSubsystem),
//                            new SetStateCommands.NoneStateCmd()));

        systemX.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.NoneStateCmd(),
                new IntakeCommands.Transfer(intakeSubsystem, dischargeSubsystem)), false);

//                    systemDPadUp.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.5));
//                    systemDPadRight.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0));
//                    systemDPadLeft.whenPressed(new IntakeCommands.SetRotationCmd(intakeSubsystem, 1));

//                    systemDPadDown.whenPressed(
//                            new IntakeCommands.SetRotationCmd(intakeSubsystem,
//                                    (1 - (limeLightSubsystem.getAngle() + 90) / 180 - 0.5) * 2 / 3 + 0.5)
//                    );

        systemDPadDown.whenPressed(new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem, 0.5));
        systemDPadUp.whenPressed(new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem, 0));
        systemDPadLeft.whenPressed(new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem, 0.25));
        systemDPadRight.whenPressed(new IntakeCommands.SampleSubmIntakeCmd(intakeSubsystem, 0.75));

//        systemDPadUp.whenPressed(new SequentialCommandGroup(
//                new IntakeCommands.SetRotationCmd(intakeSubsystem, 0),
//                new IntakeCommands.StartIntakeCmd(intakeSubsystem))
//        );
//
//        systemDPadLeft.whenPressed(
//                new SequentialCommandGroup(
//                        new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.5),
//                        new IntakeCommands.StartIntakeCmd(intakeSubsystem))
//        );

//                    systemDPadLeft.whenReleased(
//                            new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.6)
//                    );
//                    systemDPadRight.whenPressed(
//                            new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.75)
//                    );
//                    systemDPadRight.whenReleased(new IntakeCommands.SetRotationCmd(intakeSubsystem, 0.25));


        systemLeftBumper.whenPressed(
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)
        );

        systemBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));

        driverBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));

    }

    public void noneBindings() {

        driverB.whenPressed(new ClimbCommands.moveWinchByTime(climbSubsystem, 1, "both", 5000));


        driverLeftStick.whenPressed(() -> limeLightSubsystem.limelight.captureSnapshot("la shot"));


        systemLeftStick.whenPressed(new DischargeCommands.ResetDischarge(dischargeSubsystem));
        systemRightBumper.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));

        driverDPadDown.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> -0.25, () -> 0.0,
                () -> 1.0, true));
        driverDPadUp.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> 0.25, () -> 0.0,
                () -> 1.0, true));
        driverDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.25, () -> 0.0, () -> 0.0,
                () -> 1.0, true));
        driverDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.25, () -> 0.0, () -> 0.0,
                () -> 1.0, true));


        mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                mecanumX, mecanumY, mecanumR, ()
                -> Math.max(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * 0.5 + 0.5, true));

        intakeSubsystem.setDefaultCommand(new IntakeCommands.IntakeManualGoToCmd(intakeSubsystem, systemGamepad::getLeftY));

        driverLeftBumper.whenPressed(() -> mecanumDrive.resetPos(new Point(0.9, 1)));

//        systemPs.whenPressed(new SetStateCommands.ClimbStateCmd());
        //systemRightStick.whenPressed(ChamberOnly)

//        driverA.whenPressed(AutoUtils.goToHPFromChamber(mecanumDrive, telemetry).beforeStarting(() -> {
//            queueable = true;
//        }).andThen(pullQueue()), false);
//
//        driverB.whenPressed(new SequentialCommandGroup(new SetStateCommands.BasketStateCmd(),
//                AutoUtils.driverBasketPrep(mecanumDrive, dischargeSubsystem, telemetry).beforeStarting(() -> {
//                    queueable = true;
//                }).whenFinished(pullQueue)), false);
//
//        driverY.whenPressed(new SequentialCommandGroup(new SetStateCommands.ChamberStateCmd(),
//                AutoUtils.driverChamberPrep(mecanumDrive, dischargeSubsystem, telemetry).beforeStarting(() -> {
//                    queueable = true;
//                }).whenFinished(pullQueue)), false);
//
//        driverX.whenPressed(AutoUtils.goToHPFromSub(mecanumDrive, dischargeSubsystem, telemetry).beforeStarting(() -> queueable = true).andThen(new ScheduleCommand(pullQueue())), false);
//        CommandScheduler.getInstance();
//        systemA.whenPressed(
//                new ConditionalCommand(
//                        new InstantCommand(() -> queue.add(
//                                new SequentialCommandGroup(
//                                    new SetStateCommands.ChamberStateCmd(), //change to chamber state
//                                    new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
//                                    new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)))),
//
//                        new SequentialCommandGroup(
//                                new SetStateCommands.ChamberStateCmd(), //change to chamber state
//                                new DischargeCommands.DischargeGrabCmd(dischargeSubsystem),
//                                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight)),
//                        queueableSup));

//                            new DischargeCommands.DischargeGotoCmd(dischargeSubsystem
//                                    , dischargeSubsystem.highChamberHeight, multipleTelemetry))); //go to chamber
        systemY.whenPressed(
                new SequentialCommandGroup(
                        new IntakeCommands.WaitForTransferEnd(),
                        new SetStateCommands.BasketStateCmd(), //change to chamber state
                        //new DischargeCommands.GoToTarget(dischargeSubsystem.highBasketHeight),
                        new ParallelCommandGroup(
                                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highBasketHeight),
                                new IntakeCommands.SlideUntilCmd(intakeSubsystem, 1000, 1, false))));
//        systemY.whenPressed(new LimelightCommands.AlignXCmd(limeLightSubsystem, mecanumDrive));

        systemA.whenPressed(
                new SequentialCommandGroup(
                        new IntakeCommands.WaitForTransferEnd(),
                        new SetStateCommands.ChamberStateCmd(), //change to chamber state
                        new ParallelCommandGroup(
                                new DischargeCommands.GoToTarget(dischargeSubsystem, dischargeSubsystem.highChamberHeight),
                                new IntakeCommands.SlideUntilCmd(intakeSubsystem, 1000, 1, false))));


        systemX.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.AutoIntakeStateCmd(),
                new LimelightCommands.LimelightStartIntake(limeLightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive),
                new SetStateCommands.IntakeStateCmd()
        ));
//        systemX.whenPressed(new LimelightCommands.AlignXCmd(limeLightSubsystem,mecanumDrive));
        systemB.whenPressed(new IntakeCommands.SlideUntilCmd(intakeSubsystem, 1500, 0.8, true));
        systemB.whenReleased(new SequentialCommandGroup(new SetStateCommands.IntakeStateCmd(),
                new IntakeCommands.StartIntakeCmd(intakeSubsystem))).and(new Trigger(() -> !systemStart.get()));

//                    systemLeftStick.whenPressed(new DischargeCommands.GearBoxClimbCmd(dischargeSubsystem));
//                    systemRightStick.whenPressed(new DischargeCommands.GearBoxDischargeCmd(dischargeSubsystem));

        systemDPadLeft.whenPressed(new SequentialCommandGroup(new IntakeCommands.WaitForTransferEnd(), new DischargeCommands.HpDischarge(dischargeSubsystem)));

        systemLeftBumper.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.NoneStateCmd(),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));
//        systemDPadUp.whenPressed(new LimelightCommands.LimelightCompleteSubIntake(limeLightSubsystem, intakeSubsystem, dischargeSubsystem, mecanumDrive));
//                    systemDPadDown.whenPressed(new SequentialCommandGroup(
//                            new IntakeCommands.StartIntakeCmd(intakeSubsystem, true, limeLightSubsystem::getYDistance),
//                            new SetStateCommands.IntakeStateCmd()));
//                    systemDPadLeft.whenPressed(new LimelightCommands.AlignXCmd(limeLightSubsystem, mecanumDrive));
        systemBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));

        driverRightBumper.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));
//        driverA.whenPressed(new MecanumCommands.SetExtraRotationCmd(mecanumDrive, 225));
        systemStart.toggleWhenPressed(new InstantCommand(() -> limeLightSubsystem.setPipeline(Pipelines.YELLOW)), new InstantCommand(() -> limeLightSubsystem.setPipeline(pipeline)));

    }

    public void climbBindings() {
        driverRightBumper.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setWentWentServo(0.5)), new InstantCommand(() -> mecanumDrive.setWentWentServo(1)));
        DischargeCommands.MotorControl.setMode(DischargeCommands.MotorControl.Mode.DO_NOTHING);
        intakeSubsystem.setDefaultCommand(new IntakeCommands.NoOpCommand(intakeSubsystem));

        systemLeftStick.whenPressed(new DischargeCommands.ResetDischarge(dischargeSubsystem));
        systemRightBumper.whenPressed(new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem));

        driverDPadDown.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> -0.25, () -> 0.0,
                () -> 1.0, true));
        driverDPadUp.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.0, () -> 0.25, () -> 0.0,
                () -> 1.0, true));
        driverDPadLeft.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> -0.25, () -> 0.0, () -> 0.0,
                () -> 1.0, true));
        driverDPadRight.whileHeld(new MecanumCommands.PowerCmd(telemetry, mecanumDrive, () -> 0.25, () -> 0.0, () -> 0.0,
                () -> 1.0, true));

        mecanumDrive.setDefaultCommand(new MecanumCommands.PowerCmd(telemetry, mecanumDrive,
                mecanumX, mecanumY, mecanumR, ()
                -> Math.max(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * 0.5 + 0.5, true));


        systemLeftBumper.whenPressed(new SequentialCommandGroup(
                new SetStateCommands.NoneStateCmd(),
                new DischargeCommands.GoHomeCmd(dischargeSubsystem)));

        systemBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));

        driverBack.toggleWhenPressed(new InstantCommand(() -> mecanumDrive.setMoverServo(0.5)), new InstantCommand(() -> mecanumDrive.setMoverServo(0.08)));
//        driverA.whenPressed(new MecanumCommands.SetExtraRotationCmd(mecanumDrive, 225));
        systemStart.toggleWhenPressed(new InstantCommand(() -> limeLightSubsystem.setPipeline(Pipelines.YELLOW)), new InstantCommand(() -> limeLightSubsystem.setPipeline(pipeline)));

        driverB.whenPressed(new ClimbCommands.moveWinchByTime(climbSubsystem, 1, "both", 7500));
        systemA.whenPressed(new ClimbCommands.moveWinchByTime(climbSubsystem, -0.02, "hi mate", 4000));

        climbSubsystem.setDefaultCommand(new ClimbCommands.SetWinchPower(climbSubsystem, () -> -systemGamepad.getRightY(), () -> -systemGamepad.getLeftY()));

    }

    public void autoIntakeBindings() {
        systemB.whenPressed(new SequentialCommandGroup(new SetStateCommands.IntakeStateCmd(),
                new IntakeCommands.StartIntakeCmd(intakeSubsystem))).and(new Trigger(() -> !systemStart.get()));
    }


    public void telemetries() {
//        telemetry.addData("hello", "hello");
        //---states & commands---
        telemetry.addLine("---states & commands---");
        telemetry.addData("state", robotState);
        telemetry.addData("discharge current command", dischargeSubsystem.getCurrentCommand().getName());
        telemetry.addData("intake current command", intakeSubsystem.getCurrentCommand().getName());

        //---lift---
        telemetry.addLine("---lift---");
        multipleTelemetry.addData("lift mode", DischargeCommands.MotorControl.getMode());
        multipleTelemetry.addData("lift pos", dischargeSubsystem.getLiftPosInCM());
        multipleTelemetry.addData("lift goto target", DischargeCommands.MotorControl.getTargetPosition());
        multipleTelemetry.addData("lift goto error", DischargeCommands.MotorControl.getTargetPosition() - dischargeSubsystem.getLiftPosInCM());
        multipleTelemetry.addData("lift stay still target", DischargeCommands.MotorControl.getStayStillTarget());
        multipleTelemetry.addData("lift stay still error", DischargeCommands.MotorControl.getTargetPosition() - dischargeSubsystem.getPosition());
        telemetry.addData("lift touch", dischargeSubsystem.isHome());
        telemetry.addData("current", dischargeSubsystem.getCurrent());

        //---intake---
        telemetry.addLine("---intake---");
        telemetry.addData("home", intakeSubsystem.isHome());
        multipleTelemetry.addData("transferring", IntakeCommands.Transfer.transferring);
        multipleTelemetry.addData("tick avg", intakeSubsystem.getAveragePosition() / 435.0 * 1150 / 58 * 46);
        multipleTelemetry.addData("tick 1", intakeSubsystem.getMotorPosition() / 435.0 * 1150 / 58 * 46);
        multipleTelemetry.addData("tick 2", intakeSubsystem.getMotor2Position() / 435.0 * 1150 / 58 * 46);
        telemetry.addData("intakePower", intakeSubsystem.getPower());
        telemetry.addData("currentIntake", intakeSubsystem.getCurrent());

//        ---limelight---
        telemetry.addLine("---limelight---");
        telemetry.addData("running", limeLightSubsystem.limelight.isRunning());
        multipleTelemetry.addData("x limelight", limeLightSubsystem.getXDistance());
        multipleTelemetry.addData("y limelight", limeLightSubsystem.getYDistance());
        multipleTelemetry.addData("angle limelight", limeLightSubsystem.getAngle());
        multipleTelemetry.addData("pipeline", limeLightSubsystem.getCurrentPipeline());
        multipleTelemetry.addData("rawY", limeLightSubsystem.getRawY());
        multipleTelemetry.addData("initialDistance", limeLightSubsystem.initialDistance);
        multipleTelemetry.addData("cm", limeLightSubsystem.limelightInCm);
        multipleTelemetry.addData("x odometer limelight", limeLightSubsystem.getXDistanceOdometer());
        multipleTelemetry.addData("fhd", limeLightSubsystem.alignedY);
        telemetry.addData("servo angle", 1 - (limeLightSubsystem.getAngle() + 90) / 180);
        multipleTelemetry.addData("derivative", LimelightCommands.AlignXCmd.derivative);

        //----climb---
        telemetry.addData("right servo power", climbSubsystem.getrLastPower());
        telemetry.addData("left servo power", climbSubsystem.getlLastPower());


        //---mecanum---
        telemetry.addLine("---limelight---");
        multipleTelemetry.addData("pos", mecanumDrive.getPosition());
        multipleTelemetry.addData("savedAngle", SavedVariables.angle);
//        multipleTelemetry.addData("bl", mecanumDrive.bl.getCurrentPosition());
//        multipleTelemetry.addData("br", mecanumDrive.br.getCurrentPosition());
//        multipleTelemetry.addData("fl", mecanumDrive.fl.getCurrentPosition());
//        multipleTelemetry.addData("fr", mecanumDrive.fr.getCurrentPosition());
        multipleTelemetry.addData("flPower", mecanumDrive.fl.getPower());
        multipleTelemetry.addData("frPower", mecanumDrive.fr.getPower());
        multipleTelemetry.addData("blPower", mecanumDrive.bl.getPower());
        multipleTelemetry.addData("brPower", mecanumDrive.br.getPower());
        multipleTelemetry.addData("heading", mecanumDrive.getHeading());
        multipleTelemetry.addData("driverx", mecanumX.get());
        multipleTelemetry.addData("drivery", mecanumY.get());
//        multipleTelemetry.addData("servo pos", intakeSubsystem.getZServoPosition());
//        telemetry.addData("y saved",SavedVariables.y);
//        telemetry.addData("robot x,y", mecanumDrive.getPosition());
//        telemetry.addData("robot angle", mecanumDrive.getHeading());
        multipleTelemetry.update();
        telemetry.update();
    }

    public static void setRobotState(RobotState state) {
        robotState = state;
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
