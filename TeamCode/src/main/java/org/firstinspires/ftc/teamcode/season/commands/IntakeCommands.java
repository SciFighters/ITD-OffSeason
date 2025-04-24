package org.firstinspires.ftc.teamcode.season.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands.DischargeGrabCmd;

import java.util.function.Supplier;

public class IntakeCommands {

    public static class NoOpCommand extends CommandBase {
        public NoOpCommand(IntakeSubsystem intakeSubsystem) {
            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            // Do nothing
        }

        @Override
        public boolean isFinished() {
            return false; // Runs indefinitely
        }
    }

    public static class OpenScrewCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        final boolean wait;
        ElapsedTime time = new ElapsedTime();

        public OpenScrewCmd(IntakeSubsystem intakeSubsystem, boolean wait) {
            this.intakeSubsystem = intakeSubsystem;
            this.wait = wait;
//            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.openScrew();
            time.reset();
        }

        @Override
        public boolean isFinished() {
            return time.seconds() > intakeSubsystem.openScrewTime || !wait;
        }
    }

    public static class CloseScrewCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        final boolean wait;
        ElapsedTime time = new ElapsedTime();


        public CloseScrewCmd(IntakeSubsystem intakeSubsystem, boolean wait) {
            this.intakeSubsystem = intakeSubsystem;
            this.wait = wait;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            intakeSubsystem.closeScrew();
            time.reset();
        }

        @Override
        public boolean isFinished() {
            return time.seconds() > intakeSubsystem.openScrewTime || !wait;
        }
    }

    public static class SlideGotoCmd extends CommandBase {
        IntakeSubsystem subsystem;
        int position = 0;
        Supplier<Integer> positionSupplier;
        boolean limelight = false;
        final int maxPosition = 3000;
        final double ratio = (435.0 / 1150.0) * (58.0 / 46.0);


        public SlideGotoCmd(IntakeSubsystem subsystem, int position) {
            this.subsystem = subsystem;
            if (position > maxPosition) {
                position = maxPosition;
            } else if (position < 0) {
                position = 0;
            }
            this.position = (int) (position * ratio);
            addRequirements(subsystem);
        }

        public SlideGotoCmd(IntakeSubsystem subsystem, Supplier<Integer> position) {
            this.subsystem = subsystem;
            positionSupplier = position;
            limelight = true;
        }

        @Override
        public void initialize() {
            if (limelight) {
                subsystem.setTargetPos((int) (positionSupplier.get() * ratio));
            } else {
                subsystem.setTargetPos(position);
            }
            subsystem.armGoToTarget();
            subsystem.runWithEncoders();
        }

        @Override
        public boolean isFinished() {
            if (limelight) {
                return Math.abs(subsystem.getMotorPosition() - positionSupplier.get() * ratio) <= 15;
            }
            return Math.abs(subsystem.getMotorPosition() - position) <= 15;
        }
    }

    public static class SlideHomeCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double lastTick;
        double lastTime = 0;
        final double maxDuration = 3;
        final boolean initTime;
        final int minPosOffset = 40;
        ElapsedTime elapsedTime = new ElapsedTime();

        public SlideHomeCmd(IntakeSubsystem intakeSubsystem, boolean initTime) {
            this.intakeSubsystem = intakeSubsystem;
            this.initTime = initTime;
            //addRequirements(intakeSubsystem);

        }

        @Override
        public void initialize() {
            lastTick = intakeSubsystem.getAveragePosition();
            elapsedTime.reset();
            intakeSubsystem.setArmPower(0);
            intakeSubsystem.runWithoutEncoders();
            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            int pos = intakeSubsystem.getMotorPosition();
            if (initTime) {
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesHalfSpeed);
            } else if (pos > 320.0) {
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesSpeed);
            } else if (pos > 140) {
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesHalfSpeed);
            } else {
                intakeSubsystem.setRawPower(-intakeSubsystem.slidesLowSpeed);
            }

        }

        @Override
        public boolean isFinished() {
            if (!initTime && elapsedTime.seconds() > maxDuration) {
                return true;
            }

//            double deltaTime = elapsedTime.seconds() - lastTime;
//            if (deltaTime > 0.2) {
//                double avg = intakeSubsystem.getAveragePosition();
//                double deltaTick = avg - lastTick;
//                lastTick = avg;
//                lastTime = elapsedTime.seconds();
//                return (Math.abs(deltaTick) <= 8 && (avg < 200 || initTime));

//            }
            return intakeSubsystem.isHome(); //|| (intakeSubsystem.getCurrent() > 90 && intakeSubsystem.getAveragePosition() < 400);
        }

        @Override
        public void end(boolean interrupted) {
            if (!interrupted)
                intakeSubsystem.resetEncoders();
        }
    }


    public static class ClawStageCmd extends CommandBase {
        IntakeSubsystem subsystem;
        double stage;
        ElapsedTime elapsedTime = new ElapsedTime();

        public ClawStageCmd(IntakeSubsystem subsystem, double stage) {
            this.stage = stage;
            this.subsystem = subsystem;
//            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.setHServoPosition(stage);
            elapsedTime.reset();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    //doesn't have addRequirements(subsystem);

    public static class RotateBackCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;

        public RotateBackCmd(IntakeSubsystem intakeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
        }


        @Override
        public void initialize() {
            intakeSubsystem.setRotationServoPosition(0.5);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class SetRotationCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double position;
        Supplier<Double> positionSupplier;
        boolean limelight = false;

        public SetRotationCmd(IntakeSubsystem intakeSubsystem, double position) {
            this.intakeSubsystem = intakeSubsystem;
            this.position = position;
//            addRequirements(intakeSubsystem);
        }

        public SetRotationCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> position) {
            this.intakeSubsystem = intakeSubsystem;
            this.positionSupplier = position;
            limelight = true;
//            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            if (limelight) {
//                position = (positionSupplier.get() > 0) ? positionSupplier.get() : 180 + positionSupplier.get();
//                double wanted = ((position / 180 - 0.5) * 2 / 3 + 0.5);
//                if (wanted > 0.75) {
//                    wanted = 0;
//                }
                position = positionSupplier.get() + 90;
                double wanted = position / 180;
                wanted = (wanted - 0.5) * 2 / 3 + 0.5;
                intakeSubsystem.setRotationServoPosition(wanted);//((1 - (positionSupplier.get() + 90) / 180 - 0.5) * 2 / 3 + 0.5)

            } else
                intakeSubsystem.setRotationServoPosition((position - 0.5) * 2 / 3 + 0.5);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    //doesn't have addRequirements
    public static class SetZRotationSupplierCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        Supplier<Double> position;

        public SetZRotationSupplierCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> position) {
            this.intakeSubsystem = intakeSubsystem;
            this.position = position;
//            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            intakeSubsystem.setRotationServoPosition(position.get());
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    public static class IntakeManualGoToCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        Supplier<Double> power;
        ElapsedTime elapsedTime = new ElapsedTime();
        double lastTimeMilli = 0;
        private static boolean isEnabled = true;
        private static IntakeManualGoToCmd currentInstance;

        public IntakeManualGoToCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> power) {
            this.intakeSubsystem = intakeSubsystem;
            this.power = power;


            currentInstance = this;
            addRequirements(intakeSubsystem);
        }

        public static void setEnabled(boolean enabled) {
            isEnabled = enabled;
        }

        public static boolean isEnabled() {
            return isEnabled;
        }

        @Override
        public void initialize() {
            elapsedTime.reset();
        }

        @Override
        public void execute() {
            if (isEnabled) {
                intakeSubsystem.setRawPower(power.get() * 1);
            }
//            if (isEnabled) {
//                double timeMilli = elapsedTime.milliseconds();
//                if (Math.abs(power.get()) > 0.2) {
//                    intakeSubsystem.runWithEncoders();
//                    intakeSubsystem.changeTargetPos(power.get() * ((timeMilli - lastTimeMilli) * (intakeSubsystem.manualTicksPerSecond / 1000.0)));
//                    intakeSubsystem.armGoToTarget();
//                }
//                lastTimeMilli = timeMilli;
//            }
        }


        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            currentInstance = null; // Clear the static reference when the command ends
        }


        public static void endCommand() {
            if (currentInstance != null) {
                CommandScheduler.getInstance().cancel(currentInstance);
            }
        }

    }


    public static class SetPowerCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        double power;

        public SetPowerCmd(IntakeSubsystem intakeSubsystem, double power) {
            this.intakeSubsystem = intakeSubsystem;
            this.power = power;
        }

        @Override
        public void initialize() {
            intakeSubsystem.setRawPower(power);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class Wait extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        ElapsedTime runtime = new ElapsedTime();
        final double duration;
        double startTime;

        //power from -1 to 1
        public Wait(IntakeSubsystem intakeSubsystem, double duration) {
            this.intakeSubsystem = intakeSubsystem;
            this.duration = duration;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void initialize() {
            startTime = runtime.seconds();
        }

        @Override
        public boolean isFinished() {
            return (runtime.seconds() - startTime) > duration;
        }
    }

    //if doesnt work check if everything needs to not have addRequirements(subsystem);
    public static class StartIntakeCmd extends SequentialCommandGroup {
        private final int pos = 1200;

        public StartIntakeCmd(IntakeSubsystem subsystem) {
            StartIntake(subsystem, false, pos);
        }

        public StartIntakeCmd(IntakeSubsystem subsystem, boolean auto) {
            StartIntake(subsystem, auto, pos);
        }

        public StartIntakeCmd(IntakeSubsystem subsystem, boolean auto, int position) {
            StartIntake(subsystem, auto, position);
        }

        public StartIntakeCmd(IntakeSubsystem subsystem, Supplier<Integer> position, Supplier<Boolean> magniv) {
//            StartIntake(subsystem, auto, position);
            double angle = 0.5;
            addCommands(
//                    new ClawStageCmd(subsystem, ClawStages.UPPER),
                    new SetRotationCmd(subsystem, angle),
                    new SlideGotoCmd(subsystem, position),
                    new ClawStageCmd(subsystem, ClawStages.LOWER));


        }


        private void StartIntake(IntakeSubsystem subsystem, boolean auto, int position) {
            if (auto) {
                addCommands(
//                        new ClawStageCmd(subsystem, ClawStages.UPPER),
                        new SetRotationCmd(subsystem, 0.5),
                        new SlideGotoCmd(subsystem, position),
                        new ClawStageCmd(subsystem, ClawStages.INTAKE)
                );

            } else {
                addCommands(
//                        new ClawStageCmd(subsystem, ClawStages.UPPER),
                        new SetRotationCmd(subsystem, 0.5),
                        new SlideUntilCmd(subsystem, position, 1, true),
                        new WaitCommand(1000),
                        new ClawStageCmd(subsystem, ClawStages.INTAKE)
                );
                addRequirements(subsystem);
            }
        }


        @Override
        public void initialize() {
            super.initialize();
            IntakeManualGoToCmd.setEnabled(false);
        }

        @Override
        public void end(boolean interrupted) {
            IntakeManualGoToCmd.setEnabled(true);
        }
    }

    public static class SlideUntilCmd extends CommandBase {
        IntakeSubsystem subsystem;
        final int position;
        final int maxPosition = 3000;
        double power;
        boolean wait;
        final double ratio = (435.0 / 1150.0) * (58.0 / 46.0);

        public SlideUntilCmd(IntakeSubsystem subsystem, int position, double power, boolean wait) {
            this.power = power;
            this.subsystem = subsystem;
            this.wait = wait;
            if (position > maxPosition) {
                position = maxPosition;
            } else if (position < 0) {
                position = 0;
            }
            this.position = (int) (position * ratio);
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
            subsystem.runWithoutEncoders();
            IntakeManualGoToCmd.setEnabled(false);
            if (subsystem.getMotorPosition() < position) {
                subsystem.setRawPower(power);
            }

        }

        @Override
        public boolean isFinished() {
            return (subsystem.getMotorPosition() > position) || !wait;
        }

        @Override
        public void end(boolean interrupted) {
            subsystem.setArmPower(0);
//            subsystem.setTargetPos(subsystem.getMotorPosition());
//            subsystem.armGoToTarget();
//            subsystem.runWithEncoders();
            IntakeManualGoToCmd.setEnabled(true);
        }
    }

    public static class RestartIntakeCmd extends SequentialCommandGroup {
        public RestartIntakeCmd(IntakeSubsystem subsystem) {
            addCommands(
                    new CloseScrewCmd(subsystem, true),
                    new ClawStageCmd(subsystem, ClawStages.INTAKE)
            );
            addRequirements(subsystem);
        }

    }

    //public static class ManualIntakeCmd extends ParallelCommandGroup {
    //    public ManualIntakeCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> power, Supplier<Double> position) {
    //        addCommands
    //                (new SetZRotationSupplierCmd(intakeSubsystem, position),
    //                        new IntakeManualGoToCmd(intakeSubsystem, power));
    //        addRequirements(intakeSubsystem);
    //    }
    //}

    public static class SampleSubmIntakeCmd extends SequentialCommandGroup {
        public SampleSubmIntakeCmd(IntakeSubsystem intakeSubsystem) {
            this(intakeSubsystem, 0.5);
        }

        public SampleSubmIntakeCmd(IntakeSubsystem intakeSubsystem, double angle) {
            addCommands(
                    new ClawStageCmd(intakeSubsystem, ClawStages.LOWER),
                    new WaitCommand(250),
                    new SetRotationCmd(intakeSubsystem, angle),
                    new WaitCommand((long) Math.abs(angle - 0.5) * 500),
                    new OpenScrewCmd(intakeSubsystem, true),
                    new ClawStageCmd(intakeSubsystem, ClawStages.ROTATE_HEIGHT),
                    //new WaitCommand(500),
                    new RotateBackCmd(intakeSubsystem),
                    new WaitCommand((long) Math.abs(angle - 0.5) * 500),
                    new ClawStageCmd(intakeSubsystem, ClawStages.INTAKE)
            );
            addRequirements(intakeSubsystem);
        }

        public SampleSubmIntakeCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> angle) {
            double position = angle.get() + 90;
            double wanted = ((position / 180 - 0.5) * 2 / 3 + 0.5);

            addCommands(
                    new ClawStageCmd(intakeSubsystem, ClawStages.LOWER),
                    new WaitCommand(350),
                    new SetRotationCmd(intakeSubsystem, angle),
                    new WaitCommand((long) wanted * 1500),
                    new OpenScrewCmd(intakeSubsystem, true),
                    new ClawStageCmd(intakeSubsystem, ClawStages.ROTATE_HEIGHT),
                    //new WaitCommand(100),
                    new RotateBackCmd(intakeSubsystem),
                    new WaitCommand((long) wanted * 750),
                    new ClawStageCmd(intakeSubsystem, ClawStages.INTAKE)
            );
            addRequirements(intakeSubsystem);
        }


    }

    public static class SampleGroundIntakeCmd extends SequentialCommandGroup {
        public SampleGroundIntakeCmd(IntakeSubsystem intakeSubsystem) {
            final double
                    spinPower = 1,
                    middleTime = 0.5,
                    grabbingTime = 0.3,
                    holdingPower = 0.07;


            addCommands(
                    new ClawStageCmd(intakeSubsystem, ClawStages.LOWER),
                    new WaitCommand(100),
                    new SetRotationCmd(intakeSubsystem, 0.5),
                    new OpenScrewCmd(intakeSubsystem, true),
                    new RotateBackCmd(intakeSubsystem),
                    new ClawStageCmd(intakeSubsystem, ClawStages.INTAKE));
            addRequirements(intakeSubsystem);
        }
    }//todo: remove later


    public static class ReturnArmForTransferCmd extends SequentialCommandGroup {
        public ReturnArmForTransferCmd(IntakeSubsystem intakeSubsystem, boolean initTime) {

            addCommands(
                    new ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
//                    new Wait(intakeSubsystem, 0.2),
                    new ParallelCommandGroup(
                            new SlideHomeCmd(intakeSubsystem, initTime),
                            new SetRotationCmd(intakeSubsystem, 0.5)
                    ));


            addRequirements(intakeSubsystem);
        }

    }

    public static class ReturnArmForAutoTransferCmd extends SequentialCommandGroup {
        public ReturnArmForAutoTransferCmd(IntakeSubsystem intakeSubsystem, boolean initTime) {

            addCommands(
                    new RotateBackCmd(intakeSubsystem),
                    new ClawStageCmd(intakeSubsystem, ClawStages.UPPER),
                    //new Wait(intakeSubsystem, 0),
                    new ParallelCommandGroup(
                            new SlideHomeCmd(intakeSubsystem, initTime),
                            new SetRotationCmd(intakeSubsystem, 0.5)
                    ));
            addRequirements(intakeSubsystem);
        }

    }


    public static class Transfer extends SequentialCommandGroup {
        final int slidesBackAfterTransfer = 5;
        public static boolean transferring = false;
        IntakeSubsystem intakeSubsystem;

        public Transfer(IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addCommands(
                    new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                    new ParallelCommandGroup(
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                            new ReturnArmForTransferCmd(intakeSubsystem, false)),
                    new SetPowerCmd(intakeSubsystem, -0.18),
                    //new Wait(intakeSubsystem, 0.1), //for safety
                    new DischargeGrabCmd(dischargeSubsystem),
                    new WaitCommand(100),
                    new CloseScrewCmd(intakeSubsystem, false),
                    new SetPowerCmd(intakeSubsystem, 0),
                    new SlideUntilCmd(intakeSubsystem, intakeSubsystem.minSlidesPos + slidesBackAfterTransfer + 40, 0.4, false)
            );
            addRequirements(intakeSubsystem, dischargeSubsystem); //may be unnecessary
        }


        @Override
        public void initialize() {
            super.initialize();
            transferring = true;
            DischargeCommands.MotorControl.setMode(DischargeCommands.MotorControl.Mode.DO_NOTHING);
        }

        @Override
        public void end(boolean interrupted) {
            transferring = false;
            intakeSubsystem.setArmPower(0);
        }
    }

    public static class AutoTransfer extends SequentialCommandGroup {
        final int slidesBackAfterTransfer = 10;
        public static boolean transferring = false;
        IntakeSubsystem intakeSubsystem;

        public AutoTransfer(IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            addCommands(
                    new DischargeCommands.DischargeReleaseCmd(dischargeSubsystem),
                    new ParallelCommandGroup(
                            new DischargeCommands.GoHomeCmd(dischargeSubsystem),
                            new ReturnArmForAutoTransferCmd(intakeSubsystem, false)),
                    new SetPowerCmd(intakeSubsystem, -0.18),
                    //new Wait(intakeSubsystem, 0.1), //for safety
                    new DischargeGrabCmd(dischargeSubsystem),
                    new CloseScrewCmd(intakeSubsystem, true),
                    new SetPowerCmd(intakeSubsystem, 0),
                    new SlideUntilCmd(intakeSubsystem, intakeSubsystem.minSlidesPos + slidesBackAfterTransfer, 0.4, false)
            );
            addRequirements(intakeSubsystem, dischargeSubsystem); //may be unnecessary
        }


        @Override
        public void initialize() {
            super.initialize();
            transferring = true;
            DischargeCommands.MotorControl.setMode(DischargeCommands.MotorControl.Mode.DO_NOTHING);
        }

        @Override
        public void end(boolean interrupted) {
            transferring = false;
            intakeSubsystem.setArmPower(0);
        }
    }

    public static class WaitForTransferEnd extends CommandBase {
        public WaitForTransferEnd() {

        }


        @Override
        public boolean isFinished() {

            return !Transfer.transferring;
        }
    }


    public static class SetPositionCmd extends CommandBase {
        IntakeSubsystem intakeSubsystem;
        int pos;

        public SetPositionCmd(IntakeSubsystem intakeSubsystem, int pos) {
            this.pos = pos;
            this.intakeSubsystem = intakeSubsystem;
        }

        @Override
        public void initialize() {
            intakeSubsystem.setPositionCorrection(1000);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class DontKillYourselfCmd extends SequentialCommandGroup {

        public DontKillYourselfCmd(IntakeSubsystem intakeSubsystem, int pos) {

            addCommands(new SetPositionCmd(intakeSubsystem, pos),
                    new StartIntakeCmd(intakeSubsystem));

        }
    }

    public static class IntakeShakeCmd extends CommandBase {
        double power, interval;
        ElapsedTime runtime = new ElapsedTime();
        IntakeSubsystem intakeSubsystem;
        int switched = 1;

        public IntakeShakeCmd(IntakeSubsystem intakeSubsystem, double power, double interval) {
            this.interval = interval;
            this.power = power;
            this.intakeSubsystem = intakeSubsystem;
            addRequirements(intakeSubsystem);
        }

        @Override
        public void execute() {
            if (runtime.seconds() <= interval) {
                intakeSubsystem.setArmPower(power * switched);
            } else {
                runtime.reset();
                switched *= -1;
            }
        }
    }
}