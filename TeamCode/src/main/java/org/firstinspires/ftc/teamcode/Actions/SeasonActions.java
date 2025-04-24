package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.commands.DischargeCommands.*;
import org.firstinspires.ftc.teamcode.season.commands.IntakeCommands.*;
import org.firstinspires.ftc.teamcode.season.subsystems.ClawStages;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

public class SeasonActions {
    public static Action openScrew(IntakeSubsystem intakeSubsystem, boolean wait) {
        return new CommandAction(new OpenScrewCmd(intakeSubsystem, wait));
    }

    public static Action closeScrew(IntakeSubsystem intakeSubsystem, boolean wait) {
        return new CommandAction(new CloseScrewCmd(intakeSubsystem, wait));
    }

    public static Action setClawStage(IntakeSubsystem intakeSubsystem, double stage) {
        return new CommandAction(new ClawStageCmd(intakeSubsystem, stage));
    }

    public static Action slideHome(IntakeSubsystem intakeSubsystem, boolean initTime) {
        return new CommandAction(new SlideHomeCmd(intakeSubsystem, initTime));
    }

    public static Action setRotation(IntakeSubsystem intakeSubsystem, double position) {
        return new CommandAction(new SetRotationCmd(intakeSubsystem, position));
    }

    public static Action setPower(IntakeSubsystem intakeSubsystem, double power) {
        return new CommandAction(new SetPowerCmd(intakeSubsystem, power));
    }

    public static Action slideUntil(IntakeSubsystem intakeSubsystem, int position, double power, boolean wait) {
        return new CommandAction(new SlideUntilCmd(intakeSubsystem, position, power, wait));
    }

    public static Action slideGoto(IntakeSubsystem subsystem, int position) {
        return new CommandAction(new SlideGotoCmd(subsystem, position));
    }

    public static Action dischargeRelease(DischargeSubsystem dischargeSubsystem) {
        return new CommandAction(new DischargeReleaseCmd(dischargeSubsystem));
    }

    public static Action dischargeGrab(DischargeSubsystem dischargeSubsystem) {
        return new CommandAction(new DischargeGrabCmd(dischargeSubsystem));
    }

    public static class GoHome implements Action {
        DischargeSubsystem dischargeSubsystem;
        int lastTick;
        //        double lastTime = 0;
        double maxDuration;
        final int minTargetOffset = 50;
        ElapsedTime elapsedTime = new ElapsedTime();
        double downTimer = 0;
        boolean switched = false;
        boolean initialized = false;

        public GoHome(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            maxDuration = 4;
        }

        public GoHome(DischargeSubsystem dischargeSubsystem, double maxDuration) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.maxDuration = maxDuration;
        }


        public void initialize() {
            MotorControl.setMode(MotorControl.Mode.OFF);
            lastTick = dischargeSubsystem.getPosition();
            elapsedTime.reset();
            dischargeSubsystem.setPower(0);
            dischargeSubsystem.runWithoutEncoders();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                initialize();
                initialized = true;
            }
            MotorControl.setMode(MotorControl.Mode.OFF);
            double curPos = dischargeSubsystem.getPosition();
            if (curPos < 160) {
                dischargeSubsystem.setRawPower(-dischargeSubsystem.slidesLowSpeed);
            } else if (curPos > 350)
                dischargeSubsystem.setRawPower(-dischargeSubsystem.slidesSpeed);
            else
                dischargeSubsystem.setRawPower(-dischargeSubsystem.slidesHalfSpeed);
            if(isFinished()){
                end(false);
                return false;
            }
            return true;

        }


        public boolean isFinished() {
            return (dischargeSubsystem.isHome() || (elapsedTime.seconds() > maxDuration) || dischargeSubsystem.getCurrent() > 8);//
        }


        public void end(boolean interrupted) {
            dischargeSubsystem.setPower(0);
            MotorControl.setMode(MotorControl.Mode.DO_NOTHING);

            if (dischargeSubsystem.getGearBoxRatio() == dischargeSubsystem.dischargeRatio && !interrupted) {
                dischargeSubsystem.minLiftPos = dischargeSubsystem.getPosition() + minTargetOffset;
                dischargeSubsystem.setTargetPosInTicks(dischargeSubsystem.getPosition() + minTargetOffset);
                dischargeSubsystem.resetEncoders();
            }
        }


    }




    public static class GoToTargetWait implements Action {
        boolean initialized = false;
        private final DischargeSubsystem dischargeSubsystem;
        private int target;
        Supplier<Integer> targetSupplier;
        final boolean supplier;
        int waitDistance;

        public GoToTargetWait(DischargeSubsystem dischargeSubsystem, int target) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.target = target;
            supplier = false;
            waitDistance = 60;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                MotorControl.setTargetPosition(target);
                MotorControl.setMode(MotorControl.Mode.GO_TO_TARGET);
                initialized = true;
            }
            return !(Math.abs(MotorControl.getTargetPosition() - dischargeSubsystem.getLiftPosInCM()) < 60);
        }
    }

    public static class GoToTarget implements Action {
        boolean initialized = false;
        private final DischargeSubsystem dischargeSubsystem;
        private int target;
        Supplier<Integer> targetSupplier;
        final boolean supplier;
        int waitDistance;

        public GoToTarget(DischargeSubsystem dischargeSubsystem, int target) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.target = target;
            supplier = false;
            waitDistance = 60;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            MotorControl.setTargetPosition(target);
            MotorControl.setMode(MotorControl.Mode.GO_TO_TARGET);

            return false;
        }
    }

    public static SequentialAction chamberDischarge(DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
        return new SequentialAction(
                new GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight - 205),
                dischargeRelease(dischargeSubsystem)
        );
    }

    public static SequentialAction returnArmForTransfer(IntakeSubsystem intakeSubsystem, Boolean initTime) {
        return new SequentialAction(
                setClawStage(intakeSubsystem, ClawStages.UPPER),
                new ParallelAction(
                        slideHome(intakeSubsystem, initTime),
                        setRotation(intakeSubsystem, 0.5)
                )
        );
    }

    public static SequentialAction Transfer(IntakeSubsystem intakeSubsystem, DischargeSubsystem dischargeSubsystem) {
        return new SequentialAction(
                dischargeRelease(dischargeSubsystem),
                new ParallelAction(
                        new GoHome(dischargeSubsystem),
                        returnArmForTransfer(intakeSubsystem, false)
                ),
                setPower(intakeSubsystem, -0.18),
                dischargeGrab(dischargeSubsystem),
                new SleepAction(0.1),
                closeScrew(intakeSubsystem, false),
                setPower(intakeSubsystem, 0),
                slideUntil(intakeSubsystem, intakeSubsystem.minSlidesPos + 45, 0.4, false)

        );
    }

    public static SequentialAction SampleSubmIntakeCmd(IntakeSubsystem intakeSubsystem) {
        return SampleSubmIntakeCmd(intakeSubsystem, 0.5);
    }

    public static SequentialAction SampleSubmIntakeCmd(IntakeSubsystem intakeSubsystem, double angle) {
        return new SequentialAction(
                setClawStage(intakeSubsystem, ClawStages.LOWER),
                new SleepAction(0.25),
                setRotation(intakeSubsystem, angle),
                new SleepAction(Math.abs(angle - 0.5) * 0.5),
                openScrew(intakeSubsystem, true),
                setClawStage(intakeSubsystem, ClawStages.ROTATE_HEIGHT),
                setRotation(intakeSubsystem, 0.5),
                new SleepAction(Math.abs(angle - 0.5) * 0.5),
                setClawStage(intakeSubsystem, ClawStages.INTAKE)
        );
    }

    public static SequentialAction startIntake(IntakeSubsystem intakeSubsystem, int position) {
        return new SequentialAction(
                setRotation(intakeSubsystem, 0.5),
                slideGoto(intakeSubsystem, position),
                setClawStage(intakeSubsystem, ClawStages.INTAKE)
        );
    }

    public static class MotorControl implements Action {


        public enum Mode {DO_NOTHING, GO_TO_TARGET, MANUAL_MOVEMENT, STAY_STILL, OFF}

        private final DischargeSubsystem dischargeSubsystem;
        private final Supplier<Double> manualPower;
        private final boolean allowManualTargetAdjustment;
        private final Telemetry telemetry;

        public static Mode mode = Mode.DO_NOTHING;
        private static int targetPosition = 0;
        private static int stayStillTarget = 0;

        private final double goToKp = 5;
        private final double stayStillKp = 4;
        //        private final double stayStillKi = 0.01;
//        private double Integral = 0;
//        private double lastTime = 0;
        private final double goToMin = 0.08;
        private final double stayStillMin = 0.08;
        boolean initialized = false;
//        private ElapsedTime elapsedTime = new ElapsedTime();

        public MotorControl(DischargeSubsystem dischargeSubsystem, Supplier<Double> manualPower,
                            boolean allowManualTargetAdjustment, Telemetry telemetry) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.manualPower = manualPower;
            this.allowManualTargetAdjustment = allowManualTargetAdjustment;
            this.telemetry = telemetry;
            initialized = false;
        }

        public void initialize() {
            mode = MotorControl.Mode.DO_NOTHING;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            telemetry.addData("mode",mode);
            if (!initialized) {
                initialize();
                initialized = true;
            }
            double manual = manualPower.get();
            int currentPosition = (int) dischargeSubsystem.getLiftPosInCM();
            double error = targetPosition - currentPosition;

            switch (mode) {
                case OFF:
                    break;

                case DO_NOTHING:
                    dischargeSubsystem.setPower(0);
                    if (manual > 0.15) {
                        mode = MotorControl.Mode.MANUAL_MOVEMENT;
                    }
                    break;

                case GO_TO_TARGET:
                    if (allowManualTargetAdjustment && Math.abs(manual) > 0.2) {
                        targetPosition += manual; // Adjust target dynamically
                    }
                    double pidOutput = calculateGoToTargetPID(error, currentPosition);
                    dischargeSubsystem.setPower(pidOutput);
                    if (Math.abs(error) <= 20) {
                        mode = MotorControl.Mode.STAY_STILL;
                        stayStillTarget = targetPosition;
//                        lastTime = elapsedTime.seconds();
                    }
                    break;

                case MANUAL_MOVEMENT:
                    dischargeSubsystem.setPower(-manual);
                    if (Math.abs(manual) < 0.15) {
                        mode = MotorControl.Mode.STAY_STILL;
                        stayStillTarget = currentPosition;
//                        lastTime = elapsedTime.seconds();
                    }
                    break;

                case STAY_STILL:
                    if (Math.abs(manual) > 0.2) {
                        mode = MotorControl.Mode.MANUAL_MOVEMENT;
                    }
                    double holdPositionError = stayStillTarget - currentPosition;
                    double holdOutput = calculateStayStillPID(holdPositionError);
                    dischargeSubsystem.setPower(holdOutput);
                    break;
            }
            return true;
        }

        private double calculateGoToTargetPID(double error, int curPos) {
            if (curPos < 80)
                return 0.6;
            if (curPos < 150)
                return 0.75;
            if (Math.abs(error) >= 200)
                return Math.signum(error); // Full power in the direction of the target
            error /= 1000; //normalize error
            return goToKp * error + (Math.signum(error) * goToMin);
        }

        private double calculateStayStillPID(double error) {
//            double time = elapsedTime.seconds();
//            Integral += error * stayStillKi * (time - lastTime);
            error /= 1000; //normalize error
//            lastTime = time;
            return stayStillKp * error + (Math.signum(error) * stayStillMin);

        }

        public static void setMode(MotorControl.Mode newMode) {
            mode = newMode;
        }

        public static Mode getMode() {
            return mode;
        }

        public static void setTargetPosition(int target) {
            targetPosition = target;
        }

        public static int getTargetPosition() {
            return targetPosition;
        }

        public static int getStayStillTarget() {
            return stayStillTarget;
        }
    }


}
