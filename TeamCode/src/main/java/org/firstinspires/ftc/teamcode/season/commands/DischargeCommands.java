package org.firstinspires.ftc.teamcode.season.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;

import java.util.function.Supplier;

public class DischargeCommands {

    public static class NoOpCommand extends CommandBase {
        public NoOpCommand(DischargeSubsystem dischargeSubsystem) {
            addRequirements(dischargeSubsystem);
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

//    public static class DischargeManualCmd extends CommandBase {
//        Supplier<Double> power;
//        DischargeSubsystem dischargeSubsystem;
//        Telemetry telemetry;
//        public static boolean isMoving;
//
//
//        public DischargeManualCmd(Supplier<Double> power, DischargeSubsystem dischargeSubsystem,
//                                  Telemetry telemetry) {
//            this.telemetry = telemetry;
//            this.power = power;
//            this.dischargeSubsystem = dischargeSubsystem;
//            addRequirements(dischargeSubsystem);
//        }
//
//        @Override
//        public void initialize() {
//            dischargeSubsystem.runWithEncoders();
//        }
//
//        @Override
//        public void execute() {
//            if (Math.abs(power.get()) > 0.2) {
//                MotorGotoCmd.stayOn = -2;
//                dischargeSubsystem.setPower(power.get());
//                isMoving = true;
//            } else {
//                if (isMoving) {
//                    MotorGotoCmd.stayOn = (int) dischargeSubsystem.getLiftPosInCM();
//                }
//                //dischargeSubsystem.setPower(0);
//                isMoving = false;
//            }
//        }
//    }
//
//    public static class MotorGotoCmd extends CommandBase {
//        DischargeSubsystem dischargeSubsystem;
//        Telemetry telemetry;
//        private static int targetInCM;
//        int error;
//        final double kp = 0.01;
//        final double minPower = 0.05;
//        public static int stayOn = -2;
//
//        public MotorGotoCmd(DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
//            this.dischargeSubsystem = dischargeSubsystem;
//            this.telemetry = telemetry;
//        }
//
//
//        @Override
//        public void initialize() {
//            if (!IntakeCommands.Transfer.transferring) {
//                dischargeSubsystem.runWithEncoders();
//            }
//        }
//
//        @Override
//        public void execute() {
//            if (stayOn == -1) {
//                setPidPower(targetInCM);
//            } else if (stayOn == -2) {
//            } else {
//                setPidPower(stayOn);
//            }
//        }
//
//        private void setPidPower(int target) {
//            error = target - dischargeSubsystem.getPosition();
//            if (Math.abs(error) >= 200) {
//                dischargeSubsystem.setPower(1 * Math.signum(error));
//            } else {
//                dischargeSubsystem.setPower(Math.max(error * kp, minPower));
//            }
//        }
//
//        @Override
//        public boolean isFinished() {
//            return false;
//        }
//
//    }
//
//    public static class DischargeGotoCmd extends CommandBase {
//        DischargeSubsystem dischargeSubsystem;
//        Telemetry telemetry;
//        private final int targetInCM;
//        boolean finish = false;
//        int tolerance = 20;
//
//        public DischargeGotoCmd(DischargeSubsystem dischargeSubsystem, int targetInCM, Telemetry telemetry) {
//            this.dischargeSubsystem = dischargeSubsystem;
//            this.targetInCM = targetInCM;
//            this.telemetry = telemetry;
//            addRequirements(dischargeSubsystem);
//        }
//
//
//        @Override
//        public void initialize() {
//            if (IntakeCommands.Transfer.transferring) {
//                finish = true;
//            } else {
//                dischargeSubsystem.runWithEncoders();
//                MotorGotoCmd.targetInCM = this.targetInCM;
//            }
//        }
//
//
//        @Override
//        public boolean isFinished() {
//            //tolerance in both gears
//            finish = Math.abs(dischargeSubsystem.getLiftPosInCM() - targetInCM) <= tolerance;
//            return finish;
//        }
//
//    }


    public static class MotorControl extends CommandBase {
        public enum Mode {DO_NOTHING, GO_TO_TARGET, MANUAL_MOVEMENT, STAY_STILL, OFF}

        private final DischargeSubsystem dischargeSubsystem;
        private final Supplier<Double> manualPower;
        private final boolean allowManualTargetAdjustment;
        private final Telemetry telemetry;

        private static Mode mode = Mode.DO_NOTHING;
        private static int targetPosition = 0;
        private static int stayStillTarget = 0;

        private final double goToKp = 5;
        private final double stayStillKp = 4;
        //        private final double stayStillKi = 0.01;
//        private double Integral = 0;
//        private double lastTime = 0;
        private final double goToMin = 0.08;
        private final double stayStillMin = 0.08;
//        private ElapsedTime elapsedTime = new ElapsedTime();

        public MotorControl(DischargeSubsystem dischargeSubsystem, Supplier<Double> manualPower,
                            boolean allowManualTargetAdjustment, Telemetry telemetry) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.manualPower = manualPower;
            this.allowManualTargetAdjustment = allowManualTargetAdjustment;
            this.telemetry = telemetry;
        }

        @Override
        public void initialize() {
            mode = Mode.DO_NOTHING;
        }

        @Override
        public void execute() {
            double manual = manualPower.get();
            int currentPosition = (int) dischargeSubsystem.getLiftPosInCM();
            double error = targetPosition - currentPosition;

            switch (mode) {
                case OFF:
                    break;

                case DO_NOTHING:
                    dischargeSubsystem.setPower(0);
                    if (manual > 0.15) {
                        mode = Mode.MANUAL_MOVEMENT;
                    }
                    break;

                case GO_TO_TARGET:
                    if (allowManualTargetAdjustment && Math.abs(manual) > 0.2) {
                        targetPosition += manual; // Adjust target dynamically
                    }
                    double pidOutput = calculateGoToTargetPID(error, currentPosition);
                    dischargeSubsystem.setPower(pidOutput);
                    if (Math.abs(error) <= 20) {
                        mode = Mode.STAY_STILL;
                        stayStillTarget = targetPosition;
//                        lastTime = elapsedTime.seconds();
                    }
                    break;

                case MANUAL_MOVEMENT:
                    dischargeSubsystem.setPower(-manual);
                    if (Math.abs(manual) < 0.15) {
                        mode = Mode.STAY_STILL;
                        stayStillTarget = currentPosition;
//                        lastTime = elapsedTime.seconds();
                    }
                    break;

                case STAY_STILL:
                    if (Math.abs(manual) > 0.2) {
                        mode = Mode.MANUAL_MOVEMENT;
                    }
                    double holdPositionError = stayStillTarget - currentPosition;
                    double holdOutput = calculateStayStillPID(holdPositionError);
                    dischargeSubsystem.setPower(holdOutput);
                    break;
            }
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

        public static void setMode(Mode newMode) {
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

    public static class GoToTarget extends CommandBase {
        private final int target;

        public GoToTarget(DischargeSubsystem dischargeSubsystem, int target) {
            this.target = target;
//            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            MotorControl.setMode(MotorControl.Mode.GO_TO_TARGET);
            MotorControl.setTargetPosition(target);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class GoToTargetWait extends CommandBase {
        private final DischargeSubsystem dischargeSubsystem;
        private int target;
        Supplier<Integer> targetSupplier;
        final boolean supplier;
        boolean chamber = false;
        int waitDistance;

        public GoToTargetWait(DischargeSubsystem dischargeSubsystem, int target) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.target = target;
            supplier = false;
            waitDistance = 60;
            addRequirements(dischargeSubsystem);
        }

        public GoToTargetWait(DischargeSubsystem dischargeSubsystem, int target, int waitDistance) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.target = target;
            supplier = false;
            this.waitDistance = waitDistance;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {

            MotorControl.setTargetPosition(target);
            MotorControl.setMode(MotorControl.Mode.GO_TO_TARGET);

        }

        @Override
        public boolean isFinished() {
            return Math.abs(MotorControl.getTargetPosition() - dischargeSubsystem.getLiftPosInCM()) < 60;
        }
    }


    public static class GoHomeCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        int lastTick;
        //        double lastTime = 0;
        double maxDuration;
        final int minTargetOffset = 50;
        ElapsedTime elapsedTime = new ElapsedTime();
        double downTimer = 0;
        boolean switched = false;

        public GoHomeCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            maxDuration = 4;
            addRequirements(dischargeSubsystem);
        }

        public GoHomeCmd(DischargeSubsystem dischargeSubsystem, double maxDuration) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.maxDuration = maxDuration;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            MotorControl.setMode(MotorControl.Mode.OFF);
            lastTick = dischargeSubsystem.getPosition();
            elapsedTime.reset();
            dischargeSubsystem.setPower(0);
            dischargeSubsystem.runWithoutEncoders();
        }

        @Override
        public void execute() {
            MotorControl.setMode(MotorControl.Mode.OFF);
            double curPos = dischargeSubsystem.getPosition();
            if (curPos < 160) {
                dischargeSubsystem.setRawPower(-dischargeSubsystem.slidesLowSpeed);
            } else if (curPos > 350)
                dischargeSubsystem.setRawPower(-dischargeSubsystem.slidesSpeed);
            else
                dischargeSubsystem.setRawPower(-dischargeSubsystem.slidesHalfSpeed);
//            if (dischargeSubsystem.getPosition() < 200 && !switched) {
//                maxDuration = 2;
//                elapsedTime.reset();
//                switched = true;
//            }
//            if (downTimer == 0 && curPos < 200)
//                downTimer = elapsedTime.seconds();


        }

        @Override
        public boolean isFinished() {
            return (dischargeSubsystem.isHome() || (elapsedTime.seconds() > maxDuration) || dischargeSubsystem.getCurrent() > 8);//
//            if (dischargeSubsystem.getGearBoxRatio() == 1) { // finish check for normal gear
//                if (elapsedTime.seconds() > maxDuration) {
//                    return true;
//                }
//                if (elapsedTime.seconds() < 0.4)
//                    return false;
//
//                double deltaTime = elapsedTime.seconds() - lastTime;
//                if (deltaTime > 0.2) {
//                    int deltaTick = dischargeSubsystem.getPosition() - lastTick;
//                    lastTick = dischargeSubsystem.getPosition();
//                    lastTime = elapsedTime.seconds();
//                    if (Math.abs(deltaTick) <= 5) {
//                        return true;
//                    }
//
//                }
//            }
//            else { // finish check for normal gear
//                if (dischargeSubsystem.getLiftPosInCM() < dischargeSubsystem.minClimbLiftPos){
//                    return true;
//                }
//            }
//            return false;
        }

        @Override
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

    public static class DischargeGrabCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public DischargeGrabCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
//            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.holdSample();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class DischargeReleaseCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public DischargeReleaseCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.releaseSample();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class DischargeHalfReleaseCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public DischargeHalfReleaseCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.halfReleaseSample();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }


    public static class DischargeClawTestCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;
        Telemetry telemetry;
        Supplier<Double> ClawPos;

        public DischargeClawTestCmd(Supplier<Double> ClawPos, DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
            this.dischargeSubsystem = dischargeSubsystem;
            this.telemetry = telemetry;
            this.ClawPos = ClawPos;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void execute() {
            double c = (ClawPos.get() + 1) / 2;
            dischargeSubsystem.testClaw(c);
            telemetry.addData("claw pos (joy stick)", c);
            telemetry.addData("claw pos (servo)", dischargeSubsystem.getClawServoPosition());
        }
    }

    public static class GearBoxClimbCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public GearBoxClimbCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.climbMode();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class GearBoxDischargeCmd extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public GearBoxDischargeCmd(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
            addRequirements(dischargeSubsystem);
        }

        @Override
        public void initialize() {
            dischargeSubsystem.dischargeMode();
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class ChamberDischargeCmd extends SequentialCommandGroup {
        public ChamberDischargeCmd(DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
            addCommands(
                    new GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight - 205),
                    //new WaitCommand(100),
                    new DischargeReleaseCmd(dischargeSubsystem), new WaitCommand(200),
                    new GoHomeCmd(dischargeSubsystem));
            addRequirements(dischargeSubsystem);
        }
    }

    public static class AutoChamberDischargeCmd extends SequentialCommandGroup {
        public AutoChamberDischargeCmd(DischargeSubsystem dischargeSubsystem, Telemetry telemetry) {
            addCommands(
                    new GoToTargetWait(dischargeSubsystem, dischargeSubsystem.highChamberHeight - 205),
                    //new WaitCommand(100),
                    new DischargeReleaseCmd(dischargeSubsystem));
            addRequirements(dischargeSubsystem);
        }
    }

    public static class ResetDischarge extends CommandBase {
        DischargeSubsystem dischargeSubsystem;

        public ResetDischarge(DischargeSubsystem dischargeSubsystem) {
            this.dischargeSubsystem = dischargeSubsystem;
        }

        @Override
        public void initialize() {
            dischargeSubsystem.resetEncoders();
//            dischargeSubsystem.runToPosition();
        }
    }

    public static class SequentialRaceWrapper extends CommandBase {
        private final Command[] commands;
        private int currentCommandIndex = 0;

        public SequentialRaceWrapper(Command... commands) {
            this.commands = commands;
        }

        @Override
        public void initialize() {
            System.out.println("Starting SequentialRaceWrapper");
            if (commands.length > 0) {
                commands[0].initialize();
            }
        }

        @Override
        public void execute() {
            if (currentCommandIndex < commands.length) {
                Command currentCommand = commands[currentCommandIndex];

                currentCommand.execute();

                if (currentCommand.isFinished()) {
                    currentCommand.end(false); // Cleanly finish the command
                    currentCommandIndex++;

                    if (currentCommandIndex < commands.length) {
                        commands[currentCommandIndex].initialize();
                    }
                }
            }
        }

        @Override
        public boolean isFinished() {
            return currentCommandIndex >= commands.length;
        }

        @Override
        public void end(boolean interrupted) {
            System.out.println("SequentialRaceWrapper ended, interrupted? " + interrupted);
            if (interrupted && currentCommandIndex < commands.length) {
                commands[currentCommandIndex].end(true);
            }
        }
    }

    public static class HpDischarge extends SequentialCommandGroup {
        public HpDischarge(DischargeSubsystem dischargeSubsystem) {
            addCommands(
                    new GoToTargetWait(dischargeSubsystem, dischargeSubsystem.lowChamberHeight),
                    new DischargeReleaseCmd(dischargeSubsystem),
                    new WaitCommand(600),
                    new GoHomeCmd(dischargeSubsystem));
        }
    }


}
