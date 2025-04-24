package org.firstinspires.ftc.teamcode.season.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.season.subsystems.DischargeSubsystem;

import java.util.function.Supplier;

public class ClimbCommands {

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


    public static class SetWinchPower extends CommandBase {
        ClimbSubsystem climbSubsystem;
        Supplier<Double> rPower;
        Supplier<Double> lPower;

        public SetWinchPower(ClimbSubsystem climbSubsystem, Supplier<Double> rPower, Supplier<Double> lPower) {
            this.climbSubsystem = climbSubsystem;
            this.rPower = rPower;
            this.lPower = lPower;
            addRequirements(climbSubsystem);
        }

        @Override
        public void execute() {
            climbSubsystem.setRServoPower(-rPower.get());
            climbSubsystem.setLServoPower(-lPower.get());
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            climbSubsystem.setRServoPower(0);
            climbSubsystem.setLServoPower(0);
        }
    }

    public static class moveWinchByTime extends CommandBase {
        ClimbSubsystem climbSubsystem;
        String servo;
        int milli;
        ElapsedTime elapsedTime = new ElapsedTime();
        double power;

        public moveWinchByTime(ClimbSubsystem climbSubsystem, double power, String servo, int milli) {
            this.climbSubsystem = climbSubsystem;
            this.power = power;
            this.servo = servo;
            this.milli = milli;
            addRequirements(climbSubsystem);
        }

        @Override
        public void initialize() {
            elapsedTime.reset();
        }

        @Override
        public void execute() {
            if (servo.equals("right"))
                climbSubsystem.setRServoPower(-power);
            else if (servo.equals("left"))
                climbSubsystem.setLServoPower(power);
            else {
                climbSubsystem.setRServoPower(power);
                climbSubsystem.setLServoPower(-power);
            }

        }

        @Override
        public boolean isFinished() {
            return elapsedTime.milliseconds() > milli;
        }

        @Override
        public void end(boolean interrupted) {
            if (servo.equals("right"))
                climbSubsystem.setRServoPower(0);
            else if (servo.equals("left"))
                climbSubsystem.setLServoPower(0);
            else {
                climbSubsystem.setRServoPower(0);
                climbSubsystem.setLServoPower(0);
            }

        }
    }

}
