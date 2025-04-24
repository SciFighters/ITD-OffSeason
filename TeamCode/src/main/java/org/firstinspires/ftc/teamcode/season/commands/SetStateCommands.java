package org.firstinspires.ftc.teamcode.season.commands;

import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.teamcode.season.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.season.teleop.EchoRed;

public class SetStateCommands {
    public static class NoneStateCmd extends CommandBase {
        @Override
        public void initialize() {
            EchoRed.setRobotState(RobotState.NONE);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class ClimbStateCmd extends CommandBase {
        @Override
        public void initialize() {
            EchoRed.setRobotState(RobotState.CLIMB);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }


    public static class BasketStateCmd extends CommandBase {
        @Override
        public void initialize() {
            EchoRed.setRobotState(RobotState.BASKET);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class ChamberStateCmd extends CommandBase {
        @Override
        public void initialize() {
            EchoRed.setRobotState(RobotState.CHAMBER);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class HPIntakeStateCmd extends CommandBase {
        @Override
        public void initialize() {
            EchoRed.setRobotState(RobotState.HPINTAKE);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class AutoChamberStateCmd extends CommandBase {
        @Override
        public void initialize() {
            EchoRed.setRobotState(RobotState.AUTOCHAMBER);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class AutoIntakeStateCmd extends CommandBase {
        @Override
        public void initialize() {
            EchoRed.setRobotState(RobotState.AUTOINTAKE);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public static class IntakeStateCmd extends CommandBase {
        @Override
        public void initialize() {
            EchoRed.setRobotState(RobotState.INTAKE);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }


}
