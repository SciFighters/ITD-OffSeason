package org.firstinspires.ftc.teamcode.season.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

public class EchoRed extends Echo {
    @Override
    public void initialize() {
        super.initialize();
        pipeline = Pipelines.RED;
    }
}