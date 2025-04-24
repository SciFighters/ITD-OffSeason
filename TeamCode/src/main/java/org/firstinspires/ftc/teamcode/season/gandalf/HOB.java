package org.firstinspires.ftc.teamcode.season.gandalf;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "HobOpMode", group = "LOTR")
public class HOB extends LinearOpMode {

    private boolean canSwitchDrivingMode = true, canSwitchDrivingDirection = true;
    private boolean oneStickDrive = false;
    private boolean allowed = true;

    @Override
    public void runOpMode() throws InterruptedException {

        DriveClass driveClass = new DriveClass(hardwareMap.get(DcMotor.class, "leftWheel"),
                hardwareMap.get(DcMotor.class, "rightWheel"));

        ArmClass armClass = new ArmClass(hardwareMap.dcMotor.get("armMotor"), this);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.a) {
                allowed = false;
            }
            if (gamepad2.b) {
                allowed = true;
            }
            if (allowed) {
                armClass.move(gamepad2.left_trigger, gamepad2.right_trigger);

                if (gamepad2.x) {
                    armClass.isReturnRequested = true;
                }
            }

            if (oneStickDrive) {
                driveClass.drive(gamepad1.left_stick_y, gamepad1.left_stick_x,
                        gamepad1.left_bumper, gamepad1.right_bumper);
            } else {
                driveClass.drive(gamepad1.left_stick_y, gamepad1.right_stick_x,
                        gamepad1.left_bumper, gamepad1.right_bumper);
            }


        }
    }
}
