package org.firstinspires.ftc.teamcode.season.gandalf;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "KiwiDriveOpMode", group = "LOTR")
public class GandalfsBeard extends LinearOpMode {
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    private DcMotor conveyorMotor;

    public BNO055IMU imu;
    private KiwiDrive kiwiDrive;
    private double timeOnPress;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //-2 to avoid auto running at the start
        timeOnPress = -2;

        //conveyor initializing
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");

        //imu initializing
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);

        //kiwidrive initializing
        kiwiDrive = new KiwiDrive(hardwareMap.get(DcMotor.class, "wheel1"),
                hardwareMap.get(DcMotor.class, "wheel2"),
                hardwareMap.get(DcMotor.class, "wheel3"), imu);

        telemetry.addData("init", "finished");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {

            //driving stuff
            kiwiDrive.setBoost(gamepad1.left_bumper, gamepad1.right_bumper);
            kiwiDrive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            kiwiDrive.dpadMovement(gamepad1.dpad_down, gamepad1.dpad_up, gamepad1.dpad_left,
                    gamepad1.dpad_right, gamepad1.right_stick_x);

            //conveyor stuff
            if (gamepad2.x) {
                timeOnPress = runtime.seconds();
            } else if (gamepad2.a) {
                timeOnPress = -2;
                conveyorMotor.setPower(-1);
            } else {
                conveyorMotor.setPower(0);
            }
            if (timeOnPress + 1 >= runtime.seconds()) {
                conveyorMotor.setPower(-1);
            }

            //telemetry
            telemetry.addData("wheel 1 power", kiwiDrive.wheel1Power);
            telemetry.addData("wheel 2 power", kiwiDrive.wheel2Power);
            telemetry.addData("wheel 3 power", kiwiDrive.wheel3Power);
            telemetry.addData("left stick x", gamepad1.left_stick_x);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("get heading", kiwiDrive.getHeading());
            telemetry.update();
        }
    }

}