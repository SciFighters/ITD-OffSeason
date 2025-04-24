package org.firstinspires.ftc.teamcode.season.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.season.IMU_Integrator;
import org.opencv.core.Point;

public class MecanumDrive extends SubsystemBase {
    public DcMotorEx fl, fr, bl, br;
    //    public Servo moverGoGo;
    public Servo moverServo;
    public Servo wentWentServo;
    DistanceSensor distanceSensor;
    MecanumDriveKinematics kinematics;
    IMU imu;
    MecanumDriveWheelSpeeds wheelSpeeds;
    private final BHI260IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
    );
    ;
    IMU_Integrator imuIntegrator;
    //bl = intakeOdometer, br = nonParallel, fr = dischargeOdometer
    ElapsedTime time = new ElapsedTime();
    Pose2d pos;
    double correctedHeading;
    boolean isFieldOriented = true;

    MultipleTelemetry telemetry;
    Point startingPosition;
    double forwardTicksPerMeter = 1817, strafeTicksPerMeter = 2048;
    double tickPerMeter = 1783;
    LinearOpMode opMode;
    double startAngle = 0;
    public double extraX = 0, extraY = 0, extraR = 0;
    double correctionX = 0, correctionY = 0;

    @Override
    public void periodic() {
        super.periodic();

        if(imuIntegrator != null)
            imuIntegrator.update();
    }

    public MecanumDrive(MultipleTelemetry telemetry, HardwareMap hm, LinearOpMode opMode) {
        this.telemetry = telemetry;
        moverServo = hm.get(Servo.class, "movergogo");
        wentWentServo = hm.get(Servo.class, "moverwentwent");
        fl = hm.get(DcMotorEx.class, "fl_motor");
        fr = hm.get(DcMotorEx.class, "fr_motor");
        br = hm.get(DcMotorEx.class, "br_motor");
        bl = hm.get(DcMotorEx.class, "bl_motor");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        distanceSensor = hm.get(DistanceSensor.class, "distanceSensor");
        this.opMode = opMode;
        imu = hm.get(BHI260IMU.class, "imu");


        startingPosition = new Point(0, 0);
        imu.initialize(parameters);

        Translation2d flLocation = new Translation2d(100, 164);//164
        Translation2d frLocation = new Translation2d(100, -164);//-164
        Translation2d brLocation = new Translation2d(-100, -164);//-164
        Translation2d blLocation = new Translation2d(-100, 164);//164
        kinematics = new MecanumDriveKinematics(flLocation, frLocation, blLocation, brLocation);
//        Translation2d OflLocation = new Translation2d(100, 164);
//        Translation2d OfrLocation = new Translation2d(100, -164);
//        Translation2d ObrLocation = new Translation2d(-100, -164);
//        Translation2d OblLocation = new Translation2d(-100, 164);
//        odometryKinematics = new MecanumDriveKinematics(OflLocation,OfrLocation,ObrLocation,OblLocation);


//        odometry = new MecanumDriveOdometry(odometryKinematics, Rotation2d.fromDegrees(getHeading()), new Pose2d(startingPosition.x,startingPosition.y,Rotation2d.fromDegrees(0)));
        time.reset();


    }

    public MecanumDrive(MultipleTelemetry telemetry, HardwareMap hm, Point start, double startAngle, LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = telemetry;
        moverServo = hm.get(Servo.class, "movergogo");
        wentWentServo = hm.get(Servo.class, "moverwentwent");
        fl = hm.get(DcMotorEx.class, "fl_motor");
        fr = hm.get(DcMotorEx.class, "fr_motor");
        br = hm.get(DcMotorEx.class, "br_motor");
        bl = hm.get(DcMotorEx.class, "bl_motor");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        startingPosition = start;
//        distanceSensor = hm.get(DistanceSensor.class, "distanceSensor");
        this.startAngle = startAngle;
//        Translation2d OflLocation = new Translation2d(100, 164);
//        Translation2d OfrLocation = new Translation2d(100, -164);
//        Translation2d ObrLocation = new Translation2d(-100, -164);
//        Translation2d OblLocation = new Translation2d(-100, 164);
//        odometryKinematics = new MecanumDriveKinematics(OflLocation,OfrLocation,ObrLocation,OblLocation);
        Translation2d flLocation = new Translation2d(164, 100);//164
        Translation2d frLocation = new Translation2d(164, -100);//-164
        Translation2d brLocation = new Translation2d(-164, -100);//-164
        Translation2d blLocation = new Translation2d(-164, 100);//164
        kinematics = new MecanumDriveKinematics(flLocation, frLocation, blLocation, brLocation);


        imu = hm.get(BHI260IMU.class, "imu");
        initIMU(imu, opMode);


//        odometry = new MecanumDriveOdometry(odometryKinematics, Rotation2d.fromDegrees(getHeading()), new Pose2d(start.x, start.y, startAngle));
        time.reset();

    }


    private void initIMU(IMU imu, LinearOpMode robot) {
        this.imu = imu;

        imuIntegrator = new IMU_Integrator(imu, startingPosition, startAngle, telemetry, bl, fr, fl);
        RobotLog.d("imu init");
        imu.initialize(parameters);
        RobotLog.d("imu init finished");


        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        robot.sleep(200);
        imuIntegrator.initialize(parameters, new Position(DistanceUnit.METER, this.startingPosition.x, this.startingPosition.y, 0, 0), new Velocity());
//        while (!imu.isGyroCalibrated() && !robot.isStopRequested() && timer.seconds() < 5) {
//            robot.sleep(50);
//        }
//        if (imu.isGyroCalibrated()) {
//            robot.telemetry.addData("Gyro", "Done Calibrating");
//            RobotLog.d("Gyro done init");
//
//        } else {
//            robot.telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
//            RobotLog.d("Gyro failed init" + " " + imu.isGyroCalibrated() + " " + imu.isAccelerometerCalibrated() + " " + imu.isMagnetometerCalibrated());
//        }


//        imu.startAccelerationIntegration(new Position(DistanceUnit.METER, this.startingPosition.x, this.startingPosition.y, 0, 0), new Velocity(), 2);
//        BNO055IMU imu1;
//        imu1.startAccelerationIntegration();
//        RobotLog.d("IMU status: %s", imu.getSystemStatus().toShortString());
//        RobotLog.d("IMU calibration status: %s", imu.getCalibrationStatus().toString());
    }


    public void drive(double x, double y, double rotation, double boost) {


        x += extraX;
        y += extraY;
        rotation += extraR;

        x *= boost * 3;
        y *= boost * 3;
        rotation *= boost * 2;

        ChassisSpeeds speeds = calcDriveSpeeds(x, y);


// Now use this in our kinematics
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        double[] speedsArr = {
                wheelSpeeds.frontLeftMetersPerSecond - rotation,
                wheelSpeeds.frontRightMetersPerSecond - rotation,
                wheelSpeeds.rearLeftMetersPerSecond + rotation,
                wheelSpeeds.rearRightMetersPerSecond + rotation};
        speedsArr = modulateSpeeds(speedsArr);
        double frontLeft = speedsArr[0];
        double frontRight = speedsArr[1];
        double backLeft = speedsArr[2];
        double backRight = speedsArr[3];
        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
//        telemetry.addData("x",x);
//        telemetry.addData("y",y);
//        telemetry.addData("fl", fl.getCurrentPosition());
//        telemetry.addData("fr", fr.getCurrentPosition());
//        telemetry.addData("bl", bl.getCurrentPosition());
//        telemetry.addData("br", br.getCurrentPosition());
//        telemetry.addData("posx", imu.getPosition().x);
//        telemetry.addData("posy", imu.getPosition().y);

        telemetry.update();
    }

    public ChassisSpeeds calcDriveSpeeds(double x, double y) {
        if (isFieldOriented) {//not field now
            return ChassisSpeeds.fromFieldRelativeSpeeds(-x, y, 0, Rotation2d.fromDegrees(getAdjustedHeading())  );
        } else {
            return new ChassisSpeeds(-x, y, 0);
        }
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return (-orientation.getYaw(AngleUnit.DEGREES)) % 360 + 180;
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void setHeading(double heading) {
        correctedHeading = heading + 180;
    }

    public double getAdjustedHeading() {
        return getHeading() + correctedHeading;
    }

    public Point getPosition() {
        return new Point(imuIntegrator.getPosition().x + correctionX, imuIntegrator.getPosition().y + correctionY);
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.isFieldOriented = fieldOriented;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public double[] modulateSpeeds(double[] speeds) {
        double max = Math.max(Math.max(Math.abs(speeds[0]), Math.abs(speeds[1])), Math.max(Math.abs(speeds[2]), Math.abs(speeds[3])));
        if (Math.abs(max) > 1) {
            for (int i = 0; i < 4; i++) {
                speeds[i] /= max;
            }
        }
        return speeds;
    }

    public void setMoverServo(double pos) {
        moverServo.setPosition(pos);
    }

    public void setWentWentServo(double pos) {
        wentWentServo.setPosition(pos);
    }

    public void resetPos(Point pos) {
        correctionX = pos.x - imuIntegrator.getPosition().x;
        correctionY = pos.y - imuIntegrator.getPosition().y;
    }
    public void resetYaw(){
        imu.resetYaw();

    }



}