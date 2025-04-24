package org.firstinspires.ftc.teamcode.season.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.season.utils.Utils;
import org.opencv.core.Point;

@Config
public class SwerveDrive extends SubsystemBase {
    private final ElapsedTime angleTimer = new ElapsedTime();
    private final ElapsedTime wheelTimer = new ElapsedTime();
    private double timeSwitched = -1;
    private boolean isOptimised = true;
    //    SwerveDriveKinematics kinematics = new SwerveDriveKinematics();
//    SwerveDriveOdometry poseEstimator = new SwerveDriveOdometry();
    public SwerveModule bl, br, fl, fr;
    private DistanceSensor distanceSensor;
    private final BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    boolean isFieldOriented;
    public static boolean cos = false;
    public double averageError = 0;
    BNO055IMU imu;
    private double[] movementVector;
    private double[][] rotationVector;
    private double[][] wheelVectors = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    MultipleTelemetry telemetry;
    double boost = 0.35;
    final double rotationConpensation = 35.8;
    double wantedAngle = 0;
    boolean wasSpinning = false, isUsingAngleCorrection, wasTimeSelected;
    double timeTillCorrection = 0.25;
    public static double kp = 0.02;
    //    public double[] position = {0, 0};
    private final int ticksPerMeter = 1007;
    public static double minAngleError = 500;
    private double correctedHeading = 0;
    private final Point startingPos;
    private Point positionAdjustment = new Point(0, 0);
    private double powerModifier;
    private Point lastPos = new Point(0, 0);
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    MultipleTelemetry multipleTelemetry;

    public SwerveDrive(HardwareMap hardwareMap, MultipleTelemetry telemetry, LinearOpMode opMode, boolean isFieldOriented) {
        this.isFieldOriented = isFieldOriented;
        this.telemetry = telemetry;
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(this.telemetry, dashboardTelemetry);
        startingPos = new Point(0, 0);
        bl = new SwerveModule(hardwareMap.get(DcMotor.class, "bl_motor"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                hardwareMap.analogInput.get("bl_encoder"), 314.64);//314.64
        br = new SwerveModule(hardwareMap.get(DcMotor.class, "br_motor"),
                hardwareMap.get(CRServo.class, "br_servo"),
                hardwareMap.analogInput.get("br_encoder"), 54.93);//54.93
        fl = new SwerveModule(hardwareMap.get(DcMotor.class, "fl_motor"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                hardwareMap.analogInput.get("fl_encoder"), 0);//353.5
        fr = new SwerveModule(hardwareMap.get(DcMotor.class, "fr_motor"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                hardwareMap.analogInput.get("fr_encoder"), 346.9);//346.9
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        initImu(hardwareMap, telemetry, opMode);
    }

    public SwerveDrive(HardwareMap hardwareMap, MultipleTelemetry telemetry, LinearOpMode opMode, boolean isFieldOriented, Point startingPos) {
        this.isFieldOriented = isFieldOriented;
        this.telemetry = telemetry;
        this.startingPos = startingPos;

        bl = new SwerveModule(hardwareMap.get(DcMotor.class, "bl_motor"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                hardwareMap.analogInput.get("bl_encoder"), 314.64);
        br = new SwerveModule(hardwareMap.get(DcMotor.class, "br_motor"),
                hardwareMap.get(CRServo.class, "br_servo"),
                hardwareMap.analogInput.get("br_encoder"), 54.93);
        fl = new SwerveModule(hardwareMap.get(DcMotor.class, "fl_motor"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                hardwareMap.analogInput.get("fl_encoder"), 356.5);
        fr = new SwerveModule(hardwareMap.get(DcMotor.class, "fr_motor"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                hardwareMap.analogInput.get("fr_encoder"), 346.9);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        initImu(hardwareMap, telemetry, opMode);
    }

    private void initImu(HardwareMap hardwareMap, MultipleTelemetry telemetry, LinearOpMode opMode) {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.accelerationIntegrationAlgorithm = new SwerveImuIntegrator(imu, this, false, new Point(0, 0), 0);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (!imu.isGyroCalibrated() && !opMode.isStopRequested() && timer.seconds() < 5) {
            opMode.sleep(50);
        }
        if (imu.isGyroCalibrated()) {
            opMode.telemetry.addData("Gyro", "Done Calibrating");
            RobotLog.d("Gyro done init");

        } else {
            telemetry.addData("Gyro", "Gyro/IMU Calibration Failed");
            RobotLog.d("Gyro failed init" + " " + imu.isGyroCalibrated() + " " + imu.isAccelerometerCalibrated() + " " + imu.isMagnetometerCalibrated());
        }

        imu.initialize(parameters);
        imu.startAccelerationIntegration(null, null, 2);

    }

    //like update but here
    public void drive(double x, double y, double rotation, double _boost) {
        this.boost = _boost * 0.7 + 0.3;
        if (Math.hypot(x, y) < 0.05 && Math.abs(rotation) < 0.05) {
            idle();
            return;
        }
        if (Math.abs(rotation) < -1) {
            if (wasSpinning) {
                angleTimer.reset();
            }
            wasSpinning = false;
            if (angleTimer.seconds() > timeTillCorrection && !wasTimeSelected) {
                wasTimeSelected = true;
                isUsingAngleCorrection = true;
                wantedAngle = getHeading();
            }
            if (isUsingAngleCorrection) {
                // rotation += -Utils.signRoot(Utils.calcDeltaAngle(wantedAngle, getHeading()) * kp ) ;
                rotation += (Utils.calcDeltaAngle(wantedAngle, getHeading())) * kp;
            }

        } else {
            wasSpinning = true;
            wasTimeSelected = false;
            isUsingAngleCorrection = false;
        }
        if (isFieldOriented)
            movementVector = rotateVectors(boost * x, boost * y, getAdjustedHeading(rotation));
        else
            movementVector = rotateVectors(boost * x, boost * y, 0);

        // Convert field speed to robot coordinates
        rotationVector = getRotationVectors(rotation);
        wheelVectors = addVectors(rotationVector, movementVector);
        for (int i = 0; i < 4; i++) {
            wheelVectors[i] = cart2polar(wheelVectors[i]);
        }
        double angleError = Math.max(Math.max(Math.abs(fl.getAngleError()), Math.abs(fr.getAngleError())),
                Math.max(Math.abs(bl.getAngleError()), Math.abs(br.getAngleError())));
        powerModifier = 1;
//        Utils.map(angleError, minAngleError, maxAngleError, 1, 0);
        modulateSpeeds(wheelVectors);
        updateSwerveModules(wheelVectors);

        telemetry();
//        telemetry.addData("power modifier", powerModifier);
//        telemetry.addData("maxAngleError", angleError);
//        telemetry.addData("x velocity", imu.getVelocity().xVeloc);
//        telemetry.addData("y velocity", imu.getVelocity().yVeloc);
        telemetry.addData("min", 0);
        telemetry.addData("max", 360);
//        multipleTelemetry.addData("bl", bl.getCurrentHeading());
//        multipleTelemetry.addData("br", br.getCurrentHeading());
//        multipleTelemetry.addData("fr", fr.getCurrentHeading());
//        multipleTelemetry.addData("fl", fl.getCurrentHeading());
        telemetry.update();

    }

    public void idle() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.updateMotor();
        fr.updateMotor();
        br.updateMotor();
        bl.updateMotor();
        fl.setServoPower(0);
        fr.setServoPower(0);
        bl.setServoPower(0);
        br.setServoPower(0);

    }

    private void telemetry() {
//        telemetry.addData("fl error", fl.getAngleError());
//        telemetry.addData("fr error", fr.getAngleError());
//        telemetry.addData("bl error", bl.getAngleError());
//        telemetry.addData("br error", br.getAngleError());
//        telemetry.addData("position: ", imu.getPosition());
//        telemetry.addData("fl angle", fl.servo.getCurrentAngle());
//        telemetry.addData("fr angle", fr.servo.getCurrentAngle());
//        telemetry.addData("bl angle", bl.servo.getCurrentAngle());
//        telemetry.addData("br angle", br.servo.getCurrentAngle());
//       telemetry.addData("fl wanted", fl.getTargetHeading());
//       telemetry.addData("fr wanted", fr.getTargetHeading());
//       telemetry.addData("bl wanted", bl.getTargetHeading());
//       telemetry.addData("br wanted", br.getTargetHeading());
//       telemetry.addData("fl current", fl.getCurrentHeading());
//       telemetry.addData("fr current", fr.getCurrentHeading());
//       telemetry.addData("bl current", bl.getCurrentHeading());
//       telemetry.addData("br current", fr.getCurrentHeading());
//       telemetry.addData("rotationConpensation: ", rotationConpensation);
//       telemetry.addData("heading", getHeading());
//       telemetry.update();
    }


    //rotate a vector by an angle for field oriented
    private double[] rotateVectors(double x, double y, double heading) {
        heading = Math.toRadians(heading);
        double[] vectors = {0, 0};
        vectors[0] = Math.cos(heading) * x - Math.sin(heading) * y;
        vectors[1] = Math.sin(heading) * x + Math.cos(heading) * y;
        return vectors;

    }

    //returns rotation vectors adjusted to each wheel in a multidimensional array
    //FL is 0, FR is 1, BR is 2, BL is 3.
    private double[][] getRotationVectors(double rotation) {
        rotation *= boost * rotation * Math.signum(rotation);
        double[][] rotationVector = {
                {Math.sin(SwerveWheels.FL.ANGLE) * rotation, Math.cos(SwerveWheels.FL.ANGLE) * rotation},
                {Math.sin(SwerveWheels.FR.ANGLE) * rotation, Math.cos(SwerveWheels.FR.ANGLE) * rotation},
                {Math.sin(SwerveWheels.BR.ANGLE) * rotation, Math.cos(SwerveWheels.BR.ANGLE) * rotation},
                {Math.sin(SwerveWheels.BL.ANGLE) * rotation, Math.cos(SwerveWheels.BL.ANGLE) * rotation}};
        return rotationVector;
    }

    public void setFieldOriented(boolean orientation) {
        isFieldOriented = orientation;
    }

    //adds the rotationVector and the movement vector making it what each wheel needs to do
    private double[][] addVectors(double[][] rotationVector, double[] movementVector) {
        double[][] vector = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
        for (int i = 0; i < 4; i++) {
            vector[i][0] = rotationVector[i][0] + movementVector[0];
            vector[i][1] = rotationVector[i][1] + movementVector[1];
        }
        return vector;
    }

    //converts vector from x and y to length and angle(in degrees)
    private double[] cart2polar(double[] v) {
        double[] vector = {0, 0};
        //check if correct
        vector[0] = Math.hypot(v[0], v[1]); // Math.sqrt(x * x + y * y);
        vector[1] = Math.toDegrees(Math.atan2(v[0], v[1]));
        return vector;
    }

    //get chassis heading in degrees
    public double getHeading() {
        Orientation orientation = imu.getAngularOrientation();
        return (-orientation.firstAngle) % 360 + 180;
    }

    public void resetHeading() {
        correctedHeading = getHeading() + 180;
    }

    public void setHeading(double heading) {
        correctedHeading = heading + 180;
    }

    public Point getPosition() {
        Position imuPos = imu.getPosition();
        return new Point(imuPos.x + startingPos.x, imuPos.y + startingPos.y);
    }

    public Point getAdjustedPosition() {
        Position imuPos = imu.getPosition();
        return new Point(imuPos.x + startingPos.x + positionAdjustment.x,
                imuPos.y + startingPos.y + positionAdjustment.y);
    }

    public void setPosition(Point position) {
        Point currentPos = getPosition();
        positionAdjustment = new Point(position.x - currentPos.x, position.y - currentPos.y);
    }

    //for it to not go to the side when spinning and driving
    public double getAdjustedHeading(double rotation) {
        return getHeading() + rotationConpensation * rotation - correctedHeading;
    }

    private void updateSwerveModules(double[][] wheelVectors) {
        double averageVectorAngle = (fl.getDeltaAngle(wheelVectors[0][1]) + fr.getDeltaAngle(wheelVectors[1][1]) +
                br.getDeltaAngle(wheelVectors[2][1]) + bl.getDeltaAngle(wheelVectors[3][1])) / 4;
        //TODO: disabled the angle optimization fix
        if (averageVectorAngle > 90 && (wheelTimer.seconds() > (timeSwitched + 0.15) || isOptimised)) {
            if (!isOptimised) {
                isOptimised = true;
                timeSwitched = wheelTimer.seconds();
            }
            fl.setHeading(wheelVectors[0][1], true);
            fr.setHeading(wheelVectors[1][1], true);
            br.setHeading(wheelVectors[2][1], true);
            bl.setHeading(wheelVectors[3][1], true);
        } else if ((averageVectorAngle < 90) && (wheelTimer.seconds() > (timeSwitched + 0.15) || !isOptimised)) {
            if (isOptimised) {
                isOptimised = false;
                timeSwitched = wheelTimer.seconds();
            }
            fl.setHeading(wheelVectors[0][1], false);
            fr.setHeading(wheelVectors[1][1], false);
            br.setHeading(wheelVectors[2][1], false);
            bl.setHeading(wheelVectors[3][1], false);
        } else if (isOptimised) {
            fl.setHeading(wheelVectors[0][1], true);
            fr.setHeading(wheelVectors[1][1], true);
            br.setHeading(wheelVectors[2][1], true);
            bl.setHeading(wheelVectors[3][1], true);
        } else {
            fl.setHeading(wheelVectors[0][1], false);
            fr.setHeading(wheelVectors[1][1], false);
            br.setHeading(wheelVectors[2][1], false);
            bl.setHeading(wheelVectors[3][1], false);
        }


        averageError = (Math.abs(fl.getAngleError()) + Math.abs(fr.getAngleError()) +
                Math.abs(br.getAngleError()) + Math.abs(bl.getAngleError())) / 4;

        if (cos) {
            powerModifier = Math.abs(Math.cos(Math.toRadians(averageError)));
        } else {
            if (averageError > minAngleError) {
                powerModifier = 0;
            } else {
                powerModifier = 1;
            }
        }
        fl.setPower(wheelVectors[0][0] * powerModifier);
        fr.setPower(wheelVectors[1][0] * powerModifier);
        br.setPower(wheelVectors[2][0] * powerModifier);
        bl.setPower(wheelVectors[3][0] * powerModifier);
        fl.update();
        fr.update();
        br.update();
        bl.update();
    }

    private double[][] modulateSpeeds(double[][] vectors) {
        double a = Math.max(Math.max(Math.abs(vectors[0][0]), Math.abs(vectors[1][0])),
                Math.max(Math.abs(vectors[2][0]), Math.abs(vectors[3][0])));
        if (a > 1) {
            for (int i = 0; i < 4; i++) {
                vectors[i][0] /= a;
            }
        }
        return vectors;
    }

    public double[] calcDeltaPos() {
        double heading = getHeading();
        double[] position = {0, 0};
        double posDiff = fl.getPositionDifference();
        double angle = Math.toRadians(fl.getCurrentHeading() + heading);
        position[0] += Math.sin(angle) * posDiff / (4 * ticksPerMeter);
        position[1] += Math.cos(angle) * posDiff / (4 * ticksPerMeter);

        posDiff = fr.getPositionDifference();
        angle = Math.toRadians(fr.getCurrentHeading() + heading);
        position[0] += Math.sin(angle) * posDiff / (4 * ticksPerMeter);
        position[1] += Math.cos(angle) * posDiff / (4 * ticksPerMeter);

        posDiff = bl.getPositionDifference();
        angle = Math.toRadians(bl.getCurrentHeading() + heading);
        position[0] += Math.sin(angle) * posDiff / (4 * ticksPerMeter);
        position[1] += Math.cos(angle) * posDiff / (4 * ticksPerMeter);

        posDiff = br.getPositionDifference();
        angle = Math.toRadians(br.getCurrentHeading() + heading);
        position[0] += Math.sin(angle) * posDiff / (4 * ticksPerMeter);
        position[1] += Math.cos(angle) * posDiff / (4 * ticksPerMeter);

        return position;
    }

    public double[] getTargetAngles() {
        return new double[]{wheelVectors[0][1],
                wheelVectors[1][1],
                wheelVectors[2][1],
                wheelVectors[3][1]};
    }

    public double[] getCurrentAngles() {
        return new double[]{fl.getCurrentHeading(),
                fr.getCurrentHeading(),
                br.getCurrentHeading(),
                bl.getCurrentHeading()};
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

//    public void update() {
//        // TODO: calc wheel velocity
//
//        localizer.update();
//    }
//
//    // ###################################### Roadrunner implementation #####################################
//    @NonNull
//    @Override
//    public Localizer getLocalizer() {
//        return localizer;
//    }
//
//    @Override
//    public void setLocalizer(@NonNull Localizer localizer) {
//        assert false;
//    }
//
//    @Override
//    protected double getRawExternalHeading() {
//        return imu.getAngularOrientation().firstAngle;
//    }
//
//    @Override
//    public void setDrivePower(@NonNull Pose2d pose2d) {
//        double angle = -Math.toDegrees(pose2d.getHeading());
//        drive(pose2d.getX(), pose2d.getY(), angle ,1);
//    }
//
//    @Override
//    public void setDriveSignal(@NonNull DriveSignal driveSignal) {
//        // TODO: understand and implement
//    }
//
//    public List<Double> getWheelPositions() {
//        return new ArrayList<>(Arrays.asList(fl.getPosition(), bl.getPosition(), fr.getPosition(), br.getPosition()));
//    }
//
//    public List<Double> getModuleOrientations() {
//        return new ArrayList<>(Arrays.asList(
//                Math.toRadians(-fl.getCurrentHeading()),
//                Math.toRadians(-bl.getCurrentHeading()),
//                Math.toRadians(-fr.getCurrentHeading()),
//                Math.toRadians(-br.getCurrentHeading())));
//    }
//
//    public List<Double> getWheelVelocities() {
//        // TODO: understand and implement
//
//        return null;
//    }
//
//    public double getWheelBase() {
//        return 12.8346456693;
//    }
//
//    public double getTrackWidth() {
//        return 11.6535433071;
//    }
}
