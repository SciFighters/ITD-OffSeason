package org.firstinspires.ftc.teamcode.season.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.*;


// todo:
// add gyro
// max servo 90
// servo noy to return zero
//
@TeleOp(name = "ilanTest2", group = "Linear Opmode")
public class Swerve_Drive extends LinearOpMode {

    Chassis robot;
    boolean fieldOrientedDriving = false;
    double speedMultiplier = 0.25;
    double adjustGamepadTime = 0;
    double lastServoCalibrationTime = 0;
    double rotationCalibration = 0;

    static final int FIELD_X_SIZE = 1500;
    static final int FIELD_Y_SIZE = 1000;
    static final int motorTicksPerRevolution = 28;
    static final int motorMaxRPM = 6000;
    static final int motorMaxTickPerSecond = motorTicksPerRevolution * motorMaxRPM / 60;
    static final double wheelCircumference = 19.8;  //cm
    static final double motorGearRatio = 7;  // gear ratio from motor to wheel

    @Override
    public void runOpMode() {
        // Wait for the game to start
        //telemetry.setAutoClear(false);

        waitForStart();
        initSwerveDrive();
        while (opModeIsActive()) {
            drive();
            telemetry.update();
        }
    }

    public void initSwerveDrive() {
        robot = new Chassis();
        fieldOrientedDriving = false;
    }

    public void drive() {
        if (gamepad1.y) {
            robot.minSpeedTest();
            return;
        }

        robot.servoCalibration();
        if (gamepad1.x) fieldOrientedDriving = true;
        if (gamepad1.b) fieldOrientedDriving = false;

        adjustSpeed();

        double lx = gamepad1.left_stick_x * speedMultiplier;
        double ly = gamepad1.left_stick_y * speedMultiplier;

        Vector2d direction = new Vector2d(lx, -ly);
        print("direction=" + direction);

        double directionSpeed = direction.magnitude();  //??? what units?
        double rotation = gamepad1.right_stick_x * speedMultiplier;

        //if (directionSpeed>0) rotation += rotationCalibration;

        double chassisAngle = robot.getGyroOrientation();
        print2("yaw", chassisAngle);
        print2("calc yaw", robot.getCalcOrientation());
        print("location=" + robot.getCenterLocation());


        for (int i = 0; i < 4; i++) {
            Vector2d dirVec = new Vector2d();
            Vector2d rotVec = new Vector2d();

            double alpha = directionSpeed > 0 ? direction.angle() : 0;
            if (fieldOrientedDriving) alpha -= chassisAngle;
            dirVec.setAngle(alpha, directionSpeed);

            double rotVecAngle = robot.wheels[i].chassisPos.angle() + 90;
            if (rotation < 0) rotVecAngle -= 90 * 2;
            rotVec.setAngle(rotVecAngle, abs(rotation) * 0.8);

            Vector2d combVec = rotVec.add(dirVec);

            print("wheel " + i);
            print("   location " + robot.wheels[i].location);
            print("   steering " + robot.wheels[i].steering);
            print2("   odometer", robot.wheels[i].odometer);

            robot.wheels[i].setSteering(combVec);
        }
        robot.move(0);
    }

    class AxonServo {
        private Servo servo;
        private AnalogInput absEncoder;  // To read the position feedback
        private static final double errorThreshold1 = 1;  // Threshold degrees for stopping servo
        private double targetAngle;
        private double kp, ki, kd;
        public double zeroAngle;   // calibration of wheel steering
        private int polarity1;   // 1 or -1 , two avoid steering above 90 degress
        private double minSpeed = 0.02;

        public void init(String servoName, String feedbackName, int calibration) {
            servo = hardwareMap.get(Servo.class, servoName);
            absEncoder = hardwareMap.get(AnalogInput.class, feedbackName);
            targetAngle = 0;
            polarity1 = 1;
            zeroAngle = calibration;
            setPidConstants(0.00334, 0, 0);
            Runnable controller = new Runnable() {
                @Override
                public void run() {
                    moveToTargetAngle();
                }
            };
            Thread controllerThread = new Thread(controller);
            controllerThread.start();
        }

        ;

        public void setPidConstants(double p, double i, double d) {
            kp = p;
            ki = i;
            kd = d;
        }

        public void flipPolarity() {
            polarity1 *= -1;
        }

        public int polarity() {
            return polarity1;
        }

        void setTargetAngle(double a) {
            targetAngle = a;
        }

        void setSpeed(double speed) {    // speed range -1:1
            if (speed > 0) {
                speed += minSpeed;
            } else if (speed < 0) {
                speed -= minSpeed;
            }
            double v = 0.5 + speed / 2;
            servo.setPosition(v); // this is speed in Continues Rotation mode, range 0:1
        }

        public double testServo() {
            double a = 0;
            int sides = 7;
            double score = 0;
            for (int j = 0; j < sides; j++) {
                if (!opModeIsActive()) break;
                setTargetAngle(a);
                sleep(2000);
                a = angle180(a + 360 / sides);
            }
            return score;
        }

        public double stopServo() {
            setSpeed(0);
            double a1 = 999;

            for (int samePos = 0; samePos < 20; ) {
                double a0 = getCurrentAngle();
                if (a0 == a1) samePos++;
                a1 = a0;
            }
            return a1;
        }

        // how far a target is from current servo position
        public double targetDistance(double targetAngle) {
            double currentAngle = getCurrentAngle();
            if (polarity() == -1) currentAngle = angle180(currentAngle + 180);
            double target1 = targetAngle + zeroAngle;
            double error = angle180(target1 - currentAngle);
            return error;
        }

        public void moveToTargetAngle() {
            print("moveToTargetAngle start");
            // Start a control loop to adjust the servo based on feedback
            double prevError = 0;
            double startError = 999;
            double ierror = 0;
            int nPositive = 0;
            int nNegative = 0;

            while (opModeIsActive()) {
                double error = targetDistance(targetAngle);
                if (startError == 999) startError = error;
                double derror = error - prevError;
                ierror += error;
                // Calculate speed (-1.0 to 1.0) based on the error
                double speed = -pid(error, ierror, derror, kp, ki, kd);
                if (!gamepad1.y)
                    setSpeed(speed);
                prevError = error;
            }
            print("moveToTargetAngle exit");
        }

        // Reads the current position of the servo from the analog feedback sensor
        private double getCurrentAngle() {
            // Assuming the analog feedback value is in the range [0, 1], convert it to an angle
            return absEncoder.getVoltage() / 3.274 * 360 - 180;
        }
    }


    void adjustSpeed() {
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            long timeNow = System.currentTimeMillis();
            if (timeNow - adjustGamepadTime > 500) {
                adjustGamepadTime = timeNow;
                if (gamepad1.left_bumper) speedMultiplier /= 1.2;
                if (gamepad1.right_bumper) speedMultiplier *= 1.2;
                print2("speed multipler", speedMultiplier);
            }
        }

        if (gamepad1.dpad_left || gamepad1.dpad_right) {
            long timeNow = System.currentTimeMillis();
            if (timeNow - adjustGamepadTime > 500) {
                adjustGamepadTime = timeNow;
                if (gamepad1.dpad_left) rotationCalibration -= 0.001;
                if (gamepad1.dpad_right) rotationCalibration += 0.001;
                print2("rotationCalibration", rotationCalibration);
            }
        }
    }

    // Simple method to calculate speed based on error (P controller)
    private double pid(double e, double ie, double de, double kp, double ki, double kd) {
        // Proportional control (tune the constant as needed)
        return kp * e + ki * ie + kd * de;
    }

    void sleep(int ms) {
        //telemetry.sleep(ms);
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    // change any angle aggrees to range -180:180
    double angle180(double a) {
        a = a % 360;
        if (a > 180) a -= 360;
        else if (a < -180) a += 360;
        return a;
    }

    double random2(double low, double high) {
        if (high < low) {
            double t = high;
            high = low;
            low = t;
        }
        return low + (Math.random() * (high - low));
    }

    String d2(double d) {
        return String.format("%.2f", d);
    }

    void print(String s) {
        telemetry.addLine(s);
    }

    void print2(String s, double d) {
        print(s + " " + d2(d));
    }


    void optimizePID(AxonServo servo) {
        double kp = 0;
        double ki = 0;
        double kd = 0;
        double best_kp = 0.003;
        double best_ki = 0;
        double best_kd = 0;
        double bestScore = 0;

        double a = 0;
        while (opModeIsActive()) {
            kp = best_kp * random2(0.7, 1.3) + random2(-0.0001, 0.0001);
            ki = best_ki; //* random2(0.8, 1.2) + random2(0,0.001);
            kd = best_kd * random2(0.7, 1.3) + random2(-0.0001, 0.0001);
            servo.setPidConstants(kp, ki, kd);
            double score = servo.testServo();

            if (score > bestScore) {
                bestScore = score;
                best_kp = kp;
                best_ki = ki;
                best_kd = kd;
                telemetry.addData("score", score);
                telemetry.addData("kp", kp);
                telemetry.addData("ki", ki);
                telemetry.addData("kd", kd);
                telemetry.update();
                sleep(5000);
                servo.testServo();
            }
        }
    }

    class Chassis {
        SwerveModule[] wheels = new SwerveModule[4];

        // Declare the IMU object
        BNO055IMU imu;
        Orientation imuAngles;

        Chassis() {
            for (int i = 0; i < 4; i++) wheels[i] = new SwerveModule();
            build();
            imuSetup();
        }

        void imuSetup() {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(parameters);
        }

        double getGyroOrientation() {
            imuAngles = imu.getAngularOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZYX,
                    AngleUnit.DEGREES);

            double yawAngle = -imuAngles.firstAngle;
            return yawAngle;
        }

        void build() {
            double cx = 38;    // chassis width, cm distance betwwen wheels center
            double cy = 34.5;  // chassis length cm

            wheels[0].init("fl");
            wheels[1].init("fr");
            wheels[2].init("bl");
            wheels[3].init("br");

            wheels[0].chassisPos.set(-cx / 2, cy / 2);  // front left
            wheels[1].chassisPos.set(cx / 2, cy / 2);   // front right
            wheels[2].chassisPos.set(-cx / 2, -cy / 2); // back left
            wheels[3].chassisPos.set(cx / 2, -cy / 2);  // back right
            calcWheelsFieldPos();

            wheels[0].servo.zeroAngle = 171; //-20;
            wheels[1].servo.zeroAngle = 164; //-100;
            wheels[2].servo.zeroAngle = 132; //-35;
            wheels[3].servo.zeroAngle = 232; //-103;
        }

        void calcWheelsFieldPos() {
            Vector2d center = new Vector2d(0, 0);
            double alpha = 0;
            if (wheels[0].location.x != 0) {
                center = getCenterLocation();
                alpha = getGyroOrientation();
            }
            for (int i = 0; i < 4; i++) {
                Vector2d pos = new Vector2d(wheels[i].chassisPos);
                pos.rotate(alpha);
                wheels[i].location = center.add(pos);
                wheels[i].setId(i + 1);
                wheels[i].setChassis(this);
            }
        }

        double getCalcOrientation() {
            Vector2d v02 = wheels[0].location.subtract(wheels[2].location);  // angle from 2(BL) to 0(FL)
            Vector2d v01 = wheels[0].location.subtract(wheels[1].location);  // angle from 1(FR) to 0(FL) wheels which should be Perpendicular to v02
            print2("v02", v02.angle());
            print2("v01", v01.angle());
            v01.rotate(90);
            double a = ((v01.add(v02)).divide(2)).angle(); // the average of two vectors (angles)
            return a;
        }

        //
        // the center location of the chassis is the average loations
        // of the four wheels
        //
        Vector2d getCenterLocation() {
            Vector2d sum = new Vector2d();
            for (SwerveModule wheel : wheels) sum.add2(wheel.location);
            return sum.divide(4);
        }

        boolean move(int deltaTime) {
            boolean moved = false;
            double chassisAngle = getGyroOrientation();
            for (SwerveModule wheel : wheels) {
                if (wheel.move(deltaTime, chassisAngle)) moved = true;
            }
            if (moved) calcWheelsFieldPos(); // fix location drift
            return moved;
        }

        void minSpeedTest() {
            double rightTrugger = gamepad1.right_trigger / 10;
            wheels[0].servo.setSpeed(rightTrugger);
            print2("test min speed", rightTrugger);
        }

        void servoCalibration() {
            int s = -1;
            if (gamepad1.dpad_up) s = 0;
            if (gamepad1.dpad_down) s = 1;
            if (gamepad1.dpad_left) s = 2;
            if (gamepad1.dpad_right) s = 3;

            if (s != -1) {
                if (gamepad1.y) {
                    // identify the servo
                    wheels[s].motor.setPower(0.5);
                    sleep(1000);
                    wheels[s].motor.setPower(0);
                } else {
                    long timeNow = System.currentTimeMillis();
                    if (timeNow - lastServoCalibrationTime > 50) {
                        lastServoCalibrationTime = timeNow;
                        if (gamepad1.x) wheels[s].servo.zeroAngle--;
                        if (gamepad1.b) wheels[s].servo.zeroAngle++;
                        telemetry.addData("calibration servo", s);
                        telemetry.addData("calibration value", wheels[s].servo.zeroAngle);
                    }
                }
            }
        }
    }

    class SwerveModule {
        AxonServo servo;
        DcMotorEx motor;

        Vector2d chassisPos = new Vector2d(); // static position on the chassis, relative to chassis mid point
        Vector2d steering = new Vector2d(); // (0,1) is straight
        int rotationSign; // 1 or -1
        Vector2d location = new Vector2d(); // absolute field location
        Chassis chassis; // link to the chassis
        int motorTicks;
        double odometer;   // total distance moved

        int id; // 1..number of modules

        void init(String name) {
            motor = hardwareMap.get(DcMotorEx.class, name + "_motor");
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            servo = new AxonServo();
            servo.init(name + "_servo", name + "_encoder", 168);
            steering.set(0, 1);
            chassisPos.set(0, 0);
            rotationSign = 1;
            motorTicks = motor.getCurrentPosition();
            odometer = 0;
        }

        void setChassis(Chassis c) {
            chassis = c;
        }

        void setId(int n) {
            id = n;
        }


        void setSteering(Vector2d v) {
            steering = v;
            double speed = v.magnitude();
            if (speed > 0) {
                double target = v.angle();
                double error = servo.targetDistance(target);
                if (abs(error) > 90) servo.flipPolarity();
                servo.setTargetAngle(target);
            }
        }

        double speed() {
            return steering.magnitude();
        } // current moving speed m/s

        double ticks2cm(double ticks) {
            return ticks / (motorTicksPerRevolution * motorGearRatio) * wheelCircumference;
        }


        boolean move(int deltaTime, double chassisAngle) {
            Vector2d r = steering.unit();  // steering angle, length 1
            r.rotate(chassisAngle);
            int newTicks = motor.getCurrentPosition();
            double deltaTicks = newTicks - motorTicks;
            double distanceMoved = ticks2cm(deltaTicks) * servo.polarity();
            odometer += distanceMoved;
            motorTicks = newTicks;
            Vector2d step = r.multiply(distanceMoved);
            print("step " + step);
            location = location.add(step);
            motor.setVelocity(speed() * motorMaxTickPerSecond * servo.polarity());
            return speed() != 0;
        }
    }

    class Vector2d {
        double x;
        double y;

        Vector2d() {
            x = 0;
            y = 0;
        }

        Vector2d(double x_, double y_) {
            x = x_;
            y = y_;
        }

        // Copy constructor
        Vector2d(Vector2d v) {
            x = v.x;
            y = v.y;
        }

        void set(double x_, double y_) {
            x = x_;
            y = y_;
        }

        // Vector addition
        Vector2d add(Vector2d other) {
            return new Vector2d(x + other.x, y + other.y);
        }

        void add2(Vector2d other) {
            x += other.x;
            y += other.y;
        }

        // Vector subtraction
        Vector2d subtract(Vector2d other) {
            return new Vector2d(x - other.x, y - other.y);
        }

        // Scalar multiplication
        Vector2d multiply(double scalar) {
            return new Vector2d(x * scalar, y * scalar);
        }

        // Scalar division
        Vector2d divide(double scalar) {
            if (scalar != 0) {
                return new Vector2d(x / scalar, y / scalar);
            } else {
                throw new IllegalArgumentException("Division by zero");
            }
        }

        // same angle, but len 1

        Vector2d unit() {
            double mag = magnitude();
            return (mag > 0) ? divide(mag) : new Vector2d(0, 1);
        }

        // Dot product
        double dot(Vector2d other) {
            return x * other.x + y * other.y;
        }

        // Magnitude (length)
        double magnitude() {
            return sqrt(x * x + y * y);
        }

        double angle() {
            //print("x1="+x1);
            //print("y1="+y1);
            return atan2(x, y) * 180 / PI;
        }

        void setAngle(double a, double len) {
            //print("setAngle a="+a);
            //print("setAngle len="+len);

            double theta = a * PI / 180;
            x = len * sin(theta);
            y = len * cos(theta);
            //print("setAngle x="+x);
            //print("setAngle y="+y);
        }

        void rotate(double a) {
            //print("rotate this="+ this);
            //print("rotate a="+a);
            //print("angle()="+angle());
            double b = angle() + a;
            setAngle(b, magnitude());
        }

        // Normalize (make unit vector)
        Vector2d normalize() {
            double mag = magnitude();
            if (mag != 0) {
                return this.divide(mag);
            } else {
                throw new IllegalArgumentException("Cannot normalize zero vector");
            }
        }

        @Override
        public String toString() {
            return "[" + d2(x) + ", " + d2(y) + "] " + d2(angle());
        }
    }
}
