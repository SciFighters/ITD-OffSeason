package org.firstinspires.ftc.teamcode.season.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.season.utils.Utils;
import org.opencv.core.Point;

import java.util.function.Supplier;

public class SwerveCommands {
    public static class NoOpCommand extends CommandBase {
        public NoOpCommand(SwerveDrive swerveDrive) {
            addRequirements(swerveDrive);
        }

        @Override
        public boolean isFinished() {
            return false; // Runs indefinitely
        }
    }

    public static class PowerCmd extends CommandBase {
        Supplier<Double> x;
        Supplier<Double> y;
        Supplier<Double> r;
        Supplier<Double> boost;
        final SwerveDrive swerveDrive;
        final Telemetry telemetry;
        final boolean isFieldOriented;
        final double rotationModifier = 0.75;

        public PowerCmd(Telemetry telemetry, SwerveDrive swerveDrive, Supplier<Double> x,
                        Supplier<Double> y, Supplier<Double> r, Supplier<Double> boost, boolean isFieldOriented) {
            this.isFieldOriented = isFieldOriented;
            this.x = x;
            this.y = y;
            this.r = () -> r.get() * rotationModifier;
            this.boost = boost;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;


            addRequirements(swerveDrive);
        }

        @Override
        public void initialize() {
            swerveDrive.setFieldOriented(isFieldOriented);
        }

        @Override
        public void execute() {
//            telemetry.addData("X", x.get());
//            telemetry.addData("Y", y.get());
//            telemetry.addData("TURN", r.get());
            swerveDrive.drive(x.get(), y.get(), r.get(), boost.get() / 2);
        }

        @Override
        public void end(boolean interrupted) {
            swerveDrive.drive(0, 0, 0, 0);
        }
    }

    public static class GotoCmd extends CommandBase {
        double x, y, wantedAngle, wantedDistance;
        double error, lastError, lastTime;
        double proportional, Integral, derivative;
        double kp = 0.025, ki = 0.0005, kd = -0.006;
        Point currentPos;
        double boost;
        double sensitivity;
        double rotation = 0;
        final double minPower = 0.04;
        SwerveDrive swerveDrive;
        Telemetry telemetry;
        boolean noRotation = false;

        public GotoCmd(Telemetry telemetry, SwerveDrive swerveDrive, double x, double y,
                       double wantedAngle, double sensitivity, double wantedDistance, double boost) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.boost = boost;
            this.sensitivity = sensitivity;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;
            this.wantedDistance = wantedDistance;

            addRequirements(swerveDrive);
        }

        public GotoCmd(Telemetry telemetry, SwerveDrive swerveDrive, double x, double y,
                       double wantedAngle, double sensitivity, double boost) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.boost = boost;
            this.sensitivity = sensitivity;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;
            this.wantedDistance = -1;

            addRequirements(swerveDrive);
        }

        public GotoCmd(Telemetry telemetry, SwerveDrive swerveDrive, double x, double y,
                       double wantedAngle, double sensitivity, double boost, boolean noRotation) {
            this.x = x;
            this.y = y;
            this.wantedAngle = wantedAngle;
            this.boost = boost;
            this.sensitivity = sensitivity;
            this.swerveDrive = swerveDrive;
            this.telemetry = telemetry;
            this.wantedDistance = -1;
            this.noRotation = noRotation;

            addRequirements(swerveDrive);
        }

        @Override
        public void execute() {
            currentPos = swerveDrive.getAdjustedPosition();
            double[] localVector = {x - currentPos.x, y - currentPos.y};
            double MovementAngle = Math.atan2(localVector[0], localVector[1]);
            double length = Range.clip(Math.hypot(localVector[0], localVector[1]), -1, 1);
            length += Math.signum(length) * minPower;
            localVector[0] = Math.sin(MovementAngle) * length;
            localVector[1] = Math.cos(MovementAngle) * length;
            double currentTime = (double) System.currentTimeMillis() / 1000;
            double deltaTime = currentTime - lastTime;
            if (lastTime != 0) {
                error = Utils.calcDeltaAngle(wantedAngle + 180, swerveDrive.getAdjustedHeading(0));
                proportional = Range.clip(error, -100, 100) * kp;
                Integral += Range.clip(error, -30, 30) * deltaTime;
                if (Math.signum(Integral) != Math.signum(error)) {
                    Integral = 0;
                }
                derivative = (lastError - error) / deltaTime;
                rotation = proportional + Integral * ki + derivative * kd + Math.signum(proportional + Integral * ki + derivative * kd) * 0.04;
                rotation = rotation * 0.65 / (boost * 0.7 + 0.3);
            }
            lastError = error;
            lastTime = currentTime;
            if (noRotation) {
                swerveDrive.drive(localVector[0], localVector[1], 0, boost);

            } else {
                swerveDrive.drive(localVector[0], localVector[1], rotation / 2.5, boost);
            }

        }

        @Override
        public boolean isFinished() {
            return (((Math.hypot(currentPos.x - x, currentPos.y - y) < sensitivity) || (swerveDrive.getDistance() <= wantedDistance))
                    && ((Math.abs(wantedAngle + 180 - swerveDrive.getAdjustedHeading(0)) < 3) || noRotation));
        }

        @Override
        public void end(boolean interrupted) {
            swerveDrive.idle();
        }
    }

    public static class SplineGotoCmd extends CommandBase {
        double x, y, wantedAngle;
        double error, lastError, lastTime;
        double proportional, Integral, derivative;
        double kp = 0.025, ki = 0.0005, kd = -0.006;
        Point currentPos;
        double boost;
        double sensitivity;
        double rotation = 0;
        final double minPower = 0.04;
        SwerveDrive swerveDrive;
        Point[] points;

        public SplineGotoCmd(SwerveDrive swerveDrive, Point p0, Point p1, Point p2, double boost, double sensitivity) {
            this.swerveDrive = swerveDrive;
            this.boost = boost;
            this.sensitivity = sensitivity;
            points[0] = p0;
            points[1] = p1;
            points[2] = p2;
        }

        @Override
        public void execute() {
            double t = Range.clip(findClosestT(swerveDrive.getPosition(), points[0], points[1], points[2]) + 0.15, 0, 1);
            Point wantedPosition = new Point(Math.pow(1 - t, 2) * points[0].x + 2 * t * (1 - t) *
                    points[1].x + Math.pow(t, 2) * points[2].x,
                    Math.pow(1 - t, 2) * points[0].y + 2 * t * (1 - t) *
                            points[1].y + Math.pow(t, 2) * points[2].y);
            currentPos = swerveDrive.getAdjustedPosition();
            double[] localVector = {x - currentPos.x, y - currentPos.y};
            double MovementAngle = Math.atan2(localVector[0], localVector[1]);
            double length = Range.clip(Math.hypot(localVector[0], localVector[1]), -1, 1);
            length += Math.signum(length) * minPower;
            localVector[0] = Math.sin(MovementAngle) * length;
            localVector[1] = Math.cos(MovementAngle) * length;
            double currentTime = (double) System.currentTimeMillis() / 1000;
            double deltaTime = currentTime - lastTime;
            if (lastTime != 0) {
                error = Utils.calcDeltaAngle(wantedAngle + 180, swerveDrive.getAdjustedHeading(0));
                proportional = Range.clip(error, -100, 100) * kp;
                Integral += Range.clip(error, -30, 30) * deltaTime;
                if (Math.signum(Integral) != Math.signum(error)) {
                    Integral = 0;
                }
                derivative = (lastError - error) / deltaTime;
                rotation = proportional + Integral * ki + derivative * kd + Math.signum(proportional + Integral * ki + derivative * kd) * 0.04;
                rotation = rotation * 0.65 / (boost * 0.7 + 0.3);
            }
            lastError = error;
            lastTime = currentTime;
            swerveDrive.drive(localVector[0], localVector[1], rotation, boost);

        }

        @Override
        public boolean isFinished() {
            return ((Math.hypot(currentPos.x - x, currentPos.y - y) < sensitivity) &&
                    (Math.abs(wantedAngle + 180 - swerveDrive.getAdjustedHeading(0)) < 3));
        }

        public double findClosestT(Point current, Point P0, Point P1, Point P2) {
            double lower = 0.0, upper = 1.0, tolerance = 0.005; // Precision of the search
            while (upper - lower > tolerance) {
                double mid = (lower + upper) / 2.0;
                double left = bezierDistance(mid - tolerance, current.x, current.y, P0, P1, P2);
                double right = bezierDistance(mid + tolerance, current.y, current.y, P0, P1, P2);
                if (left < right) {
                    upper = mid;
                } else {
                    lower = mid;
                }
            }
            return (lower + upper) / 2.0;
        }

        private double bezierDistance(double t, double a, double b, Point P0, Point P1, Point P2) {
            // Calculate Bézier curve point at t
            double x_t = Math.pow(1 - t, 2) * P0.x + 2 * t * (1 - t) * P1.x + Math.pow(t, 2) * P2.x;
            double y_t = Math.pow(1 - t, 2) * P0.y + 2 * t * (1 - t) * P1.y + Math.pow(t, 2) * P2.y;

            // Calculate Euclidean distance to the point (a, b)
            return Math.sqrt(Math.pow(x_t - a, 2) + Math.pow(y_t - b, 2));
        }

    }

    public static class SetPositionCmd extends CommandBase {
        Point pos;
        SwerveDrive swerveDrive;

        public SetPositionCmd(SwerveDrive swerveDrive, Point pos) {
            this.pos = pos;
            this.swerveDrive = swerveDrive;
        }

        @Override
        public void initialize() {
            swerveDrive.setPosition(pos);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    @Config
    public static class SetRotationCmd extends CommandBase {
        double wantedHeading;
        double error = 0, lastError = 0, proportional, lastTime = 0, Integral, derivative;
        public static double kp = 0.025, ki = 0.0005, kd = -0.006;
        SwerveDrive swerveDrive;

        public SetRotationCmd(SwerveDrive swerveDrive, double wantedHeading) {
            this.swerveDrive = swerveDrive;
            this.wantedHeading = wantedHeading;
            addRequirements(swerveDrive);
        }

        @Override
        public void execute() {
            double currentTime = (double) System.currentTimeMillis() / 1000;
            double deltaTime = currentTime - lastTime;
            if (lastTime != 0) {
                error = Utils.calcDeltaAngle(wantedHeading + 180, swerveDrive.getAdjustedHeading(0));
                proportional = Range.clip(error, -100, 100) * kp;
                Integral += Range.clip(error, -30, 30) * deltaTime;
                if (Math.signum(Integral) != Math.signum(error)) {
                    Integral = 0;
                }
                derivative = (lastError - error) / deltaTime;
                swerveDrive.drive(0, 0,
                        proportional + Integral * ki + derivative * kd +
                                Math.signum(proportional + Integral * ki + derivative * kd) * 0.04,
                        0.5);
            }
            lastError = error;
            lastTime = currentTime;
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            Integral = 0;
            derivative = 0;
        }
    }
}
