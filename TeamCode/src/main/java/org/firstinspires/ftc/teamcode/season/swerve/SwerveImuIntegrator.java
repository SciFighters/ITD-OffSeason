package org.firstinspires.ftc.teamcode.season.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.season.swerve.SwerveDrive;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;


public class SwerveImuIntegrator implements BNO055IMU.AccelerationIntegrator {

    final double tile = 0.6;

    private boolean useDashBoard;
    final double meters_to_inches = 39.37008;
    private ArrayList<Double> pathx;
    private ArrayList<Double> pathy;
    private long lastTimestamp = 0;

    private Point origin; // origin point of action
    private Point direction; // x,y direction for dashboard

    private double angularOffset = 0;
    private SwerveDrive swerveDrive;

    private BNO055IMU imu = null;
    BNO055IMU.Parameters parameters = null;

    Position position = new Position();
    Velocity velocity = new Velocity();
    Acceleration acceleration = null;

    public Position getPosition() {
        return this.position;
    }

    public Velocity getVelocity() {
        return this.velocity;
    }

    public Acceleration getAcceleration() {
        return this.acceleration;
    }


    public double getX() {
        return position.x;
    }

    public double getY() {
        return position.y;
    }

    public SwerveImuIntegrator(BNO055IMU imu, SwerveDrive swerveDrive, boolean useDahsboard, Point origin,
                               double angularOffset) {
        //, Point direction add to parameters later
        this.imu = imu;
        // Constructor
        this.origin = origin;
//        this.direction = direction;

        this.swerveDrive = swerveDrive;
        this.useDashBoard = useDahsboard;

        if (this.useDashBoard) {
            this.pathx = new ArrayList<>();
            this.pathy = new ArrayList<>();
        }
        this.angularOffset = angularOffset;
    }

    public void resetPosition() {
        position.x = 0;
        position.y = 0;
    }

    public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
        this.parameters = parameters;
        this.position = initialPosition != null ? initialPosition : this.position;
        this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
        this.acceleration = null;

        if (this.useDashBoard) {
            Point p = transformDashboard(this.position);
            this.pathx.add(p.x);
            this.pathy.add(p.y);
        }
    }

    public Point transformDashboard(Position pos) {
        double x = Math.signum(direction.x) * (pos.x + origin.x) * meters_to_inches;
        double y = Math.signum(direction.y) * (pos.y + origin.y) * meters_to_inches;
//		double z = (pos.z + origin_offset.z) * meters_to_inches;
        return new Point(x, y);
    }


    @Override
    public void update(Acceleration linearAcceleration) {

        if (this.useDashBoard && FtcDashboard.getInstance() != null && pathx.size() > 2) {
            Telemetry t = FtcDashboard.getInstance().getTelemetry();
            t.addData("[-2]", Arrays.toString(new double[]{pathx.get(pathx.size() - 2), pathy.get(pathy.size() - 2)}));
            t.addData("[-1]", Arrays.toString(new double[]{pathx.get(pathx.size() - 1), pathy.get(pathy.size() - 1)}));
        }


        double[] delta = swerveDrive.calcDeltaPos();


        this.position.x += delta[0];
        this.position.y += delta[1];

        // 100000000000 ps = 100 ms = 0.1 s
        if (this.useDashBoard && linearAcceleration.acquisitionTime - this.lastTimestamp >= 5000000L) {
            Point p = transformDashboard(this.position); // transform
            double lastx = pathx.get(pathx.size() - 1); // last path x
            double lasty = pathy.get(pathy.size() - 1); // last path y
            if (Math.abs(lastx - p.x) > 1 || Math.abs(lasty - p.y) > 1) {
                pathx.add(p.x);
                pathy.add(p.y);

                TelemetryPacket packet = new TelemetryPacket();
                Canvas canvas = packet.fieldOverlay();

                canvas.setStroke("tomato");
                canvas.strokePolyline(to_d_katan(pathx), to_d_katan(pathy));
                Point o = transformDashboard(new Position()); // transform 0,0
                canvas.fillCircle(o.x, o.y, 3);

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }

            this.lastTimestamp = linearAcceleration.acquisitionTime;
        }


    }

    private double[] to_d_katan(ArrayList<Double> arr) {
        double[] a = new double[arr.size()];
        for (int i = 0; i < arr.size(); i++) {
            a[i] = arr.get(i);
        }

        return a;
    }

}
