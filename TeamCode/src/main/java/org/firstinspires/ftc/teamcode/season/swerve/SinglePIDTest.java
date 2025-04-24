package org.firstinspires.ftc.teamcode.season.swerve;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "PIDFinder")
public class SinglePIDTest extends LinearOpMode {
    public SwerveModule bl, br, fl, fr;
    public static boolean iThing = false;
    public static double p = 0.25, w = -0.75;
    public static double noise = 10, noiseCutoff = 1;
    public static double iZone = 5, ki = 0, iResetPoint = 10;
    public static double period = 2;
    public static String wheel = "tl";
    public static boolean sin = false;
    double targetPos = 180, currentPos = 0;
    public static double target1 = 90, target2 = 180;
    double switchTime = 0, timeOn90;
    private ElapsedTime runtime = new ElapsedTime();
    double i = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {
        bl = new SwerveModule(hardwareMap.get(DcMotor.class, "bl_motor"),
                hardwareMap.get(CRServo.class, "bl_servo"),
                hardwareMap.analogInput.get("bl_encoder"), 314.64);
        br = new SwerveModule(hardwareMap.get(DcMotor.class, "br_motor"),
                hardwareMap.get(CRServo.class, "br_servo"),
                hardwareMap.analogInput.get("br_encoder"), 54.93);
        fl = new SwerveModule(hardwareMap.get(DcMotor.class, "fl_motor"),
                hardwareMap.get(CRServo.class, "fl_servo"),
                hardwareMap.analogInput.get("fl_encoder"), 353.5);
        fr = new SwerveModule(hardwareMap.get(DcMotor.class, "fr_motor"),
                hardwareMap.get(CRServo.class, "fr_servo"),
                hardwareMap.analogInput.get("fr_encoder"), 346.9);
        waitForStart();

        runtime.reset();

        while (opModeIsActive()) {
            if (sin) {
                targetPos = Math.sin(runtime.seconds() * Math.PI * period)
                        * (target2 - target1) / 2 + (target1 + target2) / 2;
            } else if (runtime.seconds() >= switchTime + period) {
                if (targetPos == target2) {
                    targetPos = target1;
                } else {
                    targetPos = target2;
                }
                switchTime = runtime.seconds();
            }
            setTargetAngle(targetPos);
            dashboardTelemetry.addData("fl pos",fl.getCurrentHeading());
            dashboardTelemetry.addData("br pos",br.getCurrentHeading());
            dashboardTelemetry.addData("bl pos",bl.getCurrentHeading());
            dashboardTelemetry.addData("fr pos",fr.getCurrentHeading());
            dashboardTelemetry.addData("wanted",targetPos);
            dashboardTelemetry.update();

        }
    }
    private void setTargetAngle(double angle){
        fl.setHeading(angle,false);
        br.setHeading(angle,false);
        bl.setHeading(angle,false);
        fr.setHeading(angle,false);
        fl.update();
        br.update();
        bl.update();
        fr.update();

    }


    private double signroot(double x) {
        return Math.sqrt(Math.abs(x)) * Math.signum(x);
    }

    private double calcDeltaAngle(double target, double current) {
        double delta = target - current;
        if (Math.abs(delta) >= 180) {
            delta -= Math.signum(delta) * 360;
        }
        return delta;
    }

}
//            if (sin) {
//                targetPos = Math.sin(runtime.seconds() * Math.PI * period)
//                        * (target2 - target1) / 2 + (target1 + target2) / 2;
//            } else if (runtime.seconds() >= switchTime + period) {
//                if (targetPos == target2) {
//                    targetPos = target1;
//                } else {
//                    targetPos = target2;
//                }
//                switchTime = runtime.seconds();
//            }
//
//            currentPos = encoder.getVoltage() / 3.274 * 360;
//
//            double e = calcDeltaAngle(targetPos, currentPos);
//
//            if (Math.abs(targetPos - currentPos) * 0.1 < e) {
//                timeOn90 = runtime.seconds() - switchTime;
//            }
////            if(iThing){
////                if(currentPos <= iZone){
////                    if( i > iResetPoint){
////                        i = 0;
////                    }
////                    i += e;
////                }
////            }
//            if (Math.abs(e) > noiseCutoff) {
//                e += Math.random() * Math.signum(e) * noise;
//            }
//            double ne = -e / 90;
//            //double w = 1 - Math.abs(ne);
//            double power = (w >= 0) ? (signroot(ne) * (w) + ne * (1 - w)) * p :
//                    (ne * Math.abs(ne) * (Math.abs(w)) + ne * (1 + w)) * p;
//            dashboardTelemetry.addData("e", -e);
//            dashboardTelemetry.addData("power", power);
//            dashboardTelemetry.addData("current", currentPos);
//            telemetry.addData("current", currentPos);
//            dashboardTelemetry.addData("target", targetPos);
//            telemetry.addData("target", targetPos);
//            dashboardTelemetry.addData("time on 90", timeOn90);
//            dashboardTelemetry.update();
//            telemetry.update();