package org.firstinspires.ftc.teamcode.season.swerve;

public enum SwerveWheels {
    //372 wide 344 other wide
    FL(Math.atan(326.0 / 296.0)),
    FR(Math.PI - Math.atan(326.0 / 296.0)),
    BR(Math.PI + Math.atan(326.0 / 296.0)),
    BL(-Math.atan(326.0 / 296.0));

    public final double ANGLE;

    private SwerveWheels(double angle) {
        this.ANGLE = angle;
    }
}
