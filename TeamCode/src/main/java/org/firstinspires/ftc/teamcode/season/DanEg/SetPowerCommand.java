package org.firstinspires.ftc.teamcode.season.DanEg;

import com.arcrobotics.ftclib.command.CommandBase;

public class SetPowerCommand extends CommandBase {
    SampleSubsystem sub;
    double _powerSupplier;

    public SetPowerCommand(SampleSubsystem subsystem, double powerSupplier) {
        this.sub = subsystem;
        this._powerSupplier = powerSupplier;
        addRequirements(sub);
    }

    @Override
    public void execute() {
        this.sub.setPower(_powerSupplier);
    }

    @Override
    public void end(boolean interrupted) {
        this.sub.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
