package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class setFeederPower extends Command {
    private final Feeder feeder;
    private final OI oi = OI.getInstance();

    public setFeederPower(Feeder subsystem) {
        feeder = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        feeder.setPower(oi.getFeederPower());

        SmartDashboard.putNumber("OI/Feeder Power", oi.getFeederPower());
    }
}
