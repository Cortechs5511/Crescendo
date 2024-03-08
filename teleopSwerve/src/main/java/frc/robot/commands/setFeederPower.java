package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Feeder;

public class setFeederPower extends Command {
    private final Feeder feeder;
    private final OI oi = OI.getInstance();

    public setFeederPower(Feeder subsystem) {
        feeder = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        feeder.setPower(oi.getFeederPower());
    }
}
