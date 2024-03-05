package frc.robot.commands.intake;

import frc.robot.Constants.IntakeConstants;
import frc.robot.OI;
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class setFeederPower extends Command {
    private final Feeder feeder;

    public setFeederPower(Feeder subsystem) {
        feeder = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        feeder.setPower(-1 * IntakeConstants.FEEDER_POWER);

    }
}
