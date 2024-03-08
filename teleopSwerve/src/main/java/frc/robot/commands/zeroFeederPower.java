package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.OI;
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class zeroFeederPower extends Command {
    private final Feeder feeder;
    private final OI oi = OI.getInstance();

    public zeroFeederPower(Feeder subsystem) {
        feeder = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        feeder.setPower(0);
        
    }

}
