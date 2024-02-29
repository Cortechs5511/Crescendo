package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class setWristPosition extends Command {
    private final Wrist wrist;
    private final OI oi = OI.getInstance();


    public setWristPosition(Wrist subsystem) {
        wrist = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        wrist.setPosition(oi.getWristPower());
    
        // SmartDashboard.putNumber("OI/Wrist Power", oi.getWristPower());
    }
}
