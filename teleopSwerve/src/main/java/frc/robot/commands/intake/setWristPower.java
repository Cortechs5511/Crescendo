package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class setWristPower extends Command {
    private final Wrist wrist;
    private final OI oi = OI.getInstance();

    public setWristPower(Wrist subsystem) {
        wrist = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        wrist.setPower(oi.getWristPower());
    
        SmartDashboard.putNumber("OI/Arm Power", oi.getWristPower());
    }
}
