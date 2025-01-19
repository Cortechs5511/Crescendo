package frc.robot.commands.intake;

import frc.robot.Constants.WristConstants;
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
        if (oi.operatorA()) {
            wrist.setSpeakerAngle(oi.getWristPower()*WristConstants.MAX_POWER);
        } else {
            wrist.setPower(oi.getWristPower()*WristConstants.MAX_POWER);
        }
        SmartDashboard.putNumber("OI/Wrist Power", oi.getWristPower());
    }
}
