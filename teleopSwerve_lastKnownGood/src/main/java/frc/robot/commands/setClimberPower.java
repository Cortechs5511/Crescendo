package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Climber;

public class setClimberPower extends Command {
    private final Climber climber;
    private final OI oi = OI.getInstance();

    public setClimberPower(Climber subsystem) {
        climber = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        if (oi.operatorY()) {
            climber.setPower(0.3);
        }
        else if (oi.operatorX()) {
            climber.setLeftPower(-0.3);
        }
        else if (oi.operatorB()) {
            climber.setRightPower(-0.3);
        }
        else {
            climber.setPower(0);
        }
        
    }
}
