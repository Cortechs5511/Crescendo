package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class setIntakePower extends Command {
    private final Intake intake;
    private final OI oi = OI.getInstance();

    public setIntakePower(Intake subsystem) {
        intake = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setPower(oi.getIntakePower());

        SmartDashboard.putNumber("OI/Intake Power", oi.getIntakePower());
    }
}
