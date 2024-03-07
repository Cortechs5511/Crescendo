package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;

public class feederIntake extends Command {
    private final Feeder feeder;
    private final OI oi = OI.getInstance();

    public feederIntake(Feeder feederSubsystem) {
        feeder = feederSubsystem;
        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        feeder.setPower(IntakeConstants.FEEDER_INTAKE_POWER);

        SmartDashboard.putNumber("OI/Intake Power", oi.getIntakePower()*IntakeConstants.INTAKE_POWER);
    }
}
