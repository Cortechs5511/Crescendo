package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class setSpeakerPower extends Command {
    private final Intake intake;
    private final Wrist wrist;

    public setSpeakerPower(Intake intakeSubsystem, Wrist wristSubsystem) {
        intake = intakeSubsystem;
        wrist = wristSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        intake.setTopWheels(IntakeConstants.SPEAKER_POWER * 15 / 16);
        intake.setBottomWheels(IntakeConstants.SPEAKER_POWER);

        wrist.setPosition(0.85);
    }
}
