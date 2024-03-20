package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj.Timer;


public class setSpeakerPower extends Command {
    private final Intake intake;
    private final Wrist wrist;
    private final Feeder feeder;
    private final Timer timer = new Timer();

    public setSpeakerPower(Intake intakeSubsystem, Feeder feederSubsystem, Wrist wristSubsystem) {
        intake = intakeSubsystem;
        feeder = feederSubsystem;
        wrist = wristSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        intake.setTopWheels(IntakeConstants.SPEAKER_POWER * 15 / 16);
        intake.setBottomWheels(IntakeConstants.SPEAKER_POWER);
        if (timer.hasElapsed(1)) {
            feeder.setPower(-1);
        }

    }
}
