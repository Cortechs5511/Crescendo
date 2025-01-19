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
    double upperPosition;
    double lowerPosition;

    double targetPosition = 0.082;

    public setSpeakerPower(Intake intakeSubsystem, Feeder feederSubsystem, Wrist wristSubsystem, double upperRange,
            double lowerRange) {
        intake = intakeSubsystem;
        feeder = feederSubsystem;
        wrist = wristSubsystem;
        upperPosition = upperRange;
        lowerPosition = lowerRange;
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
        // wrist.setDistance(targetPosition);
        // wrist.setPositionPID(targetPosition);
        // 0.626 and 0.620
        // if (timer.hasElapsed(1) && wrist.getRawDistance() <= targetPosition + 0.003
        // && wrist.getRawDistance() >= targetPosition - 0.003) {
        // feeder.setPower(-1);
        // }
        if (timer.hasElapsed(1)) {
            feeder.setPower(-1);
        }

    }
}
