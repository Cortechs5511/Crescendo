package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;

public class setIntakePower extends Command {
    private final Intake intake;
    private final Feeder feeder;
    private final LEDs leds;
    private final OI oi = OI.getInstance();

    public setIntakePower(Intake intakeSubsystem, Feeder feederSubsystem, LEDs ledSubsystem) {
        intake = intakeSubsystem;
        feeder = feederSubsystem;
        leds = ledSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setPower(oi.getIntakePower() * IntakeConstants.INTAKE_POWER, 1, 1);
        leds.setLEDs(LEDConstants.INTAKE);

        SmartDashboard.putNumber("OI/Intake Power", oi.getIntakePower()*IntakeConstants.INTAKE_POWER);
    }
}
