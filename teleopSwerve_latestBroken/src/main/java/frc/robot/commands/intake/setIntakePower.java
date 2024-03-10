package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;

public class setIntakePower extends Command {
    private final Intake intake;
    private final Wrist wrist;
    private final LEDs leds;
    private final OI oi = OI.getInstance();

    public setIntakePower(Intake intakeSubsystem, Wrist wristSubsystem, LEDs ledSubsystem) {
        intake = intakeSubsystem;
        leds = ledSubsystem;
        wrist = wristSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(ledSubsystem);
        addRequirements(wristSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intake.setPower(oi.getIntakePower() * IntakeConstants.INTAKE_POWER);
        
        leds.setLEDs(LEDConstants.INTAKE);

        SmartDashboard.putNumber("OI/Intake Power", oi.getIntakePower()*IntakeConstants.INTAKE_POWER);
    }
}
