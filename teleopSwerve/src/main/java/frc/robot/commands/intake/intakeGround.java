package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class intakeGround extends Command {
    private final Intake intake;
    private final Feeder feeder;
    private final Wrist wrist;
    private final LEDs blinkin;
    private final OI oi = OI.getInstance();
    

    public intakeGround(Intake intakeSubsystem, Wrist wristSubsystem, Feeder feederSubsystem, LEDs ledSubsystem ) {
        intake = intakeSubsystem;
        feeder = feederSubsystem;
        wrist = wristSubsystem;
        blinkin = ledSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(wristSubsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        blinkin.setLEDs(LEDConstants.INTAKE);
        intake.setPower(IntakeConstants.INTAKE_POWER, 1, 1);
        feeder.setPower(IntakeConstants.FEEDER_INTAKE_POWER);
        wrist.setPosition(0);

        // in theory doesn't need a stop, will just turn on only when trigger is pressed
    }
    
}