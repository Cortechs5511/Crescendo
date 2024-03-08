package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;

public class setShooterPower extends Command {
    private final Intake intake;
    private final LEDs blinkin;
    private final OI oi = OI.getInstance();

    public setShooterPower(Intake intakeSubsystem, LEDs ledSubsystem) {
        intake = intakeSubsystem;
        blinkin = ledSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setPower(IntakeConstants.SHOOTER_POWER, 1, 1);
        blinkin.setLEDs(LEDConstants.SHOOTER);

        SmartDashboard.putNumber("OI/Shooter Power", oi.getShooterPower()*IntakeConstants.SHOOTER_POWER);
    }
}
