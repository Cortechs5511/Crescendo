package frc.robot.commands.shooter;

import frc.robot.OI;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;

public class setShooterPower extends Command {
    private final Intake intake;
    private final Feeder feeder;
    private final LEDs blinkin;
    private final Timer timer = new Timer();
    private final OI oi = OI.getInstance();

    public setShooterPower(Intake intakeSubsystem, Feeder feederSubsystem, LEDs ledSubsystem) {
        intake = intakeSubsystem;
        feeder = feederSubsystem;
        blinkin = ledSubsystem;
        addRequirements(intakeSubsystem);
        addRequirements(feederSubsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        // intake.setPower(oi.getShooterPower() * IntakeConstants.SHOOTER_POWER);
        intake.setTopWheels(oi.getShooterPower()*-0.27);
        intake.setBottomWheels(oi.getShooterPower()*-0.27);
        blinkin.setLEDs(LEDConstants.SHOOTER);
        if (timer.hasElapsed(1)) {
            feeder.setPower(-0.9);
        }

        SmartDashboard.putNumber("OI/Shooter Power", oi.getShooterPower()*IntakeConstants.SHOOTER_POWER);
    }
}
