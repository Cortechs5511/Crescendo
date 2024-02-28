package frc.robot.commands.intake;

import frc.robot.OI;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class setIntakePower extends Command {
    private final Intake intake;
    private final LEDs blinkin;
    private final OI oi = OI.getInstance();

    public setIntakePower(Intake subsystem, LEDs ledSubsystem) {
        intake = subsystem;
        blinkin = ledSubsystem;
        addRequirements(subsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setPower(oi.getIntakePower());
        blinkin.setLEDs(-0.99);

        SmartDashboard.putNumber("OI/Intake Power", oi.getIntakePower());
    }
}
