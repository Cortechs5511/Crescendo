package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDs;

public class setLEDAlliance extends Command {
    private final LEDs leds;
    private final Optional<Alliance> ally = DriverStation.getAlliance();

    public setLEDAlliance(LEDs subsystem) {
        leds = subsystem;
        addRequirements(leds);
    }

    @Override
    public void initialize() {
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                leds.setLEDs(LEDConstants.RED_ALLIANCE);
            }
            if (ally.get() == Alliance.Blue) {
                leds.setLEDs(LEDConstants.BLUE_ALLIANCE);
            }
        }
        else {

        }
    }

    @Override
    public void execute() {

    }
}
