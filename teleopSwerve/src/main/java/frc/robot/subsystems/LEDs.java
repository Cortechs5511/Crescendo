package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LEDConstants;


public class LEDs extends SubsystemBase{
    private final Spark blinkin = new Spark(LEDConstants.BLINKIN_CHANNEL);

    public LEDs() {

    }

    public void setLEDs(double colorID) {
        blinkin.set(colorID);
    }

    public double getLEDS() {
        return blinkin.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LED Value", getLEDS());
    }
}
