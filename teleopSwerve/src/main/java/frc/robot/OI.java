package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import frc.robot.Constants.OIConstants;

public class OI {
    private static OI oi;

    public final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    public final XboxController operatorController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);

    private OI() {
    }

    /**
     * Singleton for getting instance of operator input
     *
     * @return OI object of self
     */
    public static OI getInstance() {
        if (oi == null) {
            oi = new OI();
        }
        return oi;
    }


    public double getDriverLeftY() {
        return driverController.getLeftY();
    }

    public double getOperatorLeftY() {
        return driverController.getLeftY();
    }

    public double getDriverLeftX() {
        return driverController.getLeftX();
    }

    public double getOperatorLeftX() {
        return driverController.getLeftX();
    }

    public double getDriverRightY() {
        return driverController.getRightY();
    }

    public double getOperatorRightY() {
        return driverController.getRightY();
    }

    public double getDriverRightX() {
        return driverController.getRightX();
    }

    public double getOperatorRightX() {
        return driverController.getRightX();
    }

    public double driverLeftTrigger() {
        return driverController.getLeftTriggerAxis();
    }

    public double operatorLeftTrigger() {
        return operatorController.getLeftTriggerAxis();
    }

    public double driverRightTrigger() {
        return driverController.getRightTriggerAxis();
    }

    public double operatorRightTrigger() {
        return operatorController.getRightTriggerAxis();
    }

    /**
     * Returns power for arm 
     * 1 or 1 if stick is past deadband in both directions
     * 0 if stick is within deadband
     * 
     * @return double controller left joystick power
     */
    public double getWristPower() {
        double power = getOperatorLeftY();
        return power;
    }


    /**
     * Returns the value of left joystick with values within deadband truncated
     *
     * @return double value of joystick
     */
    // public double getLeftYDeadband() {
    //     double leftY = getLeftY();
    //     if (Math.abs(leftY) < OIConstants.DEADBAND) {
    //         return 0;
    //     }

    //     return leftY;
    // }

    /**
     * Sets rumble value of controller to specified intensity
     * 
     * @param intensity double value to set up to 1
     */
    public void setRumble(double intensity) {
        controller.setRumble(RumbleType.kLeftRumble, intensity);
        controller.setRumble(RumbleType.kRightRumble, intensity);
    }

}