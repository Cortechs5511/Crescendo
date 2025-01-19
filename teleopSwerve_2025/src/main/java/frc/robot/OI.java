package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.OIConstants;

public class OI {
    private static OI oi;

    public final XboxController driverController = new XboxController(OIConstants.DRIVER_CONTROLLER_PORT);
    public final XboxController operatorController = new XboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

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
        double value = driverController.getLeftY() * 0.4;
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        if (driverController.getRightBumperButton()) {
            return value * 0.5;
        } else if (driverController.getLeftBumperButton()) {
            return value * 0.1;
        } else {
            return value;
        }
    }

    public double getOperatorLeftY() {
        double value = operatorController.getLeftY();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return -value;
    }

    public double getDriverLeftX() {
        double value = driverController.getLeftX() * 0.4;
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        if (driverController.getRightBumperButton()) {
            return value * 0.5;
        } else if (driverController.getLeftBumperButton()) {
            return value * 0.1;
        } else {
            return value;
        }
    }

    public double getOperatorLeftX() {
        double value = operatorController.getLeftX();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public double getDriverRightY() {
        double value = driverController.getRightY();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public double getOperatorRightY() {
        double value = operatorController.getRightY();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return -value;
    }

    public double getDriverRightX() {
        double value = driverController.getRightX();
        if (Math.abs(value) < OIConstants.DEADBAND + 0.1) {
            return 0;
        }
        if (driverController.getRightBumperButton()) {
            return value * 0.5;
        } else if (driverController.getLeftBumperButton()) {
            return value * 0.1;
        } else {
            return value;
        }
    }

    public double getOperatorRightX() {
        double value = operatorController.getRightX();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public double driverLeftTrigger() {
        double value = driverController.getLeftTriggerAxis();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public double operatorLeftTrigger() {
        double value = operatorController.getLeftTriggerAxis();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public double driverRightTrigger() {
        double value = driverController.getRightTriggerAxis();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public double operatorRightTrigger() {
        double value = operatorController.getRightTriggerAxis();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public boolean operatorY() {
        return operatorController.getYButton();
    }

    public boolean operatorX() {
        return operatorController.getXButton();
    }

    public boolean operatorB() {
        return operatorController.getBButton();
    }

    public boolean operatorA() {
        return operatorController.getAButton();
    }

    public boolean driverA() {
        return driverController.getAButton();
    }
    
    public boolean driverB() {
        return driverController.getBButton();
    }

    public boolean driverX() {
        return driverController.getXButton();
    }

    public double getWristPower() {
        return getOperatorLeftY();

    }

    // public double getWristPosition() {
    // // convert joystick range to 0-1
    // return (getOperatorLeftY()+1)/2;
    // }

    public double getIntakePower() {
        return operatorLeftTrigger();
    }

    public double getShooterPower() {
        return operatorRightTrigger();
    }

    public double getFeederPower() {
        if (operatorController.getLeftBumperButton()) {
            return getOperatorRightY() * 0.5;
        } else {
            return getOperatorRightY();
        }

    }

    /**
     * Sets rumble value of controller to specified intensity
     * 
     * @param intensity double value to set up to 1
     */
    public void setRumble(double intensity) {
        driverController.setRumble(RumbleType.kLeftRumble, intensity);
        driverController.setRumble(RumbleType.kRightRumble, intensity);

        operatorController.setRumble(RumbleType.kLeftRumble, intensity);
        operatorController.setRumble(RumbleType.kRightRumble, intensity);
    }

}