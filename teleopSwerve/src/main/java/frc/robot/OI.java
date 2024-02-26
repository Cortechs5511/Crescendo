package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeConstants;

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
        double value = driverController.getLeftY();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public double getOperatorLeftY() {
        double value = operatorController.getLeftY();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
    }

    public double getDriverLeftX() {
        double value = driverController.getLeftX();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
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
        return value;
    }

    public double getDriverRightX() {
        double value = driverController.getRightX();
        if (Math.abs(value) < OIConstants.DEADBAND) {
            return 0;
        }
        return value;
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

    
    public double getWristPower() {
        return getOperatorLeftY() * IntakeConstants.WRIST_MAX_POWER;
    }

    public double getIntakePower() {
        return operatorLeftTrigger() * IntakeConstants.INTAKE_MAX_POWER;
    }

    public double getFeederPower() {
        return operatorRightTrigger() * IntakeConstants.FEEDER_MAX_POWER;
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