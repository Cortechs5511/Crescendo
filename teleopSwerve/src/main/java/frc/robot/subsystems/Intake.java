package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    
    private final CANSparkMax topWheels = createIntakeController(IntakeConstants.TOP_WHEELS_ID, false);
    private final CANSparkMax bottomWheels = createIntakeController(IntakeConstants.TOP_WHEELS_ID, true);

    public Intake() {

    }
    
    public void setPower(double power) {
        topWheels.set(power);
        bottomWheels.set(power);

    }

    private CANSparkMax createIntakeController(int port, boolean isInverted) {
        CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
        controller.restoreFactoryDefaults();

        controller.enableVoltageCompensation(IntakeConstants.VOLTAGE_COMPENSATION);
        controller.setIdleMode(IntakeConstants.INTAKE_IDLE_MODE);
        controller.setOpenLoopRampRate(IntakeConstants.RAMP_RATE);
        controller.setClosedLoopRampRate(IntakeConstants.RAMP_RATE); 

        controller.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);

        controller.setInverted(isInverted);
        return controller;
    }

    @Override
    public void periodic() {
        
    }
}
