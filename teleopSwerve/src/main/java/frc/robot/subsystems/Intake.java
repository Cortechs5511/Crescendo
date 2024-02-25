package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax topWheels = createIntakeController(IntakeConstants.TOP_WHEEL_ID, false);
    private final CANSparkMax bottomWheels = createIntakeController(IntakeConstants.TOP_WHEEL_ID, true);

    public Intake() {

    }
    
    public void setPower(double speed) {
        topWheels.set(speed);
        bottomWheels.set(speed);
    }

    private CANSparkMax createIntakeController(int port, boolean isInverted) {
        CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
        controller.restoreFactoryDefaults();

        controller.enableVoltageCompensation(IntakeConstants.WRIST_VOLTAGE_COMPENSATION);
        controller.setIdleMode(IntakeConstants.WRIST_IDLE_MODE);
        controller.setOpenLoopRampRate(IntakeConstants.RAMP_RATE);
        controller.setClosedLoopRampRate(IntakeConstants.RAMP_RATE); 

        controller.setSmartCurrentLimit(IntakeConstants.WRIST_CURRENT_LIMIT);

        controller.setInverted(isInverted);
        return controller;
    }

    @Override
    public void periodic() {
        
    }
}
