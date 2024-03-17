package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class Feeder extends SubsystemBase {
    private final CANSparkMax topFeeder = createFeederController(IntakeConstants.TOP_FEEDER_ID, false);
    private final CANSparkMax bottomFeeder = createFeederController(IntakeConstants.BOTTOM_FEEDER_ID, false);

    public Feeder() {

    }

    public void setPower(double power) {
        topFeeder.set(power);
        bottomFeeder.set(power);
    }
    
    public CANSparkMax createFeederController(int port, boolean isInverted) {
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

}
