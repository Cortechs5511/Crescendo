package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wristR = createWristController(IntakeConstants.WRIST_R_ID, false);
    private final CANSparkMax wristL = createWristController(IntakeConstants.WRIST_L_ID, false);

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(IntakeConstants.THROUGH_BORE_ID);

    public Wrist() {

    }
    
    public void zeroEncoder() {
        absoluteEncoder.reset();
    }
    
    public double getWristPosition() {
        return Math.abs(absoluteEncoder.getAbsolutePosition());
    }

    public void setPower(double speed) {
        wristR.set(speed);
        wristL.set(-speed);
    }
    
    private CANSparkMax createWristController(int port, boolean isInverted) {
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
        SmartDashboard.putNumber("Arm/Absolute Encoder Value", getWristPosition());
    }
}
