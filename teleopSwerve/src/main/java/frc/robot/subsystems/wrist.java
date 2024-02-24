package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;;

public class wrist extends SubsystemBase {
    private final CANSparkMax wrist = createWristController(IntakeConstants.WRIST_ID, false);

    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(IntakeConstants.THROUGH_BORE_ID);


    
    public void zeroEncoder() {
        absoluteEncoder.reset();
    }
    
    public double getWristPosition() {
        return Math.abs(absoluteEncoder.getAbsolutePosition());
    }

    public void setPower(double speed) {
        wrist.set(speed);
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