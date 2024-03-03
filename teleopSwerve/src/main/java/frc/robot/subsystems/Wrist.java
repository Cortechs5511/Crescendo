package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wristLeft = createWristController(WristConstants.WRIST_L_ID, true);
    private final CANSparkMax wristRight = createWristController(WristConstants.WRIST_R_ID, false);
    
    private final PIDController wristPIDController = new PIDController(0.03, 0, 0);


    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(WristConstants.THROUGH_BORE_ID);

    public Wrist() {

    }
    
    public void zeroEncoder() {
        absoluteEncoder.reset();
    }
    
    public double getRawPosition() {
        return Math.abs(absoluteEncoder.getAbsolutePosition());
    }

    public double getPosition() {
        return WristConstants.MIN_POS+(absoluteEncoder.getAbsolutePosition()/(WristConstants.MIN_POS-WristConstants.MAX_POS));
    }

    public void setPower(double power) { //wyan
        wristLeft.set(power);
        wristRight.set(power);
    }

    // set position of wrist given position range from 0.0-1.0 to encoder range
    public void setPosition(double position) {
        double range = WristConstants.MAX_POS-WristConstants.MIN_POS;
        double translatedPosition = position * range + WristConstants.MIN_POS;
        final double wristOutput = wristPIDController.calculate(absoluteEncoder.getAbsolutePosition(), translatedPosition);
        wristLeft.set(wristOutput);
        wristRight.set(wristOutput);
    }
    
    private CANSparkMax createWristController(int port, boolean isInverted) {
        CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
        controller.restoreFactoryDefaults();

        controller.enableVoltageCompensation(WristConstants.VOLTAGE_COMPENSATION);
        controller.setIdleMode(WristConstants.IDLE_MODE);
        controller.setOpenLoopRampRate(WristConstants.RAMP_RATE);
        controller.setClosedLoopRampRate(WristConstants.RAMP_RATE); 

        controller.setSmartCurrentLimit(WristConstants.CURRENT_LIMIT);

        controller.setInverted(isInverted);
        return controller;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist/Absolute Encoder Value", getRawPosition());
        SmartDashboard.putNumber("Wrist/Persentage PositionValue", getPosition());
    }
}
