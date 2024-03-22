package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

    private final RelativeEncoder leftRelativeEncoder = createEncoder(wristLeft);
    private final RelativeEncoder rightRelativeEncoder = createEncoder(wristRight);
    
    private final PIDController wristPIDController = new PIDController(0.03, 0, 0);


    private final DutyCycleEncoder absoluteEncoder = createDutyCycleEncoder(WristConstants.THROUGH_BORE_ID);


    public Wrist() {
        zero();
    }
    
    public void zero() {
        absoluteEncoder.reset();
        leftRelativeEncoder.setPosition(0);
        rightRelativeEncoder.setPosition(0);
    }

    
    public double getRawPosition() {
        double currentPosition = absoluteEncoder.getAbsolutePosition();
        
        
        return absoluteEncoder.getAbsolutePosition();
    }

    // convert from absolute encoder value to percent position
    public double getPercentPosition() {
        return (getRawPosition() - WristConstants.MIN_POS) / WristConstants.RANGE;
    }

    public void setPower(double power) {
        // 0.53, 0.6, 0.8, 0.99, 0, 0.04
        if (power > 0 && getRawPosition() < 0.525 && getRawPosition() > 0.1) {
            wristLeft.set(0);
            wristRight.set(0);
        }
        else {
            wristLeft.set(power);
            wristRight.set(power);
        }
        
    }
    // .62 SPEAKER POSITION
    //0.615 611,612

    // some distance away 6 in .617
    // side, 625 631
    // high 10 low 20s
    // set position of wrist given position range from 0.0-1.0 to encoder range
    public void setPosition(double desiredPos) {
        // convert given position from 0.0-1.0 to through bore encoder range
        // double translatedPosition = percentPosition * WristConstants.RANGE + WristConstants.MIN_POS;
        
        double wristOutput = wristPIDController.calculate(getRawPosition(), desiredPos);
        wristLeft.set(wristOutput);
        wristRight.set(wristOutput);
    }

    // 0.62
    public void setSpeakerPosition(double position) {
        if(getRawPosition() < position) {
            setPower(-0.1);
        }
        else {
            setPower(0);
        }

    }

    public double getLeftRelativePosition() {
        return leftRelativeEncoder.getPosition();
    }

    public double getRightRelativePosition() {
        return rightRelativeEncoder.getPosition();
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

    private RelativeEncoder createEncoder(CANSparkMax controller) {
        RelativeEncoder encoder = controller.getEncoder();
        encoder.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);

        return encoder;
    }

    private DutyCycleEncoder createDutyCycleEncoder(int channel) {
        DutyCycleEncoder encoder = new DutyCycleEncoder(channel);
        encoder.setDutyCycleRange(0, WristConstants.RANGE);
        encoder.setPositionOffset(WristConstants.OFFSET);
        return encoder;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist/Absolute Encoder Value", getRawPosition());
        SmartDashboard.putNumber("Wrist/Percentage Position", getPercentPosition());
        
        SmartDashboard.putNumber("Wrist/Left Relative Position", getLeftRelativePosition());
        SmartDashboard.putNumber("Wrist/Right Relative Position", getRightRelativePosition());
    }
}
