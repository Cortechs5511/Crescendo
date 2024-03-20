package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase {
    private final CANSparkMax leftClimber = createClimberController(ClimberConstants.CLIMBER_L_ID, false);
    private final CANSparkMax rightClimber = createClimberController(ClimberConstants.CLIMBER_R_ID, false);
    
    private final RelativeEncoder leftRelativeEncoder = createClimberEncoder(leftClimber);
    private final RelativeEncoder rightRelativeEncoder = createClimberEncoder(rightClimber);

    public Climber() {
        zero();
    }
    
    public void setPower(double power) {
        leftClimber.set(power);
        rightClimber.set(power);
    }

    public void setLeftPower(double power) {
        leftClimber.set(power);
    }
    
    public void setRightPower(double power) {
        rightClimber.set(power);
    }

    public double getLeftPosition() {
        return leftRelativeEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightRelativeEncoder.getPosition();
    }

    public void zero() {
        leftRelativeEncoder.setPosition(0);
        rightRelativeEncoder.setPosition(0);
    }
    
    private CANSparkMax createClimberController(int port, boolean isInverted) {
        CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
        controller.restoreFactoryDefaults();

        controller.enableVoltageCompensation(ClimberConstants.VOLTAGE_COMPENSATION);
        controller.setIdleMode(ClimberConstants.IDLE_MODE);
        controller.setOpenLoopRampRate(ClimberConstants.RAMP_RATE);
        controller.setClosedLoopRampRate(ClimberConstants.RAMP_RATE); 

        controller.setSmartCurrentLimit(ClimberConstants.CURRENT_LIMIT);

        controller.setInverted(isInverted);
        return controller;
    }

    private RelativeEncoder createClimberEncoder(CANSparkMax controller) {
        RelativeEncoder encoder = controller.getEncoder();
        encoder.setPositionConversionFactor(ClimberConstants.CONVERSION_FACTOR);
        return encoder;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Left Position", getLeftPosition());
        SmartDashboard.putNumber("Climber/Right Position", getRightPosition());
    }
}
