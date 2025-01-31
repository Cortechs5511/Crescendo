package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;


public class Climber extends SubsystemBase {
    private final SparkMax leftClimber = createClimberController(ClimberConstants.CLIMBER_L_ID, false);
    private final SparkMax rightClimber = createClimberController(ClimberConstants.CLIMBER_R_ID, false);
    
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
    
    private SparkMax createClimberController(int port, boolean isInverted) {
        SparkMax controller = new SparkMax(port, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.voltageCompensation(ClimberConstants.VOLTAGE_COMPENSATION);
        config.idleMode(ClimberConstants.IDLE_MODE);
        config.openLoopRampRate(ClimberConstants.RAMP_RATE);
        config.closedLoopRampRate(ClimberConstants.RAMP_RATE); 

        config.smartCurrentLimit(ClimberConstants.CURRENT_LIMIT);

        config.inverted(isInverted);
        config.encoder.positionConversionFactor(ClimberConstants.CONVERSION_FACTOR);
        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        return controller;
    }

    private RelativeEncoder createClimberEncoder(SparkMax controller) {
        RelativeEncoder encoder = controller.getEncoder();
        //encoder.setPositionConversionFactor(ClimberConstants.CONVERSION_FACTOR);
        return encoder;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Left Position", getLeftPosition());
        SmartDashboard.putNumber("Climber/Right Position", getRightPosition());
    }
}
