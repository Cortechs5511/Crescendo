package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    
    private final SparkMax topWheels = createIntakeController(IntakeConstants.TOP_WHEELS_ID, false);
    private final SparkMax bottomWheels = createIntakeController(IntakeConstants.BOTTOM_WHEELS_ID, false);

    public Intake() {

    }
    
    public void setPower(double power) {
        topWheels.set(power);
        bottomWheels.set(power);

    }

    public void setTopWheels(double power) {
        topWheels.set(power);
    }

    public void setBottomWheels(double power) {
        bottomWheels.set(power);
    }

    private SparkMax createIntakeController(int port, boolean isInverted) {
        SparkMax controller = new SparkMax(port, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();

        config.voltageCompensation(IntakeConstants.VOLTAGE_COMPENSATION);
        config.idleMode(IntakeConstants.INTAKE_IDLE_MODE);
        config.openLoopRampRate(IntakeConstants.RAMP_RATE);
        config.closedLoopRampRate(IntakeConstants.RAMP_RATE); 

        config.smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);

        config.inverted(isInverted);

        controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        return controller;
    }

    @Override
    public void periodic() {
        
    }
}
