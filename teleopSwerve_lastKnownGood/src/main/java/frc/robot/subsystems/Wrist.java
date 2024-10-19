package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wristLeft = createWristController(WristConstants.WRIST_L_ID, true);
    private final CANSparkMax wristRight = createWristController(WristConstants.WRIST_R_ID, false);

    private final RelativeEncoder leftRelativeEncoder = createEncoder(wristLeft);
    private final RelativeEncoder rightRelativeEncoder = createEncoder(wristRight);

    private final PIDController wristPID = new PIDController(WristConstants.PID_VALUES[0], WristConstants.PID_VALUES[1],
            WristConstants.PID_VALUES[2]);

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
        return absoluteEncoder.getAbsolutePosition();
    }

    public double getRawDistance() {
        return absoluteEncoder.getDistance();
    }

    // convert range of wrist from 0.525 to 1.0 and 0.0 to ~0.3
    // to 0 to ~0.8
    public double getTranslatedPosition() {
        double rawPosition = getRawPosition();
        if (rawPosition >= 0.525) {
            return rawPosition - 0.525;
        } else {
            return rawPosition + 0.525;
        }

    }

    // convert from translated position to percent position
    public double getPercentPosition() {
        return (getTranslatedPosition() - WristConstants.MIN_POS) / WristConstants.RANGE;
    }

    public void setSpeakerAngle(double power) {
        if (power < 0 && getRawPosition() > 0.32) {
            setPower(0);
        } else {
            setPower(power);
        }
    }

    public void setPower(double power) {
        // 0.53, 0.6, 0.8, 0.99, 0, 0.04
        if (power > 0 && getRawPosition() < 0.17) {
        wristLeft.set(0);
        wristRight.set(0);
        }
        else if (power < 0 && getRawPosition() > 0.83) {
        wristLeft.set(0);
        wristRight.set(0);
        }
        else {
        wristLeft.set(power);
        wristRight.set(power);
        }
        // wristLeft.set(power);
        // wristRight.set(power);

    }

    public double[] getPower() {
        double[] currentPower = { wristLeft.get(), wristRight.get() };
        return currentPower;
    }

    // 0.62
    public void setDistance(double distance) {
        if (getRawDistance() < distance) {
            setPower(-0.1);
        } else {
            setPower(0);
        }
    }

    public void setPositionPID(double position) {
        double wristOutput = wristPID.calculate(getRawDistance(), position);
        setPower(-wristOutput);
    }

    public void setPercentPosition(double percentPosition) {

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
        SmartDashboard.putNumber("Wrist/Raw Position", getRawPosition());
        SmartDashboard.putNumber("Wrist/Raw Distance", getRawDistance());
        SmartDashboard.putNumber("Wrist/Translated Position", getTranslatedPosition());

        SmartDashboard.putNumber("Wrist/Left Relative Position", getLeftRelativePosition());
        SmartDashboard.putNumber("Wrist/Right Relative Position", getRightRelativePosition());

        SmartDashboard.putNumberArray("Wrist Powers", getPower());
    }
}
