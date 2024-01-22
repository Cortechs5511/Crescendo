// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
// import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import com.kauailabs.navx.frc.AHRS;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String Clockwise = "Ryan's Auto Turn Clockwise";
  private static final String Anticlockwise = "Ryan's Auto Turn Anticlockwise";
  private static final String Forward = "Ryan's Auto Forward";
  private static final String Backward = "Ryan's Auto Backward";
  private static final String forwardDist = "Ryan's Forward Distance In";
  private static final String Turn90 = "Turn90";
  private static final String complexAuto = "Ryan's Complex Auto";
  private static final double positionToIn = 177.5 / ((75.479 + 76.455) / 2);
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private CANSparkMax rightMotor1 = new CANSparkMax(20, MotorType.kBrushless); 
  private CANSparkMax rightMotor2 = new CANSparkMax(11, MotorType.kBrushless); 
  private CANSparkMax leftMotor1 = new CANSparkMax(10, MotorType.kBrushless); 
  private CANSparkMax leftMotor2 = new CANSparkMax(51, MotorType.kBrushless); 
  private final SendableChooser<String> driveMode = new SendableChooser<>();
  private static final String tankDrive = "Ryan's Tank Drive";
  private static final String arcadeDrive = "Ryan's Arcade Drive";
  private RelativeEncoder leftEnc;
  private RelativeEncoder rightEnc;
  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  private Joystick controller = new Joystick(2);
  private Timer autonTimer = new Timer();
  private Timer lightTimer = new Timer();
  private Boolean autonDone = false;
  private AHRS direction = new AHRS();
  // private AddressableLED LEDs = new AddressableLED(0);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Ryan's Auto Turn Clockwise", Clockwise);
    m_chooser.addOption("Ryan's Auto Turn Anticlockwise", Anticlockwise);
    m_chooser.addOption("Ryan's Auto Forward", Forward);
    m_chooser.addOption("Ryan's Auto Backward", Backward);
    m_chooser.addOption("Turn 90", Turn90);
    m_chooser.addOption("Ryan's Complex Auto", complexAuto);
    m_chooser.addOption(forwardDist, forwardDist);
    SmartDashboard.putData("Ryan's Auto choices", m_chooser);
    
    driveMode.setDefaultOption("Tank Auto", tankDrive);
    driveMode.addOption("Arcade Auto", arcadeDrive);
    SmartDashboard.putData("Ryan's Teleop choices", driveMode);
    
    leftEnc = leftMotor1.getEncoder();
    rightEnc = rightMotor1.getEncoder();
    SmartDashboard.putNumber("auton/Length", 154);
    SmartDashboard.putNumber("auton/Width", 92);
    SmartDashboard.putNumber("auton/Angle", -90);
    SmartDashboard.putNumber("auton/Length Addition", 17);
    SmartDashboard.putNumber("auton/Angle Subtraction", 5);
    SmartDashboard.putNumber("auton/Width Subtraction", 12);
    // LEDs.setLength(16);
    // fillColor(0, 255, 0);
    // LEDs.start();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Direction", direction.getAngle());
    SmartDashboard.putNumber("LeftEnc", -leftEnc.getPosition());
    SmartDashboard.putNumber("RightEnc", rightEnc.getPosition());
    SmartDashboard.putNumber("LeftIn", -leftEnc.getPosition() * positionToIn);
    SmartDashboard.putNumber("RightIn", getRightDistance());
    SmartDashboard.putNumber("Accel Y", accelerometer.getY());
    SmartDashboard.putNumber("Current Angle", direction.getAngle());
  }

  private void fillColor(int r, int g, int b) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(16);
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }
    // LEDs.setData(buffer);
  }

  private void fillPercent(int r, int g, int b, double percent) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(16);
    int stop = (int)(percent * buffer.getLength());
    
    for (int i = 0; i < buffer.getLength(); i++) {
      if (i <= stop) {
        buffer.setRGB(i, r, g, b);
      } 
      else {
        buffer.setRGB(i, 0, 0, 0);
      }
    }
    // LEDs.setData(buffer);
  }

  private boolean simpleTurn(double targetAngle) {
    
    double currentAngle = direction.getAngle();
    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putNumber("Current Angle", currentAngle);
  
    double power = 0;
    double maxPower = 0.3;
    double percent = Math.abs(currentAngle / targetAngle);
    fillPercent(37, 100, 10, percent);
    if (currentAngle < targetAngle) {
      double rampAngle = 20;
      double error = targetAngle - currentAngle;
      if (error < rampAngle) {
        power = maxPower * (error / rampAngle);
        power = Math.max(power, 0.1);
        if (error < 1) {
          power = 0;
          return true;
        }
        setMotorPower(power, -power);
        
      }
      else {
        power = maxPower;
        setMotorPower(power, -power);
      }
      
    }

    else if (currentAngle > targetAngle) {
      double error = currentAngle - targetAngle;
      double rampAngle = 20;
      if (error < rampAngle) {
        power = maxPower * (error / rampAngle);
        power = Math.max(power, 0.1);
        if (error < 1) {
          power = 0;
          return true;
        }
        setMotorPower(-power, power);
      }
      else {
        power = maxPower;
        setMotorPower(-power, power);
    }
    SmartDashboard.putNumber("Power", power);
    }
    return false;
    
  }
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autonTimer.reset();
    autonTimer.start();
    zero();
    autonDone = false;
    autonState = 0;
    fillColor(40, 30, 10);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (autonDone) {
      return;
    }
    // double encoderCount = (leftEnc.getPosition() + rightEnc.getPosition()) / 2;
    switch (m_autoSelected) {
      case Turn90:
        simpleTurn(getNumber("auton/Angle", 90));
        break;
      case Clockwise:
        if (autonTimer.hasElapsed(1.0)) {
          stopMotor();
        }
        else {
          setMotorPower(0.5, -0.5);
        }
        
        // Put custom auto code here
        break;
      case Anticlockwise:
        if (autonTimer.hasElapsed(1.0)) {
          stopMotor();
        }
        else {
          setMotorPower(-0.5, 0.5);
        }
        
        // Put custom auto code here
        break;
      case Forward:
        if (autonTimer.hasElapsed(1.0)) {
          stopMotor();
        }
        else {
          setMotorPower(0.5, 0.5);
        }
        // Put custom auto code here
        break;
      case Backward:
        if (autonTimer.hasElapsed(1.0)) {
          stopMotor();
        }
        else {
          setMotorPower(-0.5, -0.5);
        }
        // Put custom auto code here
      case forwardDist:
        autonDone = autoDist(getNumber("Auto Distance", 120));
        break;
      case complexAuto:
        complexAuto();
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    SmartDashboard.putNumber("Auton Time", autonTimer.get());
  }

  private int autonState;

  private void complexAuto() {
    double length = SmartDashboard.getNumber("auton/Length", 154);
    double width = SmartDashboard.getNumber("auton/Width", 92);
    double angle = SmartDashboard.getNumber("auton/Angle", -90);
    double lengthAddition = SmartDashboard.getNumber("auton/Length Addition", 17);
    double angleSubtraction = SmartDashboard.getNumber("auton/Angle Subtraction", 5);
    double widthSubtraction = SmartDashboard.getNumber("auton/Width Subtraction", 12);
    
    switch (autonState) {
      case 0:
       if (autoDist(length)) {
        zeroDistance();
        autonState++;
       }
      break;
      case 1:
       if (simpleTurn(angle)) {
        zeroDistance();
        autonState++;
       }
      break;
      case 2:
       if (autoDist(width)) {
        zeroDistance();
         autonState++;
       }
      break;
      case 3:
       if (simpleTurn(angle - 90)) {
        zeroDistance();
         autonState++;
       }
      break;
      case 4:
       if (autoDist(length + lengthAddition)) {
        zeroDistance();
        autonState++;
       }
      break;
      case 5:
       if (simpleTurn(angle - 180 - angleSubtraction)) {
        zeroDistance();
        autonState++;
       }
      break;
      case 6:
       if (autoDist(width - widthSubtraction)) {
        zeroDistance();
        autonState++;
       }
      break;
      case 7:
       if (simpleTurn(angle - 270)) {
        zeroDistance();
        autonState++;
       }
      break;
      default:
       stopMotor();
    }
  }

  private boolean autoDist(double targetDist) {
    double dist = getRightDistance();
    double maxPower = 0.4;
    double rampDistance = 30;
    double percent = (dist / targetDist);
    fillPercent(100, 12, 37, percent);
    if (dist < rampDistance) {
      double power = maxPower * (dist / rampDistance);
      power = Math.max(power, 0.05);
      setMotorPower(power, power);
    }
    else if (dist < (targetDist - rampDistance)) {
      double power = maxPower;
      setMotorPower(power, power);
    }
    else if (dist < targetDist) {
      double power = maxPower * ((targetDist - dist) / rampDistance);
      power = Math.max(power, 0.05);
      setMotorPower(power, power);
    }
    else {
      stopMotor();
      fillColor(0, 0, 0);
      return true;
    }
    return false;

  }
  /**
   * Sets power levels to drive robot.
   * 
   * @param left Power for left side (positive is forward)
   * @param right Power for right side (positive is forward)
   */
  private void setMotorPower(double left, double right) {
    left = left * getNumber("leftCorrect", 0.95);
    leftMotor1.set(-left);
    leftMotor2.set(-left);
    rightMotor1.set(right);
    rightMotor2.set(right);
  }

  private void setMotorPowerLights(double left, double right) {
    setMotorPower(left, right);
    double percent = (left + right) / 2;
    double turnPercent = (Math.abs(left) + Math.abs(right)) / 2;
    boolean isTurn = Math.abs(left - right) > 0.2;
    if (percent > 0) {
      if (isTurn) {
        fillPercent(40, 10, 30, turnPercent);
      }
      else {
        fillPercent(0, 255, 0, percent);
      }
    }
    else {
      if (isTurn) {
        fillPercent(40, 10, 30, turnPercent);
      }
      else {
        fillPercent(255, 0, 0, -percent);
      }
    }
    
  }
  private void stopMotor() {
    setMotorPower(0, 0);
  }

  private void zero() {
    leftEnc.setPosition(0);
    rightEnc.setPosition(0);
    direction.reset();
  }

  private void zeroDistance() {
    leftEnc.setPosition(0);
    rightEnc.setPosition(0);
    
  }

  private double getRightDistance() {
    return rightEnc.getPosition() * positionToIn;
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    zero();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (driveMode.getSelected().equals(tankDrive)) {
      tankPeriodic();
    }
    else {
      arcadePeriodic();
    }
  }
  private void arcadePeriodic() {
    double multiplier = 0.3;
    double fasterTrigger = controller.getRawAxis(3);
    double slowerTrigger = controller.getRawAxis(2);
    boolean toggleBackward = controller.getRawButton(5);

    if (fasterTrigger>=0.5) {
      multiplier = 0.55;
    }
    else if (slowerTrigger>=0.5) {
      multiplier = 0.15;
    }
    else {
      multiplier = 0.2;
    }

    double turnMultiplier = multiplier;
    if (toggleBackward) {
      multiplier = multiplier * -1;
    }
    
    double turn = checkCloseTo0(controller.getRawAxis(4)) * turnMultiplier;
    double straight = checkCloseTo0(-controller.getRawAxis(1)) * multiplier;
    
    setMotorPowerLights(-1 * straight - turn, straight - turn);
    SmartDashboard.putNumber("Straight", straight);
    SmartDashboard.putNumber("Turn", turn);
  }

  private double checkCloseTo0(double check) {
    if (check <= 0.05 && check >= -0.05) {
      check = 0;
    }
    return check;
  }

  public void tankPeriodic() {
    double multiplier = 0.35;
    double fasterTrigger = controller.getRawAxis(3);
    double slowerTrigger = controller.getRawAxis(2);
    boolean toggleBackward = controller.getRawButton(5);

    if (fasterTrigger>=0.5) {
      multiplier = 0.85;
    }
    else if (slowerTrigger>=0.5) {
      multiplier = 0.25;
    }
    else {
      multiplier = 0.45;
    }

    if (toggleBackward) {
      multiplier = multiplier * -1;
    }
    
    double rightPower = controller.getRawAxis(5)*multiplier;
    double leftPower = -controller.getRawAxis(1)*multiplier;
    if (controller.getRawButton(1)) {
      rightPower = leftPower;
    }
    setMotorPowerLights(leftPower, rightPower);
    SmartDashboard.putNumber("Left power", leftPower);
    SmartDashboard.putNumber("Right power", rightPower);
  }


  public static double getNumber(String label, double defaultVal) {
    Double val;
    if (SmartDashboard.containsKey(label)) {
      val = SmartDashboard.getNumber(label, defaultVal);
    } else {
      val = defaultVal;
      SmartDashboard.putNumber(label, val);
    }
    return val;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    lightTimer.reset();
    lightTimer.start();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    if (lightTimer.hasElapsed(2)) {
      lightTimer.reset();
      lightTimer.start();
    }
    else if (lightTimer.hasElapsed(1)) {
      fillColor(90, 90, 90);
    }
    else {
      fillColor(12, 37, 125);
    }
    

    
    
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}