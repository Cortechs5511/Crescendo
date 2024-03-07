// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int OPERATOR_PORT = 3;
  }

  public static class SwerveConstants {
    public static final int[] MOTOR_IDS = {10, 11, 1, 20, 21, 2, 30, 31, 3, 40, 41, 4};
    
    public static final double CHASSIS_LENGTH = Units.inchesToMeters(23);
    public static final double CHASSIS_WIDTH = Units.inchesToMeters(23);
    public static final double[] MODULE_TRANSLATIONS = {
      CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2, 
      CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, CHASSIS_WIDTH / 2,
      -CHASSIS_LENGTH / 2, -CHASSIS_WIDTH / 2,
    };

    public static final double WHEEL_DIAMETER_IN = 4;
    public static final double WHEEL_CIRCUMFERENCE_IN = WHEEL_DIAMETER_IN*Math.PI;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double INCHES_PER_METER = 39.3701;
    // convert native units of rpm to meters per second
    public static final double VELOCITY_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_IN / DRIVE_GEAR_RATIO / INCHES_PER_METER / 60;

    public static final double VOLTAGE_COMPENSATION = 10;
    public static final int CURRENT_LIMIT = 40;
    public static final double RAMP_RATE = 0.05;

    public static final double[] DRIVE_PID_VALUES = {0.3, 0.0, 0.0};
    public static final double[] TURN_PID_VALUES = {0.3, 0.0, 0.0};

    public static final double MAX_SPEED = 8;

    public static final double MAX_AUTON_SPEED = 4.5;
    public static final double DRIVE_BASE_RADIUS = Units.inchesToMeters(Math.sqrt(CHASSIS_LENGTH*CHASSIS_LENGTH+CHASSIS_WIDTH*CHASSIS_WIDTH));

  }

  public static class IntakeConstants {
    
    public static final int TOP_WHEELS_ID = 51;
    public static final int BOTTOM_WHEELS_ID = 50;  
    public static final int FEEDER_ID = 52;
  
    public static final int INTAKE_CURRENT_LIMIT = 40;
    public static final double VOLTAGE_COMPENSATION = 10;
    public static final IdleMode INTAKE_IDLE_MODE = IdleMode.kCoast;
    public static final double RAMP_RATE = 0.05;

    public static final double INTAKE_POWER = 0.4;
    public static final double FEEDER_INTAKE_POWER = 0.2;
    
    
    public static final double SHOOTER_POWER = -0.8;
    public static final double FEEDER_SHOOTER_POWER = -0.5;
  }

  public static class WristConstants {
    public static final int WRIST_L_ID = 60;
    public static final int WRIST_R_ID = 61;
    public static final int THROUGH_BORE_ID = 9;

    public static final double VOLTAGE_COMPENSATION = 10;
    public static final int CURRENT_LIMIT = 40;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final double RAMP_RATE = 0.05;
    public static final double MAX_POWER = 0.5;

    // 50:1 ratio
    public static final double POSITION_CONVERSION_FACTOR = 50 * 22 / 12;
    // position of intake when up
    public static final double MAX_POS = 0.25;
    // position of intake when down (intaking)
    public static final double MIN_POS = 0;
    // range of positions
    public static final double RANGE = MAX_POS-MIN_POS;

  }

  public static class ClimberConstants {
    public static final int CLIMBER_L_ID = 1;
    public static final int CLIMBER_R_ID = 1;

    public static final double VOLTAGE_COMPENSATION = 10;
    public static final int CURRENT_LIMIT = 40;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final double RAMP_RATE = 0.05;

    public static final double MAX_POWER = 0.7;
    public static final double CONVERSION_FACTOR = 1.0;

  }

  public static class LEDConstants {
    public static final int BLINKIN_CHANNEL = 0;

    public static final double RED_ALLIANCE = -0.93;
    public static final double BLUE_ALLIANCE = -0.95;

    public static final double INTAKE = -0.95;

    public static final double SHOOTER = -0.95;
    
  }

  public static class OIConstants {
    
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double DEADBAND = 0.05;

  }



}
