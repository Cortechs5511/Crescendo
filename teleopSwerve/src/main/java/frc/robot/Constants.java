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

  public static class IntakeConstants {
    
    public static final int TOP_WHEEL_ID = 2;
    public static final int BOTTOM_WHEEL_ID = 1;
    
    public static final int FEEDER_ID = 5;

    
    public static final int INTAKE_CURRENT_LIMIT = 50;
    public static final double VOLTAGE_COMPENSATION = 10;
    public static final IdleMode INTAKE_IDLE_MODE = IdleMode.kCoast;
    public static final double RAMP_RATE = 0.05;

    public static final double INTAKE_POWER = 0.5;
    public static final double FEEDER_POWER = 0.8;

  
    
  }

  public static class WristConstants {
    public static final int WRIST_ID = 3;
    public static final int THROUGH_BORE_ID = 9;

    public static final double VOLTAGE_COMPENSATION = 10;
    public static final int CURRENT_LIMIT = 50;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final double RAMP_RATE = 0.05;
    public static final double MAX_POWER = 0.5;

    // 50:1 ratio
    public static final double WRIST_CONVERSION_FACTOR = 50;
    // position of intake when up
    public static final double MAX_POS = 0.25;
    // position of intake when down (intaking)
    public static final double MIN_POS = 0;

  }

  public static class LEDConstants {
    public static final int BLINKIN_CHANNEL = 0;
  }

  public static class OIConstants {
    
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 0;

    public static final double DEADBAND = 0.05;

  }



}
