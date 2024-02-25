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
    public static final int kDriverControllerPort = 3;
  }

  public static class IntakeConstants {
    public static final int WRIST_ID = 00;
    public static final int TOP_WHEEL_ID = 00;
    public static final int BOTTOM_WHEEL_ID = 00;
    public static final int THROUGH_BORE_ID = 00;

    public static final double WRIST_VOLTAGE_COMPENSATION = 10;
    
    public static final int WRIST_CURRENT_LIMIT = 50;
    public static final int INTAKE_CURRENT_LIMIT = 50;

    public static final IdleMode WRIST_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode INTAKE_IDLE_MODE = IdleMode.kCoast;

    public static final double RAMP_RATE = 0.05;

    public static final double WRIST_MAX_POWER = 1.0;
    public static final double INTAKE_MAX_POWER = 1.0;

    // 36:1 ratio
    public static final double WRIST_CONVERSION_FACTOR = 36;
    // position of intake when up
    public static final double WRIST_MAX_POS = 0.25;
    // position of intake when down (intaking)
    public static final double WRIST_MIN_POS = 0;
    
  }

  public static class OIConstants {
    
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 0;

    public static final double DEADBAND = 0.05;

  }



}
