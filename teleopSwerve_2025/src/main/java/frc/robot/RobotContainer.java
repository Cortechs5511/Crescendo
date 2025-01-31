// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.setClimberPower;
import frc.robot.commands.setFeederPower;
import frc.robot.commands.setLEDAlliance;
import frc.robot.commands.setSwerveState;
import frc.robot.commands.leftSideSpeakerAuto;
import frc.robot.commands.rightSideSpeakerAuto;
import frc.robot.commands.swerveDrive;
import frc.robot.commands.intake.setIntakePower;
import frc.robot.commands.intake.setWristPower;
import frc.robot.commands.shooter.setShooterPower;
import frc.robot.commands.shooter.setSpeakerPower;
import frc.robot.commands.setLEDAlliance;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SendableChooser<Command> autoChooser;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drive = new SwerveSubsystem();
  private final Wrist wrist = new Wrist();
  private final Intake intake = new Intake();
  private final Feeder feeder = new Feeder();
  private final LEDs blinkin = new LEDs();
  // private final Climber climber = new Climber();

  private final OI oi = OI.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController operatorController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
    // Configure the trigger bindings
    drive.setDefaultCommand(new swerveDrive(drive));
    blinkin.setDefaultCommand(new setLEDAlliance(blinkin));
    wrist.setDefaultCommand(new setWristPower(wrist));
    intake.setDefaultCommand(new setIntakePower(intake, blinkin));
    feeder.setDefaultCommand(new setFeederPower(feeder));
    // climber.setDefaultCommand(new setClimberPower(climber));
    configureBindings();

    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    autoChooser.addOption("Set State", new SequentialCommandGroup(new setSwerveState(drive, 0, -5, 0)));
    autoChooser.addOption("Left Side Speaker", new SequentialCommandGroup(new leftSideSpeakerAuto(drive, wrist, intake, feeder)));
    autoChooser.addOption("Right Side Speaker", new SequentialCommandGroup(new rightSideSpeakerAuto(drive, wrist, intake, feeder)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    SmartDashboard.putData("Taxi", new PathPlannerAuto("Taxi Auto"));
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    operatorController.rightTrigger().whileTrue(new setShooterPower(intake, feeder, blinkin));
    operatorController.rightBumper().whileTrue(new setSpeakerPower(intake, feeder, wrist, 0.632, 0.626)); 
    operatorController.leftBumper().whileTrue(new setSpeakerPower(intake, feeder, wrist, 0.626, 0.620)); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
