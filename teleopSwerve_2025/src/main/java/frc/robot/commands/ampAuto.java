package frc.robot.commands;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Wrist;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ampAuto extends Command{
    private final SwerveSubsystem swerve;
    private final Wrist wrist;
    private final Intake intake;
    private final Feeder feeder;
    private final Timer timer = new Timer();
    private final Optional<Alliance> ally = DriverStation.getAlliance();
    private double translationPower;

    public ampAuto(SwerveSubsystem driveSubsystem, Wrist wristSubsystem, Intake intakeSubsystem, Feeder feederSubsystem) {
        swerve = driveSubsystem;
        wrist = wristSubsystem;
        intake = intakeSubsystem;
        feeder = feederSubsystem;
        addRequirements(driveSubsystem);
        addRequirements(wristSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        timer.restart();
        // if (ally.isPresent()) {
        //     if (ally.get() == Alliance.Red) {
        //         translationPower = -0.5;
        //     }
        //     if (ally.get() == Alliance.Blue) {
        //         translationPower = 0.5;
        //     }
        // }
        translationPower = 0.5;
    }

    @Override
    public void execute() {
        
        // 0-3 second move robot
        if (!timer.hasElapsed(3)) {
            ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            translationPower, 
            5,
            0);
            swerve.driveRobotRelative(newDesiredSpeeds);
        }
        // 4+ second feeder power and stop wrist 
        else if (timer.hasElapsed(4)) {
            feeder.setPower(-0.9);
            wrist.setPower(0);
        }
        // 3-4 second stop robot lower wrist ramp up shooter
        else if (timer.hasElapsed(3)) {
            ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            0, 
            0,
            0);
            swerve.driveRobotRelative(newDesiredSpeeds);
            intake.setTopWheels(-0.27);
            intake.setBottomWheels(-0.27);
            wrist.setPower(-0.1);
        }
        else {
            feeder.setPower(0);
            intake.setPower(0);
            wrist.setPower(0);
            ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            -translationPower, 
            5,
            0);
            swerve.driveRobotRelative(newDesiredSpeeds);
        }
        
        
        
    }
}
