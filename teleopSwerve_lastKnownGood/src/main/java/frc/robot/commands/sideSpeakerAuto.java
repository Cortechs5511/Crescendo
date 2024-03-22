package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.shooter.setSpeakerPower;
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

public class sideSpeakerAuto extends Command{
    private final SwerveSubsystem swerve;
    private final Wrist wrist;
    private final Intake intake;
    private final Feeder feeder;
    private final Timer timer = new Timer();
    private final Optional<Alliance> ally = DriverStation.getAlliance();
    private double translationPower;

    public sideSpeakerAuto(SwerveSubsystem driveSubsystem, Wrist wristSubsystem, Intake intakeSubsystem, Feeder feederSubsystem) {
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
        
    }

    @Override
    public void execute() {
        
        // 0-3 second move robot
        if (!timer.hasElapsed(3)) {
            wrist.setSpeakerPosition(0.623);
        }
        else if( timer.hasElapsed(5)) {
            feeder.setPower(0);
            intake.setBottomWheels(0);
            intake.setTopWheels(0);
            ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            -5, 
            5,
            0
        );
            swerve.driveRobotRelative(newDesiredSpeeds);

        }
        else if( timer.hasElapsed(4)) {
            feeder.setPower(-1);
        }
        // 4+ second feeder power and stop wrist 
        else if (timer.hasElapsed(3)) {
            wrist.setPower(0);
            intake.setTopWheels(IntakeConstants.SPEAKER_POWER * 15 / 16);
            intake.setBottomWheels(IntakeConstants.SPEAKER_POWER);
        }

        
        else {
            // feeder.setPower(0);
            // intake.setPower(0);
            // wrist.setPower(0);
            // ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            // -translationPower, 
            // 5,
            // 0);
            // swerve.driveRobotRelative(newDesiredSpeeds);
            

        }
        
        
        
    }
}
