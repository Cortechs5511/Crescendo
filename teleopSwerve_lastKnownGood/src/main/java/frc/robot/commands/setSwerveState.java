package frc.robot.commands;

import frc.robot.Constants.SwerveConstants;
import frc.robot.OI;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class setSwerveState extends Command{
    private final SwerveSubsystem swerve;
    private final OI oi = OI.getInstance();

    public setSwerveState(SwerveSubsystem subsystem) {
        swerve = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            0, 
            -5,
            0
        );

        swerve.driveRobotRelative(newDesiredSpeeds);
        
    }
}
