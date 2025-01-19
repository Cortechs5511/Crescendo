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
    private double vx;
    private double vy;
    private double omega;

    public setSwerveState(SwerveSubsystem subsystem, double velocityX, double velocityY, double angleVelocity) {
        swerve = subsystem;
        vx = velocityX;
        vy = velocityY;
        omega = angleVelocity;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            vx, 
            vy,
            omega
        );

        swerve.driveRobotRelative(newDesiredSpeeds);
        
    }
}
