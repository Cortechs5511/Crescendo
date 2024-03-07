package frc.robot.subsystems;

import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
    
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private Gyro gyro;
    private SwerveDriveOdometry odometry;

    private Field2d field;

    public SwerveSubsystem() {
        gyro = new Gyro();
        modules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.MOTOR_IDS[0], SwerveConstants.MOTOR_IDS[1], SwerveConstants.MOTOR_IDS[2]),
            new SwerveModule(SwerveConstants.MOTOR_IDS[3], SwerveConstants.MOTOR_IDS[4], SwerveConstants.MOTOR_IDS[5]),
            new SwerveModule(SwerveConstants.MOTOR_IDS[6], SwerveConstants.MOTOR_IDS[7], SwerveConstants.MOTOR_IDS[8]),
            new SwerveModule(SwerveConstants.MOTOR_IDS[9], SwerveConstants.MOTOR_IDS[10], SwerveConstants.MOTOR_IDS[11]),
        };
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[0], SwerveConstants.MODULE_TRANSLATIONS[1]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[2], SwerveConstants.MODULE_TRANSLATIONS[3]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[4], SwerveConstants.MODULE_TRANSLATIONS[5]),
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[6], SwerveConstants.MODULE_TRANSLATIONS[7])
        );

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
        field = new Field2d();
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getPositions());
        field.setRobotPose(getPose());
        logState();
    }

    public void drive(double y, double x, double theta, boolean fieldRelative) {
        odometry.update(gyro.getRotation2d(), getPositions());

        field.setRobotPose(getPose());

        ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_SPEED * y, 
            SwerveConstants.MAX_SPEED * x,
            SwerveConstants.MAX_SPEED * theta
        );

        if (fieldRelative) {
            driveFieldRelative(newDesiredSpeeds);
        }
        else {
            driveRobotRelative(newDesiredSpeeds);
        }
        
    }

    public void logState() {
        double[] loggingState = {
            modules[0].getAngle().getRadians(),
            modules[0].getVelocity(),
            modules[1].getAngle().getRadians(),
            modules[1].getVelocity(),
            modules[2].getAngle().getRadians(),
            modules[2].getVelocity(),
            modules[3].getAngle().getRadians(),
            modules[3].getVelocity(),
        };
        
        SmartDashboard.putNumberArray("Module State", loggingState);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        // add constant for max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.MAX_SPEED);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(targetStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
          }
          return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        return positions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }
    
    // Swerve Module Class
    class SwerveModule {
        private SwerveModulePosition currentPosition;
        private SwerveModuleState currentState;
        private CANSparkMax driveMotor;
        private CANSparkMax turnMotor;
        private CoreCANcoder absoluteEncoder;

        private RelativeEncoder driveEncoder;
        private RelativeEncoder turnEncoder;

        PIDController drivePIDController;
        PIDController turnPIDController;
        
        public SwerveModule(int driveMotorPort, int steeringMotorPort, int absoluteEncoderPort) {

            driveMotor = createMotorController(driveMotorPort, false);
            turnMotor = createMotorController(steeringMotorPort, false);
            absoluteEncoder = new CoreCANcoder(absoluteEncoderPort);
            
            driveEncoder = createEncoder(driveMotor);
            turnEncoder = createEncoder(turnMotor);
            
            drivePIDController = new PIDController(SwerveConstants.DRIVE_PID_VALUES[0], SwerveConstants.DRIVE_PID_VALUES[1], SwerveConstants.DRIVE_PID_VALUES[2]);
            turnPIDController = new PIDController(SwerveConstants.TURN_PID_VALUES[0], SwerveConstants.TURN_PID_VALUES[1], SwerveConstants.TURN_PID_VALUES[2]);
            turnPIDController.enableContinuousInput(-Math.PI/2, Math.PI/2);
            
            currentState = new SwerveModuleState();
            currentPosition = new SwerveModulePosition();
        }

        private CANSparkMax createMotorController(int port, boolean isInverted) {
            CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
            controller.restoreFactoryDefaults();
    
            controller.enableVoltageCompensation(SwerveConstants.VOLTAGE_COMPENSATION);
            controller.setIdleMode(IdleMode.kCoast);
            controller.setOpenLoopRampRate(SwerveConstants.RAMP_RATE);
            controller.setClosedLoopRampRate(SwerveConstants.RAMP_RATE); 
    
            controller.setSmartCurrentLimit(SwerveConstants.CURRENT_LIMIT);
    
            controller.setInverted(isInverted);
    
            // for some reason causes robot to shake:
            //     controller.burnFlash(); 
            return controller;
        }

        private RelativeEncoder createEncoder(CANSparkMax controller) {
            // should be able to be used for both driving and turning encoder
            // driving relative encoder only cares about velocity
            // turning relative encoder only cares about position
            RelativeEncoder encoder = controller.getEncoder();
    

            // convert from native unit of rpm to m/s
            encoder.setVelocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);
    
            return encoder;
        }

        public static double fmod(double a, double b) {
            int result = (int) Math.floor(a / b);
            return a - result * b;
        }
    
        public static double referenceRadianAngle(double angle) {
            return fmod(angle, 2 * Math.PI);
        }

        public double getVelocity() {
            return driveEncoder.getVelocity();
        }

        public double getAbsoluteEncoderPos() {
            return absoluteEncoder.getAbsolutePosition().getValueAsDouble()-0.5;
        }

        public Rotation2d getAngle() {
            return new Rotation2d(getAbsoluteEncoderPos());
        }

        public void setTargetState(SwerveModuleState targetState) {
            currentState = SwerveModuleState.optimize(targetState, currentState.angle);
            currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);

            final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), currentState.speedMetersPerSecond);

            final var turnOutput = turnPIDController.calculate(getAbsoluteEncoderPos()*2*Math.PI, currentState.angle.getRadians());

            driveMotor.set(driveOutput);
            turnMotor.set(turnOutput);
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(getVelocity(), getAngle());
        }

        public SwerveModulePosition getPosition() {
            return currentPosition;
        }
    }

    // Gyro Class
    class Gyro {
        private AHRS navx = new AHRS(Port.kMXP);

        public Rotation2d getRotation2d() {
            return navx.getRotation2d();
        }
    }

}
