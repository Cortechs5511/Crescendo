package frc.robot.subsystems;

import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase{
    
    private SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private final customPIDController[] drivePIDControllers;
    private final customPIDController[] turnPIDControllers;
    private Gyro gyro;
    private SwerveDriveOdometry odometry;

    private Field2d field;

    private XboxController driveController;

    public SwerveSubsystem() {
        gyro = new Gyro();
        modules = new SwerveModule[] {
            new SwerveModule(SwerveConstants.IDS[0], SwerveConstants.IDS[1], SwerveConstants.IDS[2]),
            new SwerveModule(SwerveConstants.IDS[3], SwerveConstants.IDS[4], SwerveConstants.IDS[5]),
            new SwerveModule(SwerveConstants.IDS[6], SwerveConstants.IDS[7], SwerveConstants.IDS[8]),
            new SwerveModule(SwerveConstants.IDS[9], SwerveConstants.IDS[10], SwerveConstants.IDS[11]),
        };
        kinematics = new SwerveDriveKinematics(
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[0], SwerveConstants.MODULE_TRANSLATIONS[1]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[2], SwerveConstants.MODULE_TRANSLATIONS[3]), 
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[4], SwerveConstants.MODULE_TRANSLATIONS[5]),
            new Translation2d(SwerveConstants.MODULE_TRANSLATIONS[6], SwerveConstants.MODULE_TRANSLATIONS[7])
        );
        drivePIDControllers = new customPIDController[] {
            new customPIDController(SwerveConstants.FL_DRIVE_PID_VALUES[0], SwerveConstants.FL_DRIVE_PID_VALUES[1], SwerveConstants.FL_DRIVE_PID_VALUES[2]),
            new customPIDController(SwerveConstants.FR_DRIVE_PID_VALUES[0], SwerveConstants.FR_DRIVE_PID_VALUES[1], SwerveConstants.FR_DRIVE_PID_VALUES[2]),
            new customPIDController(SwerveConstants.BL_DRIVE_PID_VALUES[0], SwerveConstants.BL_DRIVE_PID_VALUES[1], SwerveConstants.BL_DRIVE_PID_VALUES[2]),
            new customPIDController(SwerveConstants.BR_DRIVE_PID_VALUES[0], SwerveConstants.BR_DRIVE_PID_VALUES[1], SwerveConstants.BR_DRIVE_PID_VALUES[2])  
        };
        turnPIDControllers = new customPIDController[] {
            new customPIDController(SwerveConstants.FL_TURN_PID_VALUES[0], SwerveConstants.FL_TURN_PID_VALUES[1], SwerveConstants.FL_TURN_PID_VALUES[2]),
            new customPIDController(SwerveConstants.FR_TURN_PID_VALUES[0], SwerveConstants.FR_TURN_PID_VALUES[1], SwerveConstants.FR_TURN_PID_VALUES[2]),
            new customPIDController(SwerveConstants.BL_TURN_PID_VALUES[0], SwerveConstants.BL_TURN_PID_VALUES[1], SwerveConstants.BL_TURN_PID_VALUES[2]),
            new customPIDController(SwerveConstants.BR_TURN_PID_VALUES[0], SwerveConstants.BR_TURN_PID_VALUES[1], SwerveConstants.BR_TURN_PID_VALUES[2])  
        };
        // drivePIDControllers[1].enableContinuousInput(-SwerveConstants.PID_RANGE, SwerveConstants.PID_RANGE);
        // drivePIDControllers[3].enableContinuousInput(-SwerveConstants.PID_RANGE, SwerveConstants.PID_RANGE);

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());
        field = new Field2d();

        driveController = new XboxController(0);

        /*
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, this::driveRobotRelative, 
            SwerveConstants.pathFollowerConfig, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);    // Configure AutoBuilder last
            */


        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig robotConfig = null;
        try{
          robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                   new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                   new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                robotConfig, // The robot configuration
                () -> {
                     // Boolean supplier that controls when the path will be mirrored for the red alliance
                     // This will flip the path being followed to the red side of the field.
                     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                      var alliance = DriverStation.getAlliance();
                      if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );


            // PathPlannerLogging.setLogActivePathCallback(poses); -> field.getObject("path").setPoses(poses);

            SmartDashboard.putData("field", field);
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getPositions());
        field.setRobotPose(getPose());
        logStates();
        ChassisSpeeds controllerSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * -driveController.getLeftY(), 
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * -driveController.getLeftX(), 
            SwerveConstants.MAX_ROTATIONAL_SPEED * -driveController.getRightX()
        );
        SwerveModuleState[] controllerStates = kinematics.toSwerveModuleStates(controllerSpeeds);
        double[] controllerStatesAsDoubles = {
            controllerStates[0].angle.getRadians(),
            controllerStates[0].speedMetersPerSecond,
            controllerStates[1].angle.getRadians(),
            controllerStates[1].speedMetersPerSecond,
            controllerStates[2].angle.getRadians(),
            controllerStates[2].speedMetersPerSecond,
            controllerStates[3].angle.getRadians(),
            controllerStates[3].speedMetersPerSecond,
        };
        SmartDashboard.putNumberArray("Controller State", controllerStatesAsDoubles);
    }

    public void drive(double y, double x, double theta, boolean fieldRelative, boolean reverse, boolean resetPID, boolean resetGyro) {
        odometry.update(gyro.getRotation2d(), getPositions());

        field.setRobotPose(getPose());

        ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * y, 
            SwerveConstants.MAX_TRANSLATIONAL_SPEED * x,
            SwerveConstants.MAX_ROTATIONAL_SPEED * theta
        );
        if (reverse) {
            newDesiredSpeeds = new ChassisSpeeds(
                0, 
                -5,
                0
            );
        }

        if (resetPID) {
            for (int i = 0; i < 4; i++) {
                turnPIDControllers[i].reset();
            }
        }

        if (resetGyro) {
            gyro.resetGyro();
        }

        if (fieldRelative) {
            driveFieldRelative(newDesiredSpeeds);
        }
        else {
            driveRobotRelative(newDesiredSpeeds);
        }
        
    }

    public void logStates() {
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
        SmartDashboard.putNumber("Gyro Radians", gyro.getRotation2d().getRadians());

        SmartDashboard.putNumber("Swerve/Turn Velocity", modules[0].getTurnSpeed());

        SmartDashboard.putNumber("Swerve/FL Velocity", modules[0].getVelocity());
        SmartDashboard.putNumber("Swerve/FR Velocity", modules[1].getVelocity());
        SmartDashboard.putNumber("Swerve/BL Velocity", modules[2].getVelocity());
        SmartDashboard.putNumber("Swerve/BR Velocity", modules[3].getVelocity());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, SwerveConstants.MAX_TRANSLATIONAL_SPEED);

       modules[0].setTargetState(targetStates[0], drivePIDControllers[0], turnPIDControllers[0]);
       modules[1].setTargetState(targetStates[1], drivePIDControllers[1], turnPIDControllers[1]);
       modules[2].setTargetState(targetStates[2], drivePIDControllers[2], turnPIDControllers[2]);
       modules[3].setTargetState(targetStates[3], drivePIDControllers[3], turnPIDControllers[3]);
    //    for (int i = 0; i<4; i++) {
    //     modules[i].setTargetState(targetStates[i], drivePIDControllers[i], turnPIDControllers[i]);
    //    }
    
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
        private SparkMax driveMotor;
        private SparkMax turnMotor;
        private CoreCANcoder absoluteEncoder;

        private RelativeEncoder driveEncoder;
        private RelativeEncoder turnEncoder;
        
        public SwerveModule(int driveMotorPort, int steeringMotorPort, int absoluteEncoderPort) {

            driveMotor = createMotorController(driveMotorPort, false);
            turnMotor = createMotorController(steeringMotorPort, false);
            absoluteEncoder = new CoreCANcoder(absoluteEncoderPort);
            
            driveEncoder = createEncoder(driveMotor);
            turnEncoder = createEncoder(turnMotor);
            
            currentState = new SwerveModuleState(getVelocity(), getAngle());
            currentPosition = new SwerveModulePosition();
        }

        private SparkMax createMotorController(int port, boolean isInverted) {
            SparkMax controller = new SparkMax(port, MotorType.kBrushless);
            SparkMaxConfig config = new SparkMaxConfig();
    
            config.voltageCompensation(SwerveConstants.VOLTAGE_COMPENSATION);
            config.idleMode(IdleMode.kCoast);
            config.openLoopRampRate(SwerveConstants.RAMP_RATE);
            config.closedLoopRampRate(SwerveConstants.RAMP_RATE); 
    
            config.smartCurrentLimit(SwerveConstants.CURRENT_LIMIT);
    
            config.inverted(isInverted);
    
            config.encoder.velocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);
            // for some reason causes robot to shake:
            //     controller.burnFlash(); 
            controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            return controller;
        }

        private RelativeEncoder createEncoder(SparkMax controller) {
            RelativeEncoder encoder = controller.getEncoder();
            // convert from native unit of rpm to m/s
            //encoder.setVelocityConversionFactor(SwerveConstants.VELOCITY_CONVERSION_FACTOR);
    
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
            return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
        }

        public Rotation2d getAngle() {
            return Rotation2d.fromRotations(getAbsoluteEncoderPos());
        }

        public void setTargetState(SwerveModuleState targetState, customPIDController drivePID, customPIDController turnPID) {
            Rotation2d existingAngle = getAngle();
            currentState = SwerveModuleState.optimize(targetState, getAngle());
            currentPosition = new SwerveModulePosition(currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
            double turnOutput;
            final double driveOutput = drivePID.calculate(driveEncoder.getVelocity(), currentState.speedMetersPerSecond);

            if (Math.abs(existingAngle.getRadians() - currentState.angle.getRadians()) < Math.PI / 2) {
                turnOutput = turnPID.calculate(existingAngle.getRadians(), currentState.angle.getRadians());
            }
            else {
                turnOutput = turnPID.calculate(-existingAngle.getRadians(), currentState.angle.getRadians());
            }
            // turnOutput = turnPID.calculate(existingAngle.getRadians(), currentState.angle.getRadians());
    

            turnMotor.set(turnOutput);
            driveMotor.set(driveOutput);
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(getVelocity(), getAngle());
        }

        public SwerveModulePosition getPosition() {
            return currentPosition;
        }
        public double getTurnSpeed() {
            return turnMotor.get();
        }
    }

    // Gyro Class
    class Gyro {
        private AHRS navx = new AHRS(NavXComType.kMXP_SPI);  

        // public Rotation2d getRotation2d() {
            // Rotation3d rot3d = getCurrentRotation3d();
            // Rotation2d rot2d = new Rotation2d(rot3d.getX(), rot3d.getY());
            // return rot2d;
        // }
        
        public Rotation2d getRotation2d() {
            // Get yaw in degrees from the navX
            double yawDegrees = navx.getYaw(); 
            
            // Convert degrees to radians (Rotation2d uses radians)
            double yawRadians = Math.toRadians(yawDegrees);
            
            // Create and return a Rotation2d object
            return new Rotation2d(yawRadians);
        }

        public Rotation3d getCurrentRotation3d() {
            // Get yaw, pitch, and roll from the navX
            double yawDegrees = navx.getYaw();   // Yaw in degrees
            double pitchDegrees = navx.getPitch(); // Pitch in degrees
            double rollDegrees = navx.getRoll();  // Roll in degrees
    
            // Convert degrees to radians (Rotation3d uses radians)
            double yawRadians = Math.toRadians(yawDegrees);
            double pitchRadians = Math.toRadians(pitchDegrees);
            double rollRadians = Math.toRadians(rollDegrees);
    
            // Create and return a Rotation3d object
            return new Rotation3d(rollRadians, pitchRadians, yawRadians);
        }

        public void resetGyro() {
            navx.reset();
        }
    }

}
