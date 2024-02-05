package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;

// no absolute encoder yet
class SwerveModule {
    // define all variables
    private CANSparkMax driveMotor;
    private CANSparkMax steeringMotor;
    private CoreCANcoder absoluteEncoder;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder steeringEncoder;

    /**
     * Creates an encoder object from NEO with preferred settings
     *
     * @param controller CANSparkMax controller
     * @return RelativeEncoder
     */
    private RelativeEncoder createEncoder(CANSparkMax controller) {
        // should be able to be used for both driving and turning encoder
        // driving relative encoder only cares about velocity
        // turning relative encoder only cares about position
        RelativeEncoder encoder = controller.getEncoder();

        // convert from native unit of rotation to radians
        encoder.setPositionConversionFactor(2 * Math.PI / 12.8);

        // convert from native unit of rpm to m/s
        encoder.setVelocityConversionFactor(Math.PI / 15 / 39.3701);
        encoder.setPosition(absoluteEncoder.getPosition().getValueAsDouble()-0.5);

        return encoder;
    }

    public static double fmod(double a, double b) {
        int result = (int) Math.floor(a / b);
        return a - result * b;
    }

    public static double referenceRadianAngle(double angle) {
        return fmod(angle, 2 * Math.PI);
    }
    
    
    private SwerveModuleState currentState;

    // 2 pid controllers, one for speed one for angle
    PIDController drivePIDController;
    PIDController steeringPIDController;


    public SwerveModule(
        int driveMotorPort,
        int steeringMotorPort,
        int absoluteEncoderPort
    ) {
        System.out.println("Swerve module constructor");
        driveMotor = new CANSparkMax(driveMotorPort, MotorType.kBrushless);
        steeringMotor = new CANSparkMax(steeringMotorPort, MotorType.kBrushless);
        absoluteEncoder = new CoreCANcoder(absoluteEncoderPort);
        
        driveEncoder = createEncoder(driveMotor);
        steeringEncoder = createEncoder(steeringMotor);
        
    
        // initialize pid controllers
        // need to tune p values
        drivePIDController = new PIDController(0.5, 0, 0);
        steeringPIDController = new PIDController(0.1, 0, 0);
        steeringPIDController.enableContinuousInput(-90, 90);
        
        currentState = new SwerveModuleState();
    }

    public SwerveModuleState getState() {
        return currentState;
    }

    public Rotation2d getAngle() {
        return new Rotation2d(referenceRadianAngle(steeringEncoder.getPosition()));
    }

    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoder() {
        return absoluteEncoder.getPosition().getValueAsDouble();
    }

    public void setDesiredState(SwerveModuleState newState) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        // NEED TO CHANGE POSITION TO RADIANS? USING CONVERSION FACTOR (Resolved)
        currentState = newState;

        SwerveModuleState state = SwerveModuleState.optimize(newState, new Rotation2d(referenceRadianAngle(steeringEncoder.getPosition())));

        // Calculate the drive output from the drive PID controller
        // NEED TO CHANGE RPM TO M/S USING CONVERSION FACTOR (Resolved)
        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        // NEED TO CHANGE POSITION TO RADIANS? USING CONVERSION FACTOR (Resolved)
        final var turnOutput = steeringPIDController.calculate(referenceRadianAngle(steeringEncoder.getPosition()), state.angle.getRadians());

        driveMotor.set(driveOutput*0);
        steeringMotor.set(turnOutput);

    }

    // public SwerveModuleState zeroState(){
    //     double absoluteEncoderPos = Math.PI - referenceRadianAngle(absoluteEncoder.getPosition().getValueAsDouble());
    //     return new SwerveModuleState(0.0, new Rotation2d(absoluteEncoderPos));

    // }

    public void periodic() {
        
    }
}


public class SwerveSubsystem extends SubsystemBase{
    
    // list of 4 swerve modules
    SwerveModule frontLeftModule = new SwerveModule(30, 40, 1);
    SwerveModule frontRightModule = new SwerveModule(21, 20, 2);
    SwerveModule backLeftModule = new SwerveModule(51, 61, 3);
    SwerveModule backRightModule = new SwerveModule(10, 11, 4);
    
    
    double chassisWidth = Units.inchesToMeters(30);
    double chassisLength = Units.inchesToMeters(30);

    // Defining the location of the wheels relative to the center of the robot
    Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
    Translation2d frontRightLocation = new Translation2d(chassisLength / 2, -chassisWidth / 2);
    Translation2d backLeftLocation = new Translation2d(-chassisLength / 2, chassisWidth / 2);
    Translation2d backRightLocation = new Translation2d(-chassisLength / 2, -chassisWidth / 2);

    // define kinematics object (takes chassisspeeds and returns swervemodulestates)
    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation    
        );


    // get a reference to the controller
    CommandXboxController controller;
    
        // Constructor
    public SwerveSubsystem(CommandXboxController io) {
        controller = io;
    }

    public void setChassisSpeed(ChassisSpeeds desired) {

        // Get the desired states of the wheels
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desired);


        // set state of each module (speed/direction)
        frontLeftModule.setDesiredState(newStates[0]);
        frontRightModule.setDesiredState(newStates[1]);
        backLeftModule.setDesiredState(newStates[2]);
        backRightModule.setDesiredState(newStates[3]);

        
    }

    @Override
    public void periodic() {
        
        // read data from the controller
        ChassisSpeeds newDesiredSpeeds = new ChassisSpeeds(
            // pushing forward
            -controller.getRawAxis(1),
            // pushing left
            -controller.getRawAxis(0),
            // pushing left will rotate
            -controller.getRawAxis(4)
        );
        
        setChassisSpeed(newDesiredSpeeds);

        SmartDashboard.putNumber("Angle", frontLeftModule.getAngle().getDegrees());
        SmartDashboard.putNumber("Velocity", frontLeftModule.getVelocity());
        
        // send data to smartdashboard
        double[] loggingState = {
            frontLeftModule.getAngle().getRadians(),
            frontLeftModule.getVelocity(),
            frontRightModule.getAngle().getRadians(),
            frontRightModule.getVelocity(),
            backLeftModule.getAngle().getRadians(),
            backLeftModule.getVelocity(),
            backRightModule.getAngle().getRadians(),
            backRightModule.getVelocity(),
        };
        double controlerState[] = {
            frontLeftModule.getState().angle.getRadians(),
            frontLeftModule.getState().speedMetersPerSecond,
            frontRightModule.getState().angle.getRadians(),
            frontRightModule.getState().speedMetersPerSecond,
            backLeftModule.getState().angle.getRadians(),
            backLeftModule.getState().speedMetersPerSecond,
            backRightModule.getState().angle.getRadians(),
            backRightModule.getState().speedMetersPerSecond,
        };
        
        SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
        SmartDashboard.putNumberArray("ControllerStates", controlerState);
    }
}
