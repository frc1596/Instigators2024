package org.deceivers.swerve;


import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleV3 implements SwerveModule {

    private final CANSparkMax mAzimuthMotor;
    private final CANSparkMax mDriveMotor;


    private final CANcoder mAzimuthAbsoluteEncoder;


    private final RelativeEncoder mAzimuthIncrementalEncoder;
    private final RelativeEncoder mDriveEncoder;
    private final SparkPIDController mDrivePID;
    private final SparkPIDController mAzimuthPID;
    private final Translation2d mLocation;
    private final String mName;

      private PIDController azimuthPID = new PIDController(0.01, 0, 0);

    // need to update the speed to m/s

    public SwerveModuleV3(CANSparkMax azimuthMotor, CANSparkMax driveMotor,
            Translation2d location, String name, CANcoder azimuthEncoder) {
            
        mDriveMotor = driveMotor;
        mAzimuthMotor = azimuthMotor;
        mLocation = location;
        mName = name;

        // Rest motors to factory defaults to ensure correct parameters
        mDriveMotor.restoreFactoryDefaults();
        mAzimuthMotor.restoreFactoryDefaults();

        // Get encoders
        mAzimuthAbsoluteEncoder = azimuthEncoder;
        mAzimuthIncrementalEncoder = mAzimuthMotor.getEncoder();
        mDriveEncoder = mDriveMotor.getEncoder();

        // Get PIDs
        mDrivePID = mDriveMotor.getPIDController();
        mAzimuthPID = mAzimuthMotor.getPIDController();

        // Configure drive motor controller parameters
        mDriveMotor.setInverted(true);
        mDriveMotor.setClosedLoopRampRate(0);
        mDriveMotor.setOpenLoopRampRate(.1);
        mDriveMotor.setIdleMode(IdleMode.kBrake);

        //If rpm is lower than limitRPM, the stall limit will be used, otherwise the free limit will be used
        mDriveMotor.setSmartCurrentLimit(40, 60, 100); 

        // Configure Drive Encoder
        mDriveEncoder.setPositionConversionFactor(0.319024/6.12); //1/6.12
        mDriveEncoder.setVelocityConversionFactor(((0.319024/6.12) / 60));
        mDriveEncoder.setPosition(0);

        // Configure azimuth motor controller parameters
        mAzimuthMotor.setInverted(true);
        mAzimuthMotor.setClosedLoopRampRate(0);
        mAzimuthMotor.setOpenLoopRampRate(0);
        mAzimuthMotor.setIdleMode(IdleMode.kBrake);
        mAzimuthMotor.setSmartCurrentLimit(40);

        // Configure drive absolute encoder
       // mAzimuthAbsoluteEncoder.setPositionConversionFactor(1.0);
        //mAzimuthAbsoluteEncoder.setInverted(true);
        //mAzimuthAbsoluteEncoder.setAverageDepth(1);
        
        // Configure azimuth incremental encoder
        mAzimuthIncrementalEncoder.setPositionConversionFactor(150.0/7.0); //derek Maybe fix this
        // Configure drive PID
        mDrivePID.setFF(.3);
        mDrivePID.setP(.1);


        azimuthPID.enableContinuousInput(-180,180);
        // Configure azimuth PID
        //mAzimuthPID.setFeedbackDevice(mAzimuthAbsoluteEncoder);
        // mAzimuthPID.setFeedbackDevice(mAzimuthIncrementalEncoder);
        // mAzimuthPID.setP(.05);
        // mAzimuthPID.setPositionPIDWrappingEnabled(true);
        // mAzimuthPID.setPositionPIDWrappingMinInput(0);
        // mAzimuthPID.setPositionPIDWrappingMaxInput(360);


        // Burn flahs in case of power cycle
        mDriveMotor.burnFlash();
        mAzimuthMotor.burnFlash();
    }

    // Sets the drive motor speed in open loop mode
    public void setSpeed(double speed) {
        mDriveMotor.set(speed);
    }

    // Sets the rotation speed of the azimuth motor in open loop mode
    public void setRotation(double rotation) {
        mAzimuthMotor.set(rotation);
    }

    // Gets the speed of the drive motor
    public double getSpeed() {
        return mDriveEncoder.getVelocity();
    }

    // Gets the rotation position of the azimuth module
    public double getRotation() {
        return mAzimuthAbsoluteEncoder.getPosition().getValueAsDouble()*360.0;
    }

    // Gets the x/y location of the module relative to the center of the robot
    @Override
    public Translation2d getModuleLocation() {
        return mLocation;
    }

    // Get the state (speed/rotation) of the swerve module
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(getRotation()));
    }

    // Run when swerve drive is first initialized
    @Override
    public void init() {

    }

    // Get the position of swerve modules (distance and angle)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromDegrees(getRotation()));
    }

    // Get the distance of the drive encoder
    public double getDistance() {
        return mDriveEncoder.getPosition();
    }

    // Log swerve data
    @Override
    public void log() {
        SmartDashboard.putNumber(mName + " Current", mDriveMotor.getOutputCurrent());
        SmartDashboard.putNumber(mName + " Speed", mDriveEncoder.getVelocity());
    }

    // Set the speed and direction of the swerve module
    @Override
    public void set(SwerveModuleState drive) {
        Rotation2d current = Rotation2d.fromDegrees(mAzimuthAbsoluteEncoder.getPosition().getValueAsDouble()*360.0);
        SwerveModuleState optimizedState = SwerveModuleState.optimize(drive, current);
        double setpoint = optimizedState.angle.getDegrees();
        double velocity = optimizedState.speedMetersPerSecond;
        //mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        mAzimuthMotor.set(azimuthPID.calculate(current.getDegrees(), setpoint));
        mDriveMotor.set(velocity);
        SmartDashboard.putNumber(mName + "Angle", current.getDegrees());
    }

    @Override
    public void setClosedLoop(SwerveModuleState drive) {
        Rotation2d current = Rotation2d.fromDegrees(mAzimuthAbsoluteEncoder.getPosition().getValueAsDouble()*360.0);
        SwerveModuleState optimizedState = SwerveModuleState.optimize(drive, current);
        double setpoint = optimizedState.angle.getDegrees();
        double velocity = optimizedState.speedMetersPerSecond;
        //mAzimuthPID.setReference(setpoint, ControlType.kPosition);
        
        mAzimuthMotor.set(azimuthPID.calculate(current.getDegrees(), setpoint));
        mDrivePID.setReference(velocity, ControlType.kVelocity);
    }

    // Stop all motors
    @Override
    public void stop() {
        mAzimuthMotor.set(0);
        mDriveMotor.set(0);
    }

    // set angle of swerve drive
    @Override
    public void setAngle(double angle) {
       // mAzimuthPID.setReference(angle, ControlType.kPosition);
        azimuthPID.calculate(mAzimuthAbsoluteEncoder.getPosition().getValueAsDouble()*360.0, angle);

    }

}
