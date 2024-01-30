// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivotSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 private final CANSparkMax mIntakePivot = new CANSparkMax(13, MotorType.kBrushless);
  private final RelativeEncoder mIntakeEncoder;
    private final SparkPIDController mPivotPID;


  public IntakePivotSubsystem() {
        // Configure drive motor controller parameters
        mPivotPID = mIntakePivot.getPIDController();
        mPivotPID.setP(1.0/360.0);
        mPivotPID.setSmartMotionMaxAccel(1,0);
        mIntakePivot.setIdleMode(IdleMode.kBrake);
        mIntakePivot.setSmartCurrentLimit(5);
         mIntakeEncoder = mIntakePivot.getEncoder();
         mIntakePivot.setClosedLoopRampRate(0);
         mIntakePivot.setOpenLoopRampRate(.1);
         mIntakeEncoder.setPositionConversionFactor(360.0/(20.0));
         mIntakeEncoder.setPosition(0);
         mIntakePivot.burnFlash();

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

      // set angle of swerve drive
    public void setAngle(double angle) {
        // mAzimuthPID.setReference(angle, ControlType.kPosition);
        mPivotPID.setReference(angle, ControlType.kPosition) ;
    }
}
