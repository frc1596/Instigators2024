package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivotSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 private final CANSparkMax mShooterPivot = new CANSparkMax(15, MotorType.kBrushless);
  private final RelativeEncoder mIntakeEncoder;
    private final SparkPIDController mPivotPID;

  public ShooterPivotSubsystem() {
        // Configure drive motor controller parameters
        mPivotPID = mShooterPivot.getPIDController();
        mPivotPID.setP(1.0/360.0);
        mPivotPID.setSmartMotionMaxAccel(.1,0);
        //mPivotPID.getSmartMotionAccelStrategy
        mShooterPivot.setIdleMode(IdleMode.kBrake);
        mShooterPivot.setSmartCurrentLimit(5);
         mIntakeEncoder = mShooterPivot.getEncoder();
         mShooterPivot.setClosedLoopRampRate(0);
         mShooterPivot.setOpenLoopRampRate(.1);
         mIntakeEncoder.setPositionConversionFactor(360.0/(60.0));
         mIntakeEncoder.setPosition(0);
         mShooterPivot.burnFlash();

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