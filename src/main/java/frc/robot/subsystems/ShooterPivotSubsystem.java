package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivotSubsystem extends SubsystemBase {

  private final DigitalInput noteSensor = new DigitalInput(1);

  private static double kDt = 0.02; //some sort of timing thing, don't touch probably

 private final CANSparkMax mShooterPivot = new CANSparkMax(15, MotorType.kBrushless);
 private final CANSparkMax mShooterIntake1 = new CANSparkMax(16, MotorType.kBrushless);
  private final CANSparkMax mShooterIntake2 = new CANSparkMax(17, MotorType.kBrushless);
  private final CANSparkMax mShooter1 = new CANSparkMax(18, MotorType.kBrushless);
  private final CANSparkMax mShooter2 = new CANSparkMax(19, MotorType.kBrushless);
 // private final RelativeEncoder mShooterEncoder;
    private final SparkPIDController mPivotPID;
    private final SparkPIDController mShooterPID;
private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(10, 1));
private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private static final SparkMaxAlternateEncoder.Type kAltEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
  private static final int kCPR = 8192;
  private RelativeEncoder m_alternateEncoder;

  public ShooterPivotSubsystem() {
    m_alternateEncoder = mShooterPivot.getAlternateEncoder(kAltEncType, kCPR);
    m_alternateEncoder.setInverted(true);
    m_alternateEncoder.setPosition(0);
        // Configure drive motor controller parameters
        mPivotPID = mShooterPivot.getPIDController();
        mPivotPID.setP(2);
        mPivotPID.setI(0);
        mPivotPID.setD(0);
       // mPivotPID.setFF(0.1);
        mPivotPID.setFeedbackDevice(m_alternateEncoder);

        //mPivotPID.getSmartMotionAccelStrategy
        mShooterPivot.setIdleMode(IdleMode.kBrake);
        mShooterPivot.setSmartCurrentLimit(30);
        // mShooterEncoder = mShooterPivot.getEncoder();
         mShooterPivot.setClosedLoopRampRate(0);
         mShooterPivot.setOpenLoopRampRate(.1);
         //mShooterEncoder.setPositionConversionFactor(360.0/(60.0));
         //mShooterEncoder.setPosition(0);
         mShooterPivot.burnFlash();


         mShooterIntake1.setIdleMode(IdleMode.kBrake);
         mShooterIntake1.setSmartCurrentLimit(30);
         mShooterIntake1.setClosedLoopRampRate(0);
         mShooterIntake1.setOpenLoopRampRate(.1);
         mShooterIntake1.burnFlash();

         mShooterIntake2.setIdleMode(IdleMode.kBrake);
         mShooterIntake2.setSmartCurrentLimit(30);
         mShooterIntake2.setClosedLoopRampRate(0);
         mShooterIntake2.setOpenLoopRampRate(.1);
         mShooterIntake2.burnFlash();

         mShooter1.setIdleMode(IdleMode.kCoast);
         mShooter1.setSmartCurrentLimit(40);
         mShooter1.setClosedLoopRampRate(0);
         mShooter1.setOpenLoopRampRate(.1);
         mShooter1.burnFlash();
       
          mShooterPID = mShooter1.getPIDController();

          mShooterPID.setSmartMotionMaxVelocity(6000, 0);
          mShooterPID.setSmartMotionMinOutputVelocity(1, 0);
          mShooterPID.setSmartMotionMaxAccel(10, 0);
          mShooterPID.setSmartMotionAllowedClosedLoopError(5, 0);
      
         mShooter2.follow(mShooter1, true);
        // mShooter2.setInverted(true);
         mShooter2.setIdleMode(IdleMode.kCoast);
         mShooter2.setSmartCurrentLimit(40);
         mShooter2.setClosedLoopRampRate(0);
         mShooter2.setOpenLoopRampRate(.1);
         mShooter2.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
        mPivotPID.setReference(m_setpoint.position, com.revrobotics.CANSparkBase.ControlType.kPosition);
      
     if (DriverStation.isDisabled()){
      //elbowSetpoint = new State(getElbowPostition(), 0);
      //shoulderSetpoint = new State(getShoulderPostition(), 0);
      //elbowGoal = elbowSetpoint;
      //shoulderGoal = shoulderSetpoint;
     }

      }

      // set angle of swerve drive
    public void setAngle(double angle) {
        m_goal = new TrapezoidProfile.State(angle, 0);
    }

    public void shooterIntake(double speed) {
      mShooterIntake1.set(speed);
      mShooterIntake2.set(speed);
    }

    public void shooterShoot(double speed) {
        mShooterPID.setReference(speed, com.revrobotics.CANSparkBase.ControlType.kVelocity);
    }

    public double getShooterVelocity(){
      SmartDashboard.putNumber("Shooter Speed", mShooter1.getEncoder().getVelocity());
      return(mShooter1.getEncoder().getVelocity());
    }

    public void shooterShootStop() {
        mShooter1.stopMotor();;
      }

    public boolean getSensor(){
      return(noteSensor.get());
    }
}