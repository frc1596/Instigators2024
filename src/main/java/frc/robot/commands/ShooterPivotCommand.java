package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.util.InterpolatingDouble;
import frc.robot.util.InterpolatingTreeMap;

import org.deceivers.drivers.LimelightHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterPivotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterPivotSubsystem m_subsystem;
  private final XboxController m_Controller;
  private final Limelight mLimelight;
    private PIDController autoShootPID = new PIDController(1.0,0,0);

  private int shootState = 1;
  
  private static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodMap = new InterpolatingTreeMap<>();
  static {
    hoodMap.put(new InterpolatingDouble(7.0), new InterpolatingDouble(-.21));
    hoodMap.put(new InterpolatingDouble(4.16), new InterpolatingDouble(-.2));
    hoodMap.put(new InterpolatingDouble(3.0), new InterpolatingDouble(-.185));
    hoodMap.put(new InterpolatingDouble(2.0), new InterpolatingDouble(-0.163));
    hoodMap.put(new InterpolatingDouble(1.0), new InterpolatingDouble(-0.11)); 
    hoodMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(-0.06));
  }
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterPivotCommand(ShooterPivotSubsystem subsystem, XboxController controller, Limelight limelight) {
    m_subsystem = subsystem;
    m_Controller = controller;
    mLimelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if((m_Controller.getPOV() <= 315) && (m_Controller.getPOV() >= 225)){
      m_subsystem.setAngle(-0.3); //-0.4
  } else if((m_Controller.getPOV() <= 135) && (m_Controller.getPOV() >= 45)){
    m_subsystem.setAngle(-0.13); //-0.2
    m_subsystem.shooterIntake(.4);
  }
  else if(m_Controller.getLeftBumper()){
        //Transform2d anglePos = mLimelight.getPose().minus(new Pose2d( 8.308467,  1.442593,  Rotation2d.fromDegrees(0)));
        //double desAngle = Math.atan2( 2.0 , anglePos.getX())/(2.0*Math.PI); //2 meters vertical difference between limelight and goal
        //double gotoAngle = -autoShootPID.calculate(m_subsystem.getAngle(),desAngle);
        
        SmartDashboard.putNumber("LimelightZ", LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ());
      //   SmartDashboard.putNumber("anglePosX", anglePos.getX());
      //   SmartDashboard.putNumber("anglePosY", anglePos.getY());

      //   SmartDashboard.putNumber("DesAngle", gotoAngle);
      //   if(gotoAngle < -0.4){
      //     gotoAngle = -0.4;
      //   } else if(gotoAngle > -0.1){
      //     gotoAngle = -0.1;
      //  }

        double outputAngle = hoodMap.getInterpolated(new InterpolatingDouble(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getZ())).value;
                      SmartDashboard.putNumber("OutputAngle", outputAngle);

        m_subsystem.setAngle(outputAngle); //-0.28
        m_subsystem.shooterIntake(0);
        m_subsystem.shooterShootStop();
  }
  else {
     m_subsystem.setAngle(-0.01); //-0.05
     m_subsystem.shooterIntake(0);
     m_subsystem.shooterShootStop();
  }

  if(m_Controller.getLeftTriggerAxis() > 0.1){
    m_subsystem.shooterShoot(0.1);
    
  }else if(m_Controller.getRightTriggerAxis() > 0.1){

   }

   if(shootState == 1){
    if(m_Controller.getRightTriggerAxis() > 0.1){
      shootState = 2;
    }
   } else if(shootState == 2){
        m_subsystem.shooterIntake(-0.1);
        m_subsystem.shooterShoot(0.5);
      if(!m_subsystem.getSensor()){
        shootState = 3;
      }
      if(!(m_Controller.getRightTriggerAxis() > 0.1)){
          shootState = 1;
      }
   } else if( shootState == 3){
      m_subsystem.shooterShoot(-4.1);
      if(m_subsystem.getShooterVelocity() < -4.1){
          m_subsystem.shooterIntake(1);
        }

     if(!(m_Controller.getRightTriggerAxis() > 0.1)){
        shootState = 1;
     }
   }
   
   // else{
  //   m_subsystem.shooterShootStop();
  //   if(m_subsystem.getSensor()){
  //     m_subsystem.shooterIntake(-0.2);
  //  }
  // }

  if(m_Controller.getYButton()){
    m_subsystem.shooterIntake(0.5);
  }
  


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

