package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterPivotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterPivotSubsystem m_subsystem;
  private final XboxController m_Controller;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterPivotCommand(ShooterPivotSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_Controller = controller;
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
      m_subsystem.setAngle(-0.35); //-0.4
  } else if((m_Controller.getPOV() <= 135) && (m_Controller.getPOV() >= 45) && !m_subsystem.getSensor()){
    m_subsystem.setAngle(-0.18); //-0.2
    m_subsystem.shooterIntake(.4);
  }
    else if((m_Controller.getPOV() <= 135) && (m_Controller.getPOV() >= 45) && m_subsystem.getSensor()){
        m_subsystem.shooterIntake(-0.1);
            m_subsystem.shooterShoot(0.5);
            m_subsystem.setAngle(-0.18); //-0.2

    }
    else if((!((m_Controller.getPOV() <= 135) && (m_Controller.getPOV() >= 45)) && m_subsystem.getSensor())){
        m_subsystem.shooterIntake(-0.1);
        m_subsystem.shooterShoot(0.5);
    }
  else if(m_Controller.getPOV() == 0){
        m_subsystem.setAngle(-0.21); //-0.28
        m_subsystem.shooterIntake(0);
        m_subsystem.shooterShootStop();
  }
  else {
     m_subsystem.setAngle(-0.05); //-0.05
     m_subsystem.shooterIntake(0);
     m_subsystem.shooterShootStop();
  }

  if(m_Controller.getLeftTriggerAxis() > 0.1){
    m_subsystem.shooterShoot(5);
    
  }else if(m_Controller.getRightTriggerAxis() > 0.1){
    m_subsystem.shooterShoot(-5);
    if(m_subsystem.getShooterVelocity() < -3.8){
          m_subsystem.shooterIntake(0.4);
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
