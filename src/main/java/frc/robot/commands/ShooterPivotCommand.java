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
      m_subsystem.setAngle(-130);
  } else if((m_Controller.getPOV() <= 135) && (m_Controller.getPOV() >= 45)){
    m_subsystem.setAngle(-65);
    m_subsystem.shooterIntake(1);
  }
  else {
     m_subsystem.setAngle(-15);
     m_subsystem.shooterIntake(0);

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
