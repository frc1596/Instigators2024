// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.ShooterPivotCommand;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final IntakePivotSubsystem intake = new IntakePivotSubsystem();
  private final ShooterPivotSubsystem shooter = new ShooterPivotSubsystem();
  XboxController driverController = new XboxController(0);
  XboxController operaterController = new XboxController(1);
  private final Compressor mCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid m_doubleSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
 // private final AHRS gyro = new AHRS(SPI.Port.kMXP);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  //private final SendableChooser<Command> autoChooser;

  @Override
  public void robotInit() {
        autoChooser = AutoBuilder.buildAutoChooser();

   //new Compressor(null)
   // swerve.setDefaultCommand(new DriveCommand(swerve, driverController));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putData("Auto Chooser", autoChooser);

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    //SmartDashboard.putNumber("Gyro", gyro.getAngle());
  }

  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = getAutonomousCommand();

    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);

    swerve.setDefaultCommand(new DriveCommand(swerve, driverController));
    intake.setDefaultCommand(new IntakePivotCommand(intake, operaterController));
    shooter.setDefaultCommand(new ShooterPivotCommand(shooter, operaterController));
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (driverController.getYButtonPressed()) {
      m_doubleSolenoid.toggle();
   }
   if (DriverStation.isTeleop() && (DriverStation.getMatchTime() < 0.6)){
      m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
   }
   // SmartDashboard.putNumber("Angle", gyro.getAngle());

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

