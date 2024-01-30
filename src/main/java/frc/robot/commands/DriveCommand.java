package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {
    public final SwerveSubsystem mSwerve;
    public XboxController mController;
    private PIDController rotationController = new PIDController(0.4, .0, .0);

    private PIDController limController = new PIDController(0.3, 0.0, 0.0);
    private PIDController strafeController = new PIDController(0.3,0,0);
    private JoystickHelper xHelper = new JoystickHelper(0);
  private JoystickHelper yHelper = new JoystickHelper(0);
  private JoystickHelper rotHelper = new JoystickHelper(0);
  private JoystickHelper xrHelper = new JoystickHelper(0);
  private JoystickHelper yrHelper = new JoystickHelper(0);
    private SlewRateLimiter xfilter = new SlewRateLimiter(3);
    private SlewRateLimiter yfilter = new SlewRateLimiter(3);
    private SlewRateLimiter rotfilter = new SlewRateLimiter(3);
    private boolean lastScan;
    private double driveFactor = 1;

    public DriveCommand(SwerveSubsystem swerve, XboxController XboxController) {
        mSwerve = swerve;
        mController = XboxController;
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        limController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(mSwerve);
      }
       // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVel = 0;
    double yVel = 0;
    double rotVel = 0;
    double xrVel = 0;
    double yrVel = 0;

    yVel = mController.getLeftY();
    xVel = mController.getLeftX();

    yrVel = mController.getRightY();
    xrVel = mController.getRightX();

    // slow down when right bumper
    if (mController.getRightBumper()) {
      driveFactor = 0.5;
    } else {
      driveFactor = 1.0;
    }

    //rumble when 20 seconds left
    if (DriverStation.isTeleop() && (DriverStation.getMatchTime() < 20.0) && (DriverStation.getMatchTime() > 19.0)) {
      mController.setRumble(RumbleType.kLeftRumble, 1);
      mController.setRumble(RumbleType.kRightRumble, 1);
    } else {
      mController.setRumble(RumbleType.kLeftRumble, 0);
      mController.setRumble(RumbleType.kRightRumble, 0);
    }

    rotVel = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();
    yVel = yHelper.setInput(yVel).applyPower(2).value;
    xVel = xHelper.setInput(xVel).applyPower(2).value;
    rotVel = rotHelper.setInput(rotVel).applyPower(2).value;

    yrVel = yrHelper.setInput(yrVel).applyPower(yrVel).value;
    xrVel = xrHelper.setInput(xrVel).applyPower(yrVel).value;

    yVel = yVel * driveFactor;
    xVel = xVel * driveFactor;
    rotVel = rotVel * driveFactor;

    Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(-mController.getRightX(), -mController.getRightY()));
    if (!mController.getLeftBumper()) {
      joystickAngle = joystickAngle.plus(Rotation2d.fromDegrees(180));
    }

    double joystickMagnitude = Math.sqrt(
        (mController.getRightY() * mController.getRightY()) + (mController.getRightX() * mController.getRightX()));
    if (joystickMagnitude > .2) {
        rotVel = mController.getRightX();
      //  rotVel = -rotationController.calculate(Rotation2d.fromDegrees(mSwerve.getRotation()).getRadians(),
      //     joystickAngle.getRadians());
      if (Math.abs(rotVel) > joystickMagnitude) {
        rotVel = joystickMagnitude * Math.signum(rotVel);
      }
    }

   // rotVel = -rotVel; // controls were inverted

    if (mController.getRawButton(7) & !lastScan) {
      mSwerve.resetGyro();
    }
    lastScan = mController.getRawButton(7);

    boolean fieldRelative = !mController.getAButton();
//ChassisSpeeds regularDrive = ChassisSpeeds.fromFieldRelativeSpeeds(xrVel, yrVel, joystickMagnitude, joystickAngle);
    mSwerve.drive(yfilter.calculate(yVel), xfilter.calculate(xVel), rotVel, fieldRelative);
     // mSwerve.driveFieldRelative(regularDrive);
    }

    // mDrivetrain.drive(yVel,xVel, rotVel, fieldRelative);
    // mDrivetrain.setModulesAngle(xVel);

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public class JoystickHelper {
    public double value;

    public JoystickHelper(double input){
        value = input;
    }

    public JoystickHelper applyDeadband(double deadband){
        if (Math.abs(value) < Math.abs(deadband)){
            value = 0;
        }

        return this;
    }

    public JoystickHelper applyPower(double power){
        value = Math.abs(Math.pow(value, power))*Math.signum(value);

        return this;
    }

    public JoystickHelper setInput(double input){
        value = input;
        return this;
    }

}

}
