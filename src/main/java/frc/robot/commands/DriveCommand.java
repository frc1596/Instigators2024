package frc.robot.commands;

import org.deceivers.drivers.LimelightHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {
    public final SwerveSubsystem mSwerve;
    public XboxController mController;
    public XboxController mController2;
    public Limelight mLimelight;

    private PIDController rotationController = new PIDController(0.4, .0, .0);

    private PIDController limController = new PIDController(0.3, 0.0, 0.0);

    private PIDController autoAimController = new PIDController(1.0,0,0);

    private JoystickHelper xHelper = new JoystickHelper(0);
  private JoystickHelper yHelper = new JoystickHelper(0);
  private JoystickHelper rotHelper = new JoystickHelper(0);

    private SlewRateLimiter xfilter = new SlewRateLimiter(3);
    private SlewRateLimiter yfilter = new SlewRateLimiter(3);

    private PIDController shooterAimPID = new PIDController(0.01, 0, 0);

    private boolean lastScan;
    private double driveFactor = 1;

    public DriveCommand(SwerveSubsystem swerve, XboxController XboxController, XboxController XboxController2, Limelight limelight) {
        mSwerve = swerve;
        mLimelight = limelight;
        mController2 = XboxController2;
        mController = XboxController;

        autoAimController.enableContinuousInput(-0.5, 0.5);

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
    double driveDirection = 0;
    double driveMagnitude = 0;

    yVel = mController.getLeftY();
    xVel = mController.getLeftX();

    //Trigger Drive Method (comment out these 4 lines to go back to normal drive)
    driveDirection = Math.atan2(yVel, xVel);
    driveMagnitude = mController.getRightTriggerAxis();
    yVel = Math.sin(driveDirection) * driveMagnitude;
    xVel = Math.cos(driveDirection) * driveMagnitude;

    //yrVel = mController.getRightY();
    //xrVel = mController.getRightX();

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

    yVel = yHelper.setInput(yVel).applyPower(2).value;
    xVel = xHelper.setInput(xVel).applyPower(2).value;
    rotVel = rotHelper.setInput(rotVel).applyPower(2).value;

    //yrVel = yrHelper.setInput(yrVel).applyPower(yrVel).value;
    //xrVel = xrHelper.setInput(xrVel).applyPower(yrVel).value;

    yVel = yVel * driveFactor;
    xVel = xVel * driveFactor;
    rotVel = rotVel * driveFactor;

    //Rotation2d joystickAngle = Rotation2d.fromRadians(Math.atan2(-mController.getRightX(), -mController.getRightY()));
    //if (!mController.getLeftBumper()) {
    //  joystickAngle = joystickAngle.plus(Rotation2d.fromDegrees(180));
    //}
    SmartDashboard.putNumber("FieldOrientation", mLimelight.getPose().getRotation().getRotations());

    double joystickMagnitude = Math.sqrt(
        (mController.getRightY() * mController.getRightY()) + (mController.getRightX() * mController.getRightX()));
  
    if(mController2.getLeftBumper() && !(mLimelight.getFid() == -1)){
      if((mLimelight.getFid() == 3) || (mLimelight.getFid() == 4)){
      //  rotVel = -autoAimController.calculate(LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getRotation().getAngle()-Math.toRadians(mSwerve.getRotation()), 0);
      //  SmartDashboard.putNumber("RotVel",rotVel);
      //  SmartDashboard.putNumber("LimelightRotation",LimelightHelpers.getTargetPose3d_RobotSpace("limelight").getRotation().getAngle());
      }
    }
    else if (joystickMagnitude > .04) {
        rotVel = mController.getRightX();
        rotVel = rotHelper.setInput(rotVel).applyPower(2).value;

      //  rotVel = -rotationController.calculate(Rotation2d.fromDegrees(mSwerve.getRotation()).getRadians(),
      //     joystickAngle.getRadians());
      if (Math.abs(rotVel) > joystickMagnitude) {
        rotVel = joystickMagnitude * Math.signum(rotVel);
      }
    }

    //rezero
    if (mController.getRawButton(7) & !lastScan) {
      mSwerve.resetGyro();
    }
    lastScan = mController.getRawButton(7);

    SmartDashboard.putNumber("Commanded xVel", xVel);
        SmartDashboard.putNumber("Commanded yVel", yVel);

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
