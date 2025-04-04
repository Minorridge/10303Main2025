package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class LimelightAutoAlign3D extends Command {

  private final ProfiledPIDController drivePID;
  private final ProfiledPIDController turnPID;
  private final ProfiledPIDController strafePID;
  private final Timer timer;
  private final double targetDistance;
  private final double maxDriveSpeed;
  private final double maxTurnSpeed; // Modified maxTurnSpeed
  private final double maxStrafeSpeed;
  private final double distanceTolerance;
  private final double angleTolerance;
  private final double strafeTolerance;
  private final DriveSubsystem driveSubsystem;
  private final double timeoutSeconds;
  private final NetworkTable limelightTable;
  private boolean isSearching = true;

  private enum AlignmentState {
    ROTATE_TO_ALIGN,
    DRIVE_TO_DISTANCE,
    FINE_TUNE,
    ALIGNED
  }

  private AlignmentState currentState = AlignmentState.ROTATE_TO_ALIGN;

  public LimelightAutoAlign3D(
      ProfiledPIDController drivePID,
      ProfiledPIDController turnPID,
      ProfiledPIDController strafePID,
      double targetDistance,
      double maxDriveSpeed,
      double maxTurnSpeed,
      double maxStrafeSpeed,
      double distanceTolerance,
      double angleTolerance,
      double strafeTolerance,
      DriveSubsystem driveSubsystem,
      double timeoutSeconds) {
    this.drivePID = drivePID;
    this.turnPID = turnPID;
    this.strafePID = strafePID;
    this.targetDistance = targetDistance;
    this.maxDriveSpeed = maxDriveSpeed;
    this.maxTurnSpeed = 0.3; // Modified maxTurnSpeed
    this.maxStrafeSpeed = maxStrafeSpeed;
    this.distanceTolerance = distanceTolerance;
    this.angleTolerance = angleTolerance;
    this.strafeTolerance = strafeTolerance;
    this.timer = new Timer();
    this.driveSubsystem = driveSubsystem;
    this.timeoutSeconds = timeoutSeconds;
    addRequirements(driveSubsystem);

    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void initialize() {
    drivePID.reset(driveSubsystem.getPose().getX());
    turnPID.reset(driveSubsystem.getHeading());
    strafePID.reset(driveSubsystem.getPose().getY());
    timer.reset();
    timer.start();
    currentState = AlignmentState.ROTATE_TO_ALIGN;
    isSearching = true;
  }

  @Override
  public void execute() {
    double hasTarget = limelightTable.getEntry("tv").getDouble(0);
    double[] botPose = null;

    if (hasTarget == 1 && isSearching) {
      isSearching = false;
    }

    if (hasTarget == 1 && !isSearching) {
      botPose = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
      double tx = limelightTable.getEntry("tx").getDouble(0);
      double distance =
          Math.sqrt(Math.pow(botPose[0], 2) + Math.pow(botPose[1], 2) + Math.pow(botPose[2], 2));
      double distanceError = distance - targetDistance;
      double angleError = Math.toRadians(tx);
      double strafeError = botPose[0];

      double driveSpeed = 0;
      double turnSpeed = 0;
      double strafeSpeed = 0;

      switch (currentState) {
        case ROTATE_TO_ALIGN:
          turnSpeed = turnPID.calculate(0, angleError);
          turnSpeed = Math.max(Math.min(turnSpeed, maxTurnSpeed), -maxTurnSpeed);
          if (Math.abs(angleError) < Math.toRadians(10)) {
            currentState = AlignmentState.DRIVE_TO_DISTANCE;
          }
          break;
        case DRIVE_TO_DISTANCE:
          driveSpeed = drivePID.calculate(0, distanceError);
          driveSpeed = Math.max(Math.min(driveSpeed, maxDriveSpeed), -maxDriveSpeed);
          if (Math.abs(distanceError) < 0.1) {
            currentState = AlignmentState.FINE_TUNE;
          }
          break;
        case FINE_TUNE:
          driveSpeed = drivePID.calculate(0, distanceError);
          driveSpeed = Math.max(Math.min(driveSpeed, maxDriveSpeed * 0.5), -maxDriveSpeed * 0.5);
          turnSpeed = turnPID.calculate(0, angleError);
          turnSpeed = Math.max(Math.min(turnSpeed, maxTurnSpeed * 0.5), -maxTurnSpeed * 0.5);
          strafeSpeed = strafePID.calculate(0, strafeError);
          strafeSpeed =
              Math.max(Math.min(strafeSpeed, maxStrafeSpeed * 0.5), -maxStrafeSpeed * 0.5);
          if (Math.abs(angleError) < angleTolerance
              && Math.abs(distanceError) < distanceTolerance
              && Math.abs(strafeError) < strafeTolerance) {
            currentState = AlignmentState.ALIGNED;
          }
          break;
        case ALIGNED:
          break;
      }

      driveSubsystem.drive(driveSpeed, strafeSpeed, turnSpeed, false);

      SmartDashboard.putNumber("Limelight Angle Error", Math.toDegrees(angleError));
      SmartDashboard.putNumber("Limelight Strafe Error", strafeError);
      SmartDashboard.putNumber("Distance Error", distanceError);
      SmartDashboard.putNumber("Drive Speed", driveSpeed);
      SmartDashboard.putNumber("Turn Speed", turnSpeed);
      SmartDashboard.putNumber("Strafe Speed", strafeSpeed);

      // Add PID tuning and debugging
      SmartDashboard.putNumber("Turn PID P", turnPID.getP());
      SmartDashboard.putNumber("Turn PID I", turnPID.getI());
      SmartDashboard.putNumber("Turn PID D", turnPID.getD());
      turnPID.setP(SmartDashboard.getNumber("Turn PID P", turnPID.getP()));
      turnPID.setI(SmartDashboard.getNumber("Turn PID I", turnPID.getI()));
      turnPID.setD(SmartDashboard.getNumber("Turn PID D", turnPID.getD()));
      System.out.println("tx: " + tx + ", angleError: " + Math.toDegrees(angleError));

    } else if (isSearching) {
      double searchTurnSpeed = 0.2;
      if (timer.get() < 3.0) {
        double turnDirection = (timer.get() % 2 < 1) ? 1 : -1;
        driveSubsystem.drive(0, 0, turnDirection * searchTurnSpeed, false);
      } else {
        driveSubsystem.drive(0, 0, 0, false);
      }
      SmartDashboard.putNumber("Limelight Distance", -1.0);
      SmartDashboard.putNumber("Limelight Angle Error", 0.0);
      SmartDashboard.putNumber("Limelight Strafe Error", 0.0);
    } else {
      driveSubsystem.drive(0, 0, 0, false);
    }

    if (botPose != null) {
      double distance =
          Math.sqrt(Math.pow(botPose[0], 2) + Math.pow(botPose[1], 2) + Math.pow(botPose[2], 2));
      SmartDashboard.putNumber("Limelight Distance", distance);
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(0, 0, 0, false);
    timer.stop();
    SmartDashboard.putNumber("Limelight Distance", -1.0);
    SmartDashboard.putNumber("Limelight Angle Error", 0.0);
    SmartDashboard.putNumber("Limelight Strafe Error", 0.0);
    turnPID.reset(driveSubsystem.getHeading());
  }

  @Override
  public boolean isFinished() {
    return currentState == AlignmentState.ALIGNED || timer.get() >= timeoutSeconds;
  }
}
