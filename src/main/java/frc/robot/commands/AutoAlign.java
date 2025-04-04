// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlign extends Command {
  private PIDController distanceController, heightController, rotationController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem m_robotDrive;
  private double tagID = -1;

  public AutoAlign(boolean isRightScore, DriveSubsystem m_robotDrive) {
    distanceController = new PIDController(2.5, 0.0, 0);  
    heightController = new PIDController(2, 0.0, 0); 
    rotationController = new PIDController(1, 0, 0);  
    this.isRightScore = isRightScore;
    this.m_robotDrive = m_robotDrive;
    addRequirements(m_robotDrive);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotationController.setSetpoint(0.05);
    rotationController.setTolerance(0.5);

    distanceController.setSetpoint(0.05);
    distanceController.setTolerance(0.05);

    heightController.setSetpoint(isRightScore ? 0.05 : -0.05);
    heightController.setTolerance(1);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = distanceController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -heightController.calculate(postions[0]);
      double rotValue = -rotationController.calculate(postions[4]);

      m_robotDrive.drive(xSpeed, ySpeed, rotValue, false);

      if (!rotationController.atSetpoint() ||
          !heightController.atSetpoint() ||
          !distanceController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      m_robotDrive.drive(1, 1, 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    m_robotDrive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(0.3) ||
        stopTimer.hasElapsed(0.3);
  }
}

  /*  private final DriveSubsystem m_swerve;
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private final PIDController rotationPID = new PIDController(1.0, 0, 0);
    private final PIDController distancePID = new PIDController(2.5, 0.0, 0.2); 
    private final PIDController headingPID = new PIDController(2, 0, 0); 

    private final double rotationTolerance = 0.5; // Degrees
    private final double distanceTolerance = 0.05; // Meters
    private final double headingTolerance = 1.0; // Degrees

    private final double limelightHeight = Constants.LimelightConstants.MOUNT_HEIGHT;
    private final double limelightAngle = Constants.LimelightConstants.MOUNT_ANGLE;
    
    public AutoAlign(DriveSubsystem swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(0);
        distancePID.setSetpoint(0.5); 
       this.m_robotDrive = DriveSubsystem.m_robotDrive;
        addRequirements(m_robotDrive);
    }

    @Override
    public void execute() {
        double tv = table.getEntry("tv").getDouble(0.0); // No units (1.0 if target is visible, 0.0 otherwise)
        if (tv == 1.0) {
            double tx = table.getEntry("tx").getDouble(0.0); // Degrees
            double rotationOutput = rotationPID.calculate(tx); // Degrees

            double tid = table.getEntry("tid").getDouble(0.0); // ID number (no units)
            double targetHeight = AprilTagHeightDB.getHeight(tid); // Meters

            if (targetHeight != -1) {
                double distanceOutput = distancePID.calculate(calculateDistance(targetHeight)); // Meters/second

                // Deceleration zone
                double currentDistance = calculateDistance(targetHeight); // Meters
                double speedScale = Math.min(1.0, currentDistance / 1.0); // No units
                distanceOutput *= speedScale; // Meters/second

                rotationOutput = Math.max(Math.min(rotationOutput, 0.2), -0.2); // Radians/second
                distanceOutput = Math.max(Math.min(distanceOutput, 0.2), -0.2); // Meters/second

                double ty = table.getEntry("ty").getDouble(0.0); // Degrees
                double ta = table.getEntry("ta").getDouble(0.0); // Percentage of Limelight's FOV (no units)

                double headingOutput = headingPID.calculate(ty); // Degrees

                // Scale heading output based on target area
                double headingScale = Math.min(1.0, ta / desiredTargetArea); // No units
                headingOutput *= headingScale; // Degrees

                ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    distanceOutput, // Meters/second
                    0.0, // Meters/second
                    Math.toRadians(rotationOutput + headingOutput), // Radians/second
                    m_swerve.getRotation2d() // Rotation2d (no units)
                );

                m_swerve.drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, true);
            } else {
                m_swerve.drive(0.0, 0.0, 0.0, false);
            }
        } else {
            m_swerve.drive(0.0, 0.0, 0.0, false);
        }
    }

    @Override
    public boolean isFinished() {
        double tv = table.getEntry("tv").getDouble(0.0); // No units
        double tx = table.getEntry("tx").getDouble(0.0); // Degrees
        double tid = table.getEntry("tid").getDouble(0.0); // ID number
        double targetHeight = AprilTagHeightDB.getHeight(tid); // Meters
        if (targetHeight == -1) {
            return true;
        }

        double distanceError = Math.abs(calculateDistance(targetHeight) - distancePID.getSetpoint()); // Meters
        double ty = table.getEntry("ty").getDouble(0.0); // Degrees
        double currentVelocity = Math.abs(m_swerve.getRobotRelativeSpeeds().vxMetersPerSecond); // Meters/second

        return (Math.abs(tx) < rotationTolerance && distanceError < distanceTolerance && Math.abs(ty) < headingTolerance && currentVelocity < 0.05) || tv == 0.0; // Reduced tolerance and added velocity check
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0.0, 0.0, 0.0, false);
    }

    private double calculateDistance(double targetHeight) {
        double ty = table.getEntry("ty").getDouble(0.0); // Degrees
        double angleToTarget = ty + limelightAngle; // Degrees
        double distance = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(angleToTarget)); // Meters
        System.out.println("Calculated Distance: " + distance); // Meters
        return distance; // Meters
    }
        */