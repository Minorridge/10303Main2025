// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimelightAutoAlign3D;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  private final ProfiledPIDController drivePID =
      new ProfiledPIDController(
          LimelightConstants.kDriveP,
          LimelightConstants.kDriveI,
          LimelightConstants.kDriveD,
          new TrapezoidProfile.Constraints(
              LimelightConstants.kDriveMaxV, LimelightConstants.kDriveMaxA));
  private final ProfiledPIDController turnPID =
      new ProfiledPIDController(
          LimelightConstants.kTurnP,
          LimelightConstants.kTurnI,
          LimelightConstants.kTurnD,
          new TrapezoidProfile.Constraints(
              LimelightConstants.kTurnMaxV, LimelightConstants.kTurnMaxA));
  private final ProfiledPIDController strafePID =
      new ProfiledPIDController(
          LimelightConstants.kStrafeP,
          LimelightConstants.kStrafeI,
          LimelightConstants.kStrafeD,
          new TrapezoidProfile.Constraints(
              LimelightConstants.kStrafeMaxV, LimelightConstants.kStrafeMaxA));

  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));

    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

    LimelightAutoAlign3D autoAlign3DCommand =
        new LimelightAutoAlign3D(
            drivePID,
            turnPID,
            strafePID,
            LimelightConstants.kTargetDistance,
            LimelightConstants.kMaxDriveSpeed,
            LimelightConstants.kMaxTurnSpeed,
            LimelightConstants.kMaxStrafeSpeed,
            LimelightConstants.kDistanceTolerance,
            LimelightConstants.kAngleTolerance,
            LimelightConstants.kStrafeTolerance,
            m_robotDrive,
            LimelightConstants.kTimeoutSeconds);

    m_driverController.povDown().onTrue(autoAlign3DCommand);
  }

  private void configureButtonBindings() {
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    m_driverController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());
    m_driverController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());
    m_driverController
        .b()
        .onTrue(
            m_coralSubSystem
                .setSetpointCommand(Setpoint.kFeederStation)
                .alongWith(m_algaeSubsystem.stowCommand()));
    m_driverController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    m_driverController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    m_driverController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.runIntakeCommand());
    m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.reverseIntakeCommand());
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());
  }

  public double getSimulationTotalCurrentDraw() {
    return m_coralSubSystem.getSimulationCurrentDraw()
        + m_algaeSubsystem.getSimulationCurrentDraw();
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
