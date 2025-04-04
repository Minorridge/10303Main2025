// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import dev.choreo.lib.Choreo;          // Correct package for Choreo class
import dev.choreo.lib.ChoreoTrajectory; // Correct package for ChoreoTrajectory type
import edu.wpi.first.math.kinematics.SwerveModuleState; // Needed for the corrected command lambda later
import choreo.Choreo;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();

  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
 Field2d m_field = new Field2d();

  ChoreoTrajectory traj;
  public RobotContainer() {
    traj = Choreo.getTrajectory("Trajectory");

    m_field.getObject("traj").setPoses(
      traj.getInitialPose(), traj.getFinalPose()
    );
    m_field.getObject("trajPoses").setPoses(
      traj.getPoses()
    );

    SmartDashboard.putData(m_field);
    
    // Build an auto chooser. This will use Commands.none() as the default option.

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    // Configure the button bindings
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
    // Left Bumper -> Run command to auto-align robot chassis according to AprilTag 
    m_driverController.povDown().onTrue(new AutoAlign(true, m_robotDrive).withTimeout(3));

  }

  public double getSimulationTotalCurrentDraw() {
    return m_coralSubSystem.getSimulationCurrentDraw()
        + m_algaeSubsystem.getSimulationCurrentDraw();
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
}
  public Command getAutonomousCommand() {
    var thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_robotDrive.resetOdometry(traj.getInitialPose());

    Command swerveCommand = Choreo.choreoSwerveCommand(
        traj, // Choreo trajectory from above
        m_robotDrive::getPose, // A function that returns the current field-relative pose of the robot: your
                               // wheel or vision odometry
        new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // PIDController for field-relative X
                                                                                   // translation (input: X error in meters,
                                                                                   // output: m/s).
        new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // PIDController for field-relative Y
                                                                                   // translation (input: Y error in meters,
                                                                                   // output: m/s).
        thetaController, // PID constants to correct for rotation
                         // error
        (ChassisSpeeds speeds) -> m_robotDrive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false),
        true, // Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
        m_robotDrive // The subsystem(s) to require, typically your drive subsystem only
    );

    return Commands.sequence(
        Commands.runOnce(() -> m_robotDrive.resetOdometry(traj.getInitialPose())),
        swerveCommand,
        m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false))
    );
  }

  public void periodic() {
    m_field.setRobotPose(m_robotDrive.getPose());
  }
}
