package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagConstants; // Ensure correct import
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.LimelightHelpers; // Ensure correct import

/**
 * Command to align the robot to a target POSE ON THE FIELD, defined relative
 * to whichever valid "Reef" AprilTag (IDs 6-11, 17-22) is currently
 * considered primary by the Limelight.
 * Controls Field X, Field Y, and Field Theta simultaneously using PID.
 * Relies on DriveSubsystem's getPose() for current robot field pose.
 * ** Uses older Pose2d.log(target) signature for dx/dy based on build error **
 * ** Uses explicit Rotation2d math for angular error **
 */
public class AlignToDynamicReefTagCommand extends Command {

    private final DriveSubsystem m_driveSubsystem;
    private final String m_limelightName;
    private final Pose2d m_robotOffsetRelativeToTag; // Desired offset Trelative to Tag

    // PID Controllers for Field X, Field Y, and Field Theta
    // ** TUNING REQUIRED **
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_thetaController; // Operates on angle error in Radians

    // Default PID Gains (Use values from previous tuning as starting point)
    private static final double DEFAULT_FIELD_X_KP = .3;
    private static final double DEFAULT_FIELD_X_KI = 0.0;
    private static final double DEFAULT_FIELD_X_KD = 0.0;
    private static final double DEFAULT_FIELD_Y_KP = .3;
    private static final double DEFAULT_FIELD_Y_KI = 0.0;
    private static final double DEFAULT_FIELD_Y_KD = 0.0;
    private static final double DEFAULT_FIELD_THETA_KP = .1;
    private static final double DEFAULT_FIELD_THETA_KI = 0.0;
    private static final double DEFAULT_FIELD_THETA_KD = 0.1;

    // Tolerances
    private static final double FIELD_X_TOLERANCE_METERS = 0.1;
    private static final double FIELD_Y_TOLERANCE_METERS = 0.05;
    private static final double FIELD_THETA_TOLERANCE_RADIANS = Math.toRadians(2.0);

    // Max Output Speeds for PID controllers
    private static final double MAX_VELOCITY_METERS_PER_SEC = 1.5;
    private static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI;

    // Store current target details for dashboard and isFinished
    private int m_currentTagId = -1; // ID of the tag we are currently aligning to (-1 if none)
    private double m_currentErrorX = 0.0;
    private double m_currentErrorY = 0.0;
    private double m_currentErrorThetaRad = 0.0;


    /**
     * Creates a command to align the robot dynamically to the primary visible reef tag.
     * @param drivetrain The DriveSubsystem.
     * @param limelightName The NetworkTables name of the Limelight.
     * @param robotOffsetRelativeToTag Desired pose of the robot relative to the tag.
     */
    public AlignToDynamicReefTagCommand(DriveSubsystem drivetrain, String limelightName, Pose2d robotOffsetRelativeToTag) {
        m_driveSubsystem = drivetrain;
        m_limelightName = limelightName;
        m_robotOffsetRelativeToTag = robotOffsetRelativeToTag;

        m_xController = new PIDController(DEFAULT_FIELD_X_KP, DEFAULT_FIELD_X_KI, DEFAULT_FIELD_X_KD);
        m_yController = new PIDController(DEFAULT_FIELD_Y_KP, DEFAULT_FIELD_Y_KI, DEFAULT_FIELD_Y_KD);
        m_thetaController = new PIDController(DEFAULT_FIELD_THETA_KP, DEFAULT_FIELD_THETA_KI, DEFAULT_FIELD_THETA_KD);

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Tolerances don't need setpoint here, as we check error manually in isFinished
        // m_xController.setTolerance(FIELD_X_TOLERANCE_METERS); // Not needed if checking error directly
        // m_yController.setTolerance(FIELD_Y_TOLERANCE_METERS);
        // m_thetaController.setTolerance(FIELD_THETA_TOLERANCE_RADIANS);

        // Setpoints are always the target pose components
        // We will use calculate(measurement, setpoint)

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("AlignToDynamicReefTagCommand Initialized.");
        m_xController.reset();
        m_yController.reset();
        m_thetaController.reset();
        m_currentTagId = -1; // Reset current target

        // Publish default gains
        SmartDashboard.putNumber("DynAlign/PID/X_P", DEFAULT_FIELD_X_KP);
        SmartDashboard.putNumber("DynAlign/PID/X_I", DEFAULT_FIELD_X_KI);
        SmartDashboard.putNumber("DynAlign/PID/X_D", DEFAULT_FIELD_X_KD);
        SmartDashboard.putNumber("DynAlign/PID/Y_P", DEFAULT_FIELD_Y_KP);
        SmartDashboard.putNumber("DynAlign/PID/Y_I", DEFAULT_FIELD_Y_KI);
        SmartDashboard.putNumber("DynAlign/PID/Y_D", DEFAULT_FIELD_Y_KD);
        SmartDashboard.putNumber("DynAlign/PID/Theta_P", DEFAULT_FIELD_THETA_KP);
        SmartDashboard.putNumber("DynAlign/PID/Theta_I", DEFAULT_FIELD_THETA_KI);
        SmartDashboard.putNumber("DynAlign/PID/Theta_D", DEFAULT_FIELD_THETA_KD);

        // Initialize display values
        SmartDashboard.putNumber("DynAlign/TargetTagID", m_currentTagId);
        SmartDashboard.putNumber("DynAlign/CurrentX", 0.0);
        SmartDashboard.putNumber("DynAlign/CurrentY", 0.0);
        SmartDashboard.putNumber("DynAlign/CurrentTheta", 0.0);
        SmartDashboard.putNumber("DynAlign/ErrorX", 0.0);
        SmartDashboard.putNumber("DynAlign/ErrorY", 0.0);
        SmartDashboard.putNumber("DynAlign/ErrorTheta", 0.0);
    }

    @Override
    public void execute() {
        // Update PID gains from SmartDashboard
        m_xController.setPID(
            SmartDashboard.getNumber("DynAlign/PID/X_P", DEFAULT_FIELD_X_KP),
            SmartDashboard.getNumber("DynAlign/PID/X_I", DEFAULT_FIELD_X_KI),
            SmartDashboard.getNumber("DynAlign/PID/X_D", DEFAULT_FIELD_X_KD)
        );
        // ... (Update Y and Theta controllers similarly) ...
        m_yController.setPID(
             SmartDashboard.getNumber("DynAlign/PID/Y_P", DEFAULT_FIELD_Y_KP),
             SmartDashboard.getNumber("DynAlign/PID/Y_I", DEFAULT_FIELD_Y_KI),
             SmartDashboard.getNumber("DynAlign/PID/Y_D", DEFAULT_FIELD_Y_KD)
         );
         m_thetaController.setPID(
             SmartDashboard.getNumber("DynAlign/PID/Theta_P", DEFAULT_FIELD_THETA_KP),
             SmartDashboard.getNumber("DynAlign/PID/Theta_I", DEFAULT_FIELD_THETA_KI),
             SmartDashboard.getNumber("DynAlign/PID/Theta_D", DEFAULT_FIELD_THETA_KD)
         );

        // Get current robot pose estimate
        Pose2d currentPose = m_driveSubsystem.getPose();

        // Find primary visible reef tag
        m_currentTagId = -1; // Assume no valid target initially
        Pose2d targetRobotPoseField = null;
        boolean targetVisible = LimelightHelpers.getTV(m_limelightName);

        if (targetVisible) {
            int primaryID = (int) LimelightHelpers.getFiducialID(m_limelightName);
            // Check if the primary ID is one of the reef tags we have in our map
            if (AprilTagConstants.TAG_FIELD_POSES.containsKey(primaryID)) {
                m_currentTagId = primaryID; // Store the ID we're aiming for
                Pose2d tagPoseField = AprilTagConstants.TAG_FIELD_POSES.get(primaryID);

                // Calculate the desired robot pose on the field based on this tag
                targetRobotPoseField = tagPoseField.plus(new Transform2d(
                    m_robotOffsetRelativeToTag.getTranslation(),
                    m_robotOffsetRelativeToTag.getRotation()
                ));
            }
        }
        SmartDashboard.putNumber("DynAlign/TargetTagID", m_currentTagId); // Show which tag we're using (-1 if none)


        double vx_field = 0, vy_field = 0, omega_field = 0; // Field speeds
        m_currentErrorX = 0; m_currentErrorY = 0; m_currentErrorThetaRad = 0; // Reset errors

        // Calculate control outputs ONLY if we identified a valid reef tag target pose
        if (targetRobotPoseField != null) {
            // Update PID setpoints dynamically based on the current target tag
            m_xController.setSetpoint(targetRobotPoseField.getX());
            m_yController.setSetpoint(targetRobotPoseField.getY());
            m_thetaController.setSetpoint(targetRobotPoseField.getRotation().getRadians());

            // Calculate field-relative speeds using PID
            vx_field = m_xController.calculate(currentPose.getX()); // Use calculate(measurement) overload
            vy_field = m_yController.calculate(currentPose.getY());
            omega_field = m_thetaController.calculate(currentPose.getRotation().getRadians());

            // Clamp PID outputs
            vx_field = MathUtil.clamp(vx_field, -MAX_VELOCITY_METERS_PER_SEC, MAX_VELOCITY_METERS_PER_SEC);
            vy_field = MathUtil.clamp(vy_field, -MAX_VELOCITY_METERS_PER_SEC, MAX_VELOCITY_METERS_PER_SEC);
            omega_field = MathUtil.clamp(omega_field, -MAX_ANGULAR_VELOCITY_RAD_PER_SEC, MAX_ANGULAR_VELOCITY_RAD_PER_SEC);

            // Store current errors for dashboard and isFinished
            m_currentErrorX = m_xController.getPositionError();
            m_currentErrorY = m_yController.getPositionError();
            m_currentErrorThetaRad = m_thetaController.getPositionError();

        } // If targetRobotPoseField is null, speeds remain 0

        // Convert Field Speeds to Robot Speeds
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vx_field, vy_field, omega_field);
        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldSpeeds,
            currentPose.getRotation()
        );

        // Normalize and Command Drivetrain
        double norm_vx = robotSpeeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
        double norm_vy = robotSpeeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;
        double norm_omega = robotSpeeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed;
        norm_vx = MathUtil.clamp(norm_vx, -1.0, 1.0);
        norm_vy = MathUtil.clamp(norm_vy, -1.0, 1.0);
        norm_omega = MathUtil.clamp(norm_omega, -1.0, 1.0);
        m_driveSubsystem.drive(norm_vx, norm_vy, norm_omega, false);

        // Update SmartDashboard
        SmartDashboard.putNumber("DynAlign/CurrentX", currentPose.getX());
        SmartDashboard.putNumber("DynAlign/CurrentY", currentPose.getY());
        SmartDashboard.putNumber("DynAlign/CurrentTheta", currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("DynAlign/ErrorX", m_currentErrorX);
        SmartDashboard.putNumber("DynAlign/ErrorY", m_currentErrorY);
        SmartDashboard.putNumber("DynAlign/ErrorTheta", Math.toDegrees(m_currentErrorThetaRad));
    }

    @Override
    public boolean isFinished() {
        // Finish ONLY if we were actively targeting a tag in the last loop (m_currentTagId != -1)
        // AND the errors are within tolerance.
        // Using stored errors as setpoints might change during execute.
        boolean withinTolerance = m_currentTagId != -1 && // Must have been targeting a valid tag
                                  Math.abs(m_currentErrorX) <= FIELD_X_TOLERANCE_METERS &&
                                  Math.abs(m_currentErrorY) <= FIELD_Y_TOLERANCE_METERS &&
                                  Math.abs(m_currentErrorThetaRad) <= FIELD_THETA_TOLERANCE_RADIANS;

        SmartDashboard.putBoolean("DynAlign/IsFinished", withinTolerance);
        if (withinTolerance) {
            System.out.println("AlignToDynamicReefTagCommand Finished: Aligned.");
        }
        return withinTolerance;
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0, 0, 0, false); // Stop robot
        System.out.println("AlignToDynamicReefTagCommand Ended. Interrupted: " + interrupted);
        // Clean up dashboard
        SmartDashboard.putNumber("DynAlign/TargetTagID", -1);
        SmartDashboard.putNumber("DynAlign/ErrorX", 0.0);
        SmartDashboard.putNumber("DynAlign/ErrorY", 0.0);
        SmartDashboard.putNumber("DynAlign/ErrorTheta", 0.0);
        SmartDashboard.putBoolean("DynAlign/IsFinished", false);
    }
}