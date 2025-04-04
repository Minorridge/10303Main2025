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

    // --- Dependencies and Controllers ---
    private final DriveSubsystem m_robotDrive;
    private final PIDController distanceController; // Controls forward/backward (Z-axis in target space)
    private final PIDController strafeController;   // Controls left/right (X-axis in target space)
    private final PIDController rotationController; // Controls rotation (Yaw-axis in target space)

    // --- Target Values & Tolerances (Mostly Unchanged) ---
    // TODO: Tune TARGET_DISTANCE_METERS based on your robot and goal
    private final double TARGET_DISTANCE_METERS = 0.2413;
    private final double TARGET_STRAFE_METERS = 0.0;   // Target horizontal centering
    private final double TARGET_ROTATION_DEG = 0.0;    // Target parallel alignment

    // TODO: Tune these tolerances
    private final double DISTANCE_TOLERANCE_METERS = 0.05;
    private final double STRAFE_TOLERANCE_METERS = 0.05;
    private final double ROTATION_TOLERANCE_DEG = 2.0;

    // --- Timers & Timing Constants (Unchanged) ---
    private final Timer timerAtGoal = new Timer();
    private final Timer timerLostTarget = new Timer();
    private final double TIME_AT_GOAL_SECONDS = 0.3;
    private final double LOST_TARGET_TIMEOUT_SECONDS = 0.75;

    // --- State Variables ---
    private boolean hasSeenValidTargetOnce = false; // Renamed for clarity

    // --- Valid Tag ID Range ---
    private final int MIN_VALID_TAG_ID = 6;
    private final int MAX_VALID_TAG_ID = 22;


    /**
     * Constructor: Aligns to the first valid AprilTag (ID 6-22) seen by the Limelight.
     *
     * @param robotDrive The DriveSubsystem dependency.
     */
    public AutoAlign(DriveSubsystem robotDrive) {
        m_robotDrive = robotDrive;

        // TODO: Tune these PID constants (P, I, D) vigorously!
        distanceController = new PIDController(2.0, 0.0, 0.05);
        distanceController.setSetpoint(TARGET_DISTANCE_METERS);
        distanceController.setTolerance(DISTANCE_TOLERANCE_METERS);

        strafeController = new PIDController(2.0, 0.0, 0.05);
        strafeController.setSetpoint(TARGET_STRAFE_METERS);
        strafeController.setTolerance(STRAFE_TOLERANCE_METERS);

        rotationController = new PIDController(0.05, 0.0, 0.001);
        rotationController.setSetpoint(TARGET_ROTATION_DEG);
        rotationController.setTolerance(ROTATION_TOLERANCE_DEG);

        addRequirements(m_robotDrive);
    }


    @Override
    public void initialize() {
        System.out.println("Starting AutoAlign Command - Will align to first valid tag (ID "
                + MIN_VALID_TAG_ID + "-" + MAX_VALID_TAG_ID + ") seen.");
        hasSeenValidTargetOnce = false; // Reset flag
        timerLostTarget.reset();
        timerLostTarget.start();
        timerAtGoal.reset();
        timerAtGoal.stop();

        // Reset PIDs
        distanceController.reset();
        strafeController.reset();
        rotationController.reset();
    }

    @Override
    public void execute() {
        boolean targetVisible = LimelightHelpers.getTV(""); // Check if *any* tag is visible
        double currentTagID = LimelightHelpers.getFiducialID(""); // Get ID of primary tag

        // Check if a tag is visible AND if its ID is within the valid range [6, 22]
        boolean isValidTargetVisible = targetVisible
                && currentTagID >= MIN_VALID_TAG_ID
                && currentTagID <= MAX_VALID_TAG_ID;

        if (isValidTargetVisible) {
            // We see a tag within the desired range
            hasSeenValidTargetOnce = true; // Mark that we've seen a valid target
            timerLostTarget.reset(); // Reset the lost target timer

            // Get the robot's pose relative to the currently tracked valid tag
            double[] botposeTargetSpace = LimelightHelpers.getBotPose_TargetSpace("");

            // --- Calculate PID Outputs ---
            // ** THESE SIGNS ARE CRITICAL - ADJUST BASED ON TESTING **

            double currentDistance = botposeTargetSpace[2];
            // *** FIX 1: Flipping sign for xSpeed to correct driving away/towards ***
            double xSpeed = distanceController.calculate(currentDistance); // Removed negative sign
            // If robot *still* drives away, change back to -distanceController.calculate(...)
            // and verify DriveSubsystem.drive() and Limelight Z-axis direction.

            double currentStrafe = botposeTargetSpace[0];
            // *** FIX 2 (Try if needed): Adjust sign for ySpeed if strafing is reversed ***
            double ySpeed = strafeController.calculate(currentStrafe);
            // If robot strafes LEFT when it should go RIGHT, change to:
            // double ySpeed = -strafeController.calculate(currentStrafe);

            double currentRotation = botposeTargetSpace[4];
            // *** FIX 3 (Try if needed): Adjust sign for rotSpeed if rotation is reversed ***
            double rotSpeed = -rotationController.calculate(currentRotation);
            // If robot rotates CW when it should go CCW, change to:
            // double rotSpeed = rotationController.calculate(currentRotation);


            // --- Limit Speeds (Same as before) ---
            double maxDriveSpeed = 0.2;
            double maxRotationSpeed = 0.1;
            xSpeed = Math.max(-maxDriveSpeed, Math.min(maxDriveSpeed, xSpeed));
            ySpeed = Math.max(-maxDriveSpeed, Math.min(maxDriveSpeed, ySpeed));
            rotSpeed = Math.max(-maxRotationSpeed, Math.min(maxRotationSpeed, rotSpeed));

            // --- Drive the Robot (Same as before) ---
            m_robotDrive.drive(xSpeed, ySpeed, rotSpeed, false);

            // --- Check if at Goal (Same as before) ---
            if (distanceController.atSetpoint() && strafeController.atSetpoint() && rotationController.atSetpoint()) {
                if (timerAtGoal.get() == 0) {
                    timerAtGoal.reset();
                    timerAtGoal.start();
                }
            } else {
                timerAtGoal.stop();
                timerAtGoal.reset();
            }

            // --- SmartDashboard Logging ---
            SmartDashboard.putNumber("AutoAlign/CurrentValidTagID", currentTagID);
            SmartDashboard.putBoolean("AutoAlign/ValidTargetVisible", true);
            // Add Raw Pose Data for debugging axis directions:
            SmartDashboard.putNumber("AutoAlign/RawPose_X (Strafe)", botposeTargetSpace[0]);
            SmartDashboard.putNumber("AutoAlign/RawPose_Z (Distance)", botposeTargetSpace[2]);
            SmartDashboard.putNumber("AutoAlign/RawPose_Yaw (Rotation)", botposeTargetSpace[4]);
            // PID Inputs/Outputs:
            SmartDashboard.putNumber("AutoAlign/CurrentDistance(Z)", currentDistance);
            SmartDashboard.putNumber("AutoAlign/CurrentStrafe(X)", currentStrafe);
            SmartDashboard.putNumber("AutoAlign/CurrentRotation(Yaw)", currentRotation);
            SmartDashboard.putNumber("AutoAlign/PID_XSpeed", xSpeed);
            SmartDashboard.putNumber("AutoAlign/PID_YSpeed", ySpeed);
            SmartDashboard.putNumber("AutoAlign/PID_RotSpeed", rotSpeed);
            // State Info:
            SmartDashboard.putBoolean("AutoAlign/AtDistanceSetpoint", distanceController.atSetpoint());
            SmartDashboard.putBoolean("AutoAlign/AtStrafeSetpoint", strafeController.atSetpoint());
            SmartDashboard.putBoolean("AutoAlign/AtRotationSetpoint", rotationController.atSetpoint());
            SmartDashboard.putNumber("AutoAlign/TimeAtGoal", timerAtGoal.get());

        } else {
            // No valid target visible (either no tag, or tag ID outside 6-22 range)
            SmartDashboard.putBoolean("AutoAlign/ValidTargetVisible", false);
            if (targetVisible) { // Log the invalid tag ID if one is visible but out of range
                 SmartDashboard.putNumber("AutoAlign/CurrentInvalidTagID", currentTagID);
            } else {
                 SmartDashboard.putNumber("AutoAlign/CurrentInvalidTagID", -1); // Indicate no tag visible at all
            }

            // Stop the robot
            m_robotDrive.drive(0, 0, 0, false);

            // Stop and reset the at-goal timer
            timerAtGoal.stop();
            timerAtGoal.reset();

            // Let timerLostTarget run if we previously saw a valid target
        }
        SmartDashboard.putNumber("AutoAlign/TimeSinceValidTargetLost", timerLostTarget.get());
    }

    @Override
    public boolean isFinished() {
        // Finish successfully if we have been at the setpoint for the required time
        boolean finishedAtGoal = timerAtGoal.hasElapsed(TIME_AT_GOAL_SECONDS);

        // Finish unsuccessfully if we saw a *valid* target initially but then lost *any* valid target for too long
        boolean finishedLostTarget = hasSeenValidTargetOnce && timerLostTarget.hasElapsed(LOST_TARGET_TIMEOUT_SECONDS);

        if (finishedAtGoal) {
             System.out.println("AutoAlign finished: Reached goal pose.");
             return true;
        }
        if (finishedLostTarget) {
            System.out.println("AutoAlign finished: Valid target lost for too long.");
             return true;
        }

        return false; // Keep running otherwise
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.drive(0, 0, 0, false); // Stop the robot
        timerAtGoal.stop();
        timerLostTarget.stop();
        System.out.println("AutoAlign Command ended. Interrupted: " + interrupted);
    }
}