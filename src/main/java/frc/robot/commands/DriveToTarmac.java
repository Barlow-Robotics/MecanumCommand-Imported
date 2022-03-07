// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Might be helpful: https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/creating-following-trajectory.html

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import frc.robot.RobotContainer;

public class DriveToTarmac extends CommandBase {

  private DriveSubsystem m_drive;
  Pose2d targetPose ;

  /** Creates a new ReturnToTarmac. */
  public DriveToTarmac(Pose2d p, DriveSubsystem d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = d;
    this.targetPose = p ;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    Trajectory tarmacTrajectory =
        TrajectoryGenerator.generateTrajectory(
            m_drive.getPose() ,
            // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            List.of(),
            targetPose,
            config);
            }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

    // general idea: go to initial pose 
    // have to create a path via code (not pathplanner) in order to get to said initial pose 

    // m_drive.getInitialPose();
    //trajectory.getInitialPose();
  // Called once the command ends or is interrupted.

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}