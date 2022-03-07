// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.*;
import com.pathplanner.lib.PathPlannerTrajectory.*;
import frc.robot.PPMecanumControllerCommand;

public class DriveToPose extends CommandBase {

    private DriveSubsystem m_drive;
    private Pose2d targetPose;
    PPMecanumControllerCommand ppCommand ;

    /** Creates a new ReturnToTarmac. */
    public DriveToPose(Pose2d p, DriveSubsystem d) {


        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                d.getPose(),
                // List.of(new Translation2d(1, 1), new Translation2d(-2,1)),
                List.of(
                // new Translation2d(3, 0),
                // new Translation2d(3, 3),
                // new Translation2d(0, 3)
                ),
                p,
                config);

        targetPose = p;
        m_drive = d;

        //ppCommand = new PPMecanumControllerCommand(trajectory, pose, kinematics, xController, yController, thetaController, maxWheelVelocityMetersPerSecond, outputWheelSpeeds, requirements)
        
        addRequirements(d);


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // general idea: go to initial pose
        // have to create a path via code (not pathplanner) in order to get to said
        // initial pose

        // m_drive.getInitialPose();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}