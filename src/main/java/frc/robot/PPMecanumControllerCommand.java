// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

//import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.networktables.*;


/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory
 * {@link PathPlannerTrajectory} with a
 * mecanum drive.
 */
public class PPMecanumControllerCommand extends CommandBase {
    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final MecanumDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final double m_maxWheelVelocityMetersPerSecond;
    private final Consumer<MecanumDriveWheelSpeeds> m_outputWheelSpeeds;

    /**
     * Constructs a new PPMecanumControllerCommand that when executed will follow
     * the provided
     * trajectory. The user should implement a velocity PID on the desired output
     * wheel velocities.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path -
     * this is left to the user, since it is not appropriate for paths with
     * non-stationary end-states.
     *
     * @param trajectory                      The Pathplanner trajectory to follow.
     * @param pose                            A function that supplies the robot
     *                                        pose - use one of the odometry classes
     *                                        to
     *                                        provide this.
     * @param kinematics                      The kinematics for the robot
     *                                        drivetrain.
     * @param xController                     The Trajectory Tracker PID controller
     *                                        for the robot's x position.
     * @param yController                     The Trajectory Tracker PID controller
     *                                        for the robot's y position.
     * @param thetaController                 The Trajectory Tracker PID controller
     *                                        for angle for the robot.
     * @param maxWheelVelocityMetersPerSecond The maximum velocity of a drivetrain
     *                                        wheel.
     * @param outputWheelSpeeds               A MecanumDriveWheelSpeeds object
     *                                        containing the output wheel speeds.
     * @param requirements                    The subsystems to require.
     */
    public PPMecanumControllerCommand(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> pose,
            MecanumDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            double maxWheelVelocityMetersPerSecond,
            Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
            Subsystem... requirements) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;

        m_controller = new HolonomicDriveController(xController, yController, thetaController);
        //m_controller.setTolerance(new Pose2d(0.5, 0.5, new Rotation2d(2.0 * (Math.PI / 180.0))));

        m_maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;

        m_outputWheelSpeeds = outputWheelSpeeds;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }




    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);

        NetworkTableInstance.getDefault().getEntry("trajectory/saturated_wheel_speeds_front_left").setDouble(targetWheelSpeeds.frontLeftMetersPerSecond);
        NetworkTableInstance.getDefault().getEntry("trajectory/saturated_wheel_speeds_front_right").setDouble(targetWheelSpeeds.frontRightMetersPerSecond);
        NetworkTableInstance.getDefault().getEntry("trajectory/saturated_wheel_speeds_back_left").setDouble(targetWheelSpeeds.rearLeftMetersPerSecond);
        NetworkTableInstance.getDefault().getEntry("trajectory/saturated_wheel_speeds_back_right").setDouble(targetWheelSpeeds.rearRightMetersPerSecond);

        targetWheelSpeeds.desaturate(m_maxWheelVelocityMetersPerSecond);

        NetworkTableInstance.getDefault().getEntry("trajectory/desaturated_wheel_speeds_front_left").setDouble(targetWheelSpeeds.frontLeftMetersPerSecond);
        NetworkTableInstance.getDefault().getEntry("trajectory/desaturated_wheel_speeds_front_right").setDouble(targetWheelSpeeds.frontRightMetersPerSecond);
        NetworkTableInstance.getDefault().getEntry("trajectory/desaturated_wheel_speeds_back_left").setDouble(targetWheelSpeeds.rearLeftMetersPerSecond);
        NetworkTableInstance.getDefault().getEntry("trajectory/desaturated_wheel_speeds_back_right").setDouble(targetWheelSpeeds.rearRightMetersPerSecond);

        var cp = m_pose.get() ;
        NetworkTableInstance.getDefault().getEntry("trajectory/currentPose_X").setDouble(cp.getX());
        NetworkTableInstance.getDefault().getEntry("trajectory/currentPose_Y").setDouble(cp.getY());
        NetworkTableInstance.getDefault().getEntry("trajectory/currentPose_heading").setDouble(cp.getRotation().getDegrees());
        NetworkTableInstance.getDefault().getEntry("trajectory/desiredState_X").setDouble(desiredState.poseMeters.getX());
        NetworkTableInstance.getDefault().getEntry("trajectory/desiredState_Y").setDouble(desiredState.poseMeters.getY());
        NetworkTableInstance.getDefault().getEntry("trajectory/desiredState_holonomic_rot").setDouble(desiredState.holonomicRotation.getDegrees());
        NetworkTableInstance.getDefault().getEntry("trajectory/desiredState_velocity").setDouble(desiredState.velocityMetersPerSecond);

        m_outputWheelSpeeds.accept(targetWheelSpeeds);
    }




    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }




    @Override
    public boolean isFinished() {
        // if (m_controller.atReference()) {
        //     System.out.println("Path is at the reference point");
        // }
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
        // if ( m_controller.atReference()) {
        //     return true ;
        // } else {
        //     return false ;
        // }
    }
}
