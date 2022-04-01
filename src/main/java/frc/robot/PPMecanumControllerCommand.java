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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
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

    private ArrayList<Translation2d> m_cargoLocations ;
    private Vision m_vision ;
    private PIDController m_cargoPID ;

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
            ArrayList<Translation2d> cargo ,
            DriveSubsystem drive ,
            Vision v ) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;

        m_controller = new HolonomicDriveController(xController, yController, thetaController);
        m_maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;
        m_outputWheelSpeeds = outputWheelSpeeds;
        m_cargoLocations = cargo ;
        m_vision = v ;
        m_cargoPID = new PIDController(0.005, 0.0, 0.0001) ;

        addRequirements(drive);
        addRequirements(v);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }


    private Translation2d getNearbyCargoLocation(Translation2d currentPosition, double withinRadius) {
        Translation2d result = null ;
        double closestDistance = Double.MAX_VALUE ;

        for ( var l : m_cargoLocations) {
            double d = currentPosition.getDistance(l) ;
            if ( d <= withinRadius && d < closestDistance) {
                result = l ;
                closestDistance = d ;
            }
        }
        return result ;
    }

    boolean trackingCargo = false ;
    int missedCargoFrames = 0 ;


    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        Pose2d currentPose = m_pose.get() ;

        double cargoDistanceFromCenter = 0.0 ;
        Translation2d nearestCargoLocation = getNearbyCargoLocation(currentPose.getTranslation(), 1.5) ;
        if (nearestCargoLocation != null) {
            if (m_vision.cargoIsVisible()) {
                if (!trackingCargo) {
                    m_cargoPID.reset();
                }
                trackingCargo = true ;
                missedCargoFrames = 0 ;
                cargoDistanceFromCenter = m_vision.cargoDistanceFromCenter() ;
                // need to compute velocity adjustment

            } else {
                if ( trackingCargo) {
                    missedCargoFrames++ ;
                    if ( missedCargoFrames > 5) {
                        trackingCargo = false ;
                    }
                }
            }
        } else {
            trackingCargo = false ;
            missedCargoFrames = 0 ;
        }

        ChassisSpeeds targetChassisSpeeds ;
        MecanumDriveWheelSpeeds targetWheelSpeeds ;
        if ( trackingCargo) {
            double velocity = desiredState.velocityMetersPerSecond ;
            double adjustment = m_cargoPID.calculate(cargoDistanceFromCenter) ;
            double ls = velocity - adjustment ;
            double rs = velocity + adjustment ;
            targetWheelSpeeds = new MecanumDriveWheelSpeeds(ls, rs, ls, rs ) ;
        } else {
            targetChassisSpeeds = m_controller.calculate(currentPose, desiredState, desiredState.holonomicRotation);
            targetWheelSpeeds = m_kinematics.toWheelSpeeds(targetChassisSpeeds);
        }

        //var targetChassisSpeeds = m_controller.calculate(currentPose, desiredState, desiredState.holonomicRotation);

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
