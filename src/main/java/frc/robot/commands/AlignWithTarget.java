// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class AlignWithTarget extends CommandBase {

    private PIDController pid = new PIDController(0.005, 0, 0);
    private DriveSubsystem m_drive;
    private Vision m_vision;

    private double error;
    private boolean alignmentComplete = false;

    /** Creates a new AlignRobotToVision. */
    public AlignWithTarget(Vision v, DriveSubsystem d) {
        m_vision = v;
        m_drive = d;
        addRequirements(m_vision);
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid.reset();
        alignmentComplete = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        if(m_vision.visionTargetIsVisible()) {
            System.out.println("Target is visible");
        } else { 
            System.out.println("Target is not visible");
        }

        double leftVelocity = 0.0;
        double rightVelocity = 0.0;

        if (m_vision.visionTargetIsVisible()) {
            error = m_vision.visionTargetDistanceFromCenter();
            if (Math.abs(error) < Constants.VisionConstants.AlignmentTolerence) {
                alignmentComplete = true;
            } else {
                double adjustment = pid.calculate(error);
                adjustment = Math.signum(adjustment)
                        * Math.min(Math.abs(adjustment), Constants.DriveConstants.CorrectionRotationSpeed / 4.0);
                leftVelocity =  adjustment;
                rightVelocity = -adjustment;
            }
            m_drive.setWheelSpeeds(
                    new MecanumDriveWheelSpeeds(
                            leftVelocity, rightVelocity, leftVelocity, rightVelocity));
        } else {
            alignmentComplete = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.setWheelSpeeds(
                new MecanumDriveWheelSpeeds(
                        0, 0, 0, 0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return alignmentComplete;
    }
}