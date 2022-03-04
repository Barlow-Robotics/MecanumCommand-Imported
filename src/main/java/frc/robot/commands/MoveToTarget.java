// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.*;

public class MoveToTarget extends CommandBase {
  
  private PIDController pid = new PIDController(0.01, 0, 0);
  private DriveSubsystem m_drive;

  private double error;
  private double leftVelocity;
  private double rightVelocity;
  private int missedFrames = 0;
  
  /** Creates a new MoveToTarget. */
  public MoveToTarget(DriveSubsystem d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = d;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.reset();
    missedFrames = 0 ;
 }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(NetworkTableInstance.getDefault().getEntry("robot_cam/object_found").getBoolean(false)){
        error = NetworkTableInstance.getDefault().getEntry("robot_cam/distance_from_center").getDouble(0);
        leftVelocity = 1.0 - pid.calculate(error);
        rightVelocity = 1.0 + pid.calculate(error);

        m_drive.setWheelSpeeds(
            new MecanumDriveWheelSpeeds(
              leftVelocity, rightVelocity, leftVelocity, rightVelocity)
            );
    } else{
      missedFrames++;
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setWheelSpeeds(
      new MecanumDriveWheelSpeeds(
        0, 0, 0, 0)
    );
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (missedFrames > 10){
      return true;
    }
    else{
      return false;
    }
  }
}