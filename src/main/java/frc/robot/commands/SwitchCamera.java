// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.networktables.*;

public class SwitchCamera extends CommandBase {
  /** Creates a new SwitchCamera. */
  private double cameraNumber;

  public SwitchCamera() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cameraNumber = NetworkTableInstance.getDefault().getEntry("robot_cam/camera_number").getDouble(0);

    if (cameraNumber == 0){
      NetworkTableInstance.getDefault().getEntry("robot_cam/camera_number").setDouble(1);
    }
    else {
      NetworkTableInstance.getDefault().getEntry("robot_cam/camera_number").setDouble(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
