// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.*;

public class MoveToTarget extends CommandBase {
  
  private PIDController pid = new PIDController();
  private DriveSubsystem m_drive;
  
  /** Creates a new MoveToTarget. */
  public MoveToTarget(DriveSubsystem d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = d;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //pid.reset();    ?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(inst.GetEntry("robot_cam/object_found"))
      /* if see target{
          get error
          use pid controller to calculate speed correction 
          leftV = desired speed +correction;
          rightV = desired speed - correction;
          drive.setVelocities (LF, LB, RF, RB)
      }
      *
      */
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