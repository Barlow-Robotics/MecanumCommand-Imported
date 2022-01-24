// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//No copyright? Nothing was in here when I opened it


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmBar;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;;

public class Climb extends CommandBase {
  
  private ArmBar m_armBar;

  /** Creates a new Climb. */ 
  public Climb(ArmBar a) {
    // Use addRequirements() here to declare subsystem dependencies. 
    m_armBar = a;
    addRequirements(m_armBar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armBar.rotateGripperArmDegree(Constants.ArmBarConstants.firstRotationAngle);
    //m_armBar.drive(0.0, 0.75, 0, false); how do i get the drive subsystem involved so i can get it to drive backwards until we get to the mid rung? do I have to create another Drive here?
    m_armBar.rotateGripperArmDegree(Constants.ArmBarConstants.consistentRotationAngle);
    m_armBar.releaseGripperA();
    m_armBar.rotateGripperArmDegree(Constants.ArmBarConstants.consistentRotationAngle);
    m_armBar.releaseGripperB();
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

