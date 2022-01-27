// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//No copyright? Nothing was in here when I opened it


package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmBar;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;

public class Climb extends CommandBase {
  
  private ArmBar m_armBar;
  private DriveSubsystem m_drive;

  enum ArmCommandState {
    WaitingForArmToBeStraightUp,
    DrivingBackward,
    MovingToHighBar,
    MovingToTraversalBar,
    OnTraversalBar,
    Finished
  };

  ArmCommandState currentState = ArmCommandState.WaitingForArmToBeStraightUp;

  /** Creates a new Climb. */ 
  public Climb(ArmBar a) {
    // Use addRequirements() here to declare subsystem dependencies. 
    m_armBar = a;
    addRequirements(m_armBar);
    //what value to give "m_drive"?
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    switch (currentState) {
      case WaitingForArmToBeStraightUp:
        if(Math.abs(m_armBar.getArmAngle() - ArmBarConstants.firstRotationAngle) > ArmBarConstants.firstRotationAngleTolerance) {
          m_armBar.rotateGripperArmDegree(ArmBarConstants.firstRotationAngle);
        } 
        else {
          //need to stop motor?
          currentState = ArmCommandState.DrivingBackward;
        }
        break;
      
      case DrivingBackward:
        if(m_armBar.gripperAIsClosed()) {
          m_drive.drive(0.0, 0.0, 0, false); //how incorporate drivetrain?
          currentState = ArmCommandState.MovingToHighBar;
        }
        else {
          m_drive.drive(-0.1, 0.0, 0, false);
        }
        break;

      case MovingToHighBar:  
        if(!m_armBar.gripperBIsClosed()){
          m_armBar.rotateGripperArmDegree(ArmBarConstants.consistentRotationAngle);
        }
        else{
            m_armBar.releaseGripperA();
            currentState = ArmCommandState.MovingToTraversalBar;
        }
        break;
      
      case MovingToTraversalBar:
        if(!m_armBar.gripperAIsClosed()) {
          m_armBar.rotateGripperArmDegree(ArmBarConstants.consistentRotationAngle);
          //maybe swing
        }
        else {
          m_armBar.releaseGripperB();
          currentState = ArmCommandState.OnTraversalBar;
        }
        break;

      case OnTraversalBar:
        //do we let go or do we rotate it down using the member function?
        if(m_armBar.gripperAIsClosed() && !m_armBar.gripperBIsClosed() && m_armBar.armBarMotor.get()==0.0) {
            currentState = ArmCommandState.Finished;
          }
        break;

      case Finished:
        //Bot is immobile - nothing else needs to go here
        break;
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