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
    Idle, 
    WaitingForArmToBeStraightUp,
    DrivingBackward,
    WaitingForHighBar,
    WaitingForTraversalBar,
    Finished
  };

  ArmCommandState currentState = ArmCommandState.Idle ;

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

   if(currentState == ArmCommandState.WaitingForArmToBeStraightUp) {
      if(m_armBar.armAngle() == ArmBarConstants.firstRotationAngle){
        currentState = ArmCommandState.DrivingBackward;
      }
      else if (currentState == ArmCommandState.DrivingBackward) {
        m_drive.drive(-0.1, 0.0, 0, false); //how incorporate drivetrain?
        if(m_armBar.gripperAIsClosed() && m_armBar.gripperBIsClosed()) {
          m_drive.drive(0.0, 0.0, 0, false); //how incorporate drivetrain?
          m_armBar.armBarMotor.set(TalonSRXControlMode.Velocity, 0);
          currentState = ArmCommandState.WaitingForHighBar;
        }
      } 
      else if (currentState == ArmCommandState.WaitingForHighBar) {
        if(m_armBar.gripperAIsClosed() && m_armBar.gripperBIsClosed()) {
          //let go of first bar
        }
        else if(!m_armBar.gripperAIsClosed() && !m_armBar.gripperBIsClosed()) {
          currentState = ArmCommandState.WaitingForTraversalBar;
        }
      }
      else if (currentState == ArmCommandState.WaitingForTraversalBar) {
        currentState = ArmCommandState.Finished;
      }
    }

    
    switch (currentState) {
      case WaitingForArmToBeStraightUp:
        //code for here
        break;

      case DrivingBackward:
        //code for here
        break;

      case WaitingForHighBar:
        //code for here
        break;
      
      case WaitingForTraversalBar:
        //code for here
        break;

      case Finished:
        //code for here
        break;
    }

    m_armBar.rotateGripperArmDegree(ArmBarConstants.firstRotationAngle);   
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

