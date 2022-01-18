// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmBar extends SubsystemBase {
  /** Creates a new ArmBar. */
  public ArmBar() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rotateGripperArm(){
    //Begin rotating gripper arm until a gripper registers as closed
  }

  public void releaseGripperA(){
      //Pull pin on A
  }

  public void releaseGripperB(){
      //Pull pin on B
  }

  public boolean gripperAIsClosed(){
    //Check for halleffect on A
    return(false);
  }

  public boolean gripperBIsClosed(){
    //Check for halleffect on B
    return(false);
  }
}
