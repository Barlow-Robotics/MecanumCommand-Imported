// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.DigitalOutput;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
//import sensors


public class ArmBar extends SubsystemBase {

  WPI_TalonSRX armBarMotor;
  //Add sensors

  public DigitalInput hallEffectsA1;
  public DigitalInput hallEffectsA2;
  public DigitalInput hallEffectsB1;
  public DigitalInput hallEffectsB2;

  /** Creates a new ArmBar. */
  public ArmBar() {
    armBarMotor = new WPI_TalonSRX(ArmBarConstants.ID_armBarMotor);
    armBarMotor.setSelectedSensorPosition(0.0);
    armBarMotor.set(TalonSRXControlMode.Velocity, ArmBarConstants.armBarMotorSpeed);

    hallEffectsA1 = new DigitalInput(Constants.ArmBarConstants.ID_hallEffectsA1);
    hallEffectsA2 = new DigitalInput(Constants.ArmBarConstants.ID_hallEffectsA2);
    hallEffectsB1 = new DigitalInput(Constants.ArmBarConstants.ID_hallEffectsB1);
    hallEffectsB2 = new DigitalInput(Constants.ArmBarConstants.ID_hallEffectsB2);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void rotateGripperArmDegree(double angle){
    //motor will go until the bar is rotated so that the the original bar position and the new bar position form the desired angle
    armBarMotor.set(TalonSRXControlMode.Position, (angle/360)*Constants.ArmBarConstants.gearRatio*Constants.ArmBarConstants.unitsPerRotation);
    //armBarMotor.setSelectedSensorPosition(0.0);
  }

  public void rotateGripperArmA(){
    //Begin rotating gripper arms until the A grippers registers as closed
    while(!gripperAIsClosed()){
      //rotate motor
    }
  }

  public void rotateGripperArmB(){
    //Begin rotating gripper arms until the A grippers registers as closed
    while(!gripperBIsClosed()){
      //rotate motor
    }
  }

  public void releaseGripperA(){
      //Pull pin on both A's
  }

  public void releaseGripperB(){
      //Pull pin on both B's
  }

  public boolean gripperAIsClosed(){
    //Check for halleffect on both A's
    return(hallEffectsA1.get() && hallEffectsA2.get());
  }

  public boolean gripperBIsClosed(){
    //Check for halleffect on both B's
    return(hallEffectsB1.get() && hallEffectsB2.get());
  }


  private void setMotorConfig(WPI_TalonSRX motor) {
    motor.configFactoryDefault() ;
    motor.configSelectedFeedbackSensor(
        FeedbackDevice.QuadEncoder, 
        Constants.ArmBarConstants.mainFeedbackLoop,
        Constants.ArmBarConstants.encoderTimeout
        ); 
    motor.configClosedloopRamp(Constants.ArmBarConstants.closedVoltageRampingConstant) ;
    motor.configOpenloopRamp(Constants.ArmBarConstants.manualVoltageRampingConstant) ;
    motor.configNominalOutputForward(0);
    motor.configNominalOutputReverse(0);
    motor.configPeakOutputForward(1.0);
    motor.configPeakOutputReverse(-1.0);
    //motor.configMotionCruiseVelocity( (int) (Constants.DriveConstants.unitsPerRotation * Constants.DriveConstants.desiredRPMsForDrive));
    motor.config_kF(Constants.ArmBarConstants.PID_id, Constants.ArmBarConstants.DrivetrainKf);
    motor.config_kP(Constants.ArmBarConstants.PID_id, Constants.ArmBarConstants.DrivetrainkP);
    motor.config_kI(Constants.ArmBarConstants.PID_id, 0);
    motor.config_kD(Constants.ArmBarConstants.PID_id, 0);
    motor.setNeutralMode(NeutralMode.Brake);
  }
}
