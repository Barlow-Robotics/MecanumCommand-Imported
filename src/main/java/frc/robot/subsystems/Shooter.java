// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  WPI_TalonSRX shooterMotor;
  WPI_TalonSRX receiverMotor;

  public Shooter() {
      shooterMotor = new WPI_TalonSRX(Constants.ShooterConstants.ID_shooterMotor);
      receiverMotor = new WPI_TalonSRX(Constants.ShooterConstants.ID_receiverMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startShooting() {
    shooterMotor.set(TalonSRXControlMode.Velocity, Constants.ShooterConstants.shooterMotorVelocity);
    receiverMotor.set(TalonSRXControlMode.Velocity, Constants.ShooterConstants.receiverMotorVelocity);
  }

  public void stopShooting() {
    shooterMotor.set(TalonSRXControlMode.Velocity, 0);
    receiverMotor.set(TalonSRXControlMode.Velocity, 0);
  }

  public boolean hasStarted() {
      return(shooterMotor.get()!=0 && receiverMotor.get()!=0);
  }

  public boolean isStopped() {
      return(shooterMotor.get()==0 && receiverMotor.get()==0);
  }

  public void startReceiving() {
    receiverMotor.set(TalonSRXControlMode.Velocity, Constants.ShooterConstants.receiverMotorVelocity);
  }
  
  public void stopReceiving(){
    receiverMotor.set(TalonSRXControlMode.Velocity, 0);
  }

  private void setMotorConfig(WPI_TalonSRX motor) {
    motor.configFactoryDefault() ;
    motor.configSelectedFeedbackSensor(
        FeedbackDevice.QuadEncoder, 
        Constants.IntakeConstants.mainFeedbackLoop,
        Constants.IntakeConstants.encoderTimeout
        ); 
    motor.configClosedloopRamp(Constants.IntakeConstants.closedVoltageRampingConstant) ;
    motor.configOpenloopRamp(Constants.IntakeConstants.manualVoltageRampingConstant) ;
    motor.configNominalOutputForward(0);
    motor.configNominalOutputReverse(0);
    motor.configPeakOutputForward(1.0);
    motor.configPeakOutputReverse(-1.0);
    //motor.configMotionCruiseVelocity( (int) (Constants.IntakeConstants.unitsPerRotation * Constants.IntakeConstants.desiredRPMsForDrive));
    motor.config_kF(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.DrivetrainKf);
    motor.config_kP(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.DrivetrainkP);
    motor.config_kI(Constants.IntakeConstants.PID_id, 0);
    motor.config_kD(Constants.IntakeConstants.PID_id, 0);
    motor.setNeutralMode(NeutralMode.Brake);
  }

}

