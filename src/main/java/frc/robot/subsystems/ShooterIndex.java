// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIndex extends SubsystemBase {
  /** Creates a new Shooter. */

  WPI_TalonFX beltMotor;
  WPI_TalonFX flyWheelMotor;

  public ShooterIndex() {
      beltMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_shooterMotor);
      flyWheelMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_flyWheelMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startShooting() {
    beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.shooterMotorVelocity);
    flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.flyWheelMotorVelocity);
  }

  public void stopShooting() {
    beltMotor.set(TalonFXControlMode.Velocity, 0.0);
    flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
  }

  public boolean hasStarted() {
      return(beltMotor.get()!=0.0 && flyWheelMotor.get()!=0.0
      );
  }

  public boolean isStopped() {
      return(beltMotor.get()==0 && flyWheelMotor.get()==0
      );
  }

  public void startReceiving() {
    beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.receiverMotorVelocity);
  }
  
  public void stopReceiving(){
    beltMotor.set(TalonFXControlMode.Velocity, 0);
  }

  private void setMotorConfig(WPI_TalonFX motor) {
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

