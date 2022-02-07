// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIndex extends SubsystemBase {
  /** Creates a new Shooter. */

  WPI_TalonFX beltMotor;
  WPI_TalonFX flyWheelMotor;
  // Solenoid extendSolenoid; 
  // Solenoid retractSolenoid;
  // Compressor compressor;

  public ShooterIndex() {
      beltMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_ShooterMotor);
      flyWheelMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_FlyWheelMotor);
      // retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.Retract_Solenoid);
      // extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.Extend_Solenoid);
      // compressor = new Compressor(PneumaticsModuleType.CTREPCM);

      setMotorConfig(beltMotor);
      setMotorConfig(flyWheelMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startShooting() {
    if(Math.abs(flyWheelMotor.get() - ShooterConstants.FlyWheelMotorShootingVelocity) > ShooterConstants.FlyWheelShootingTolerance) {
      beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.BeltMotorShootingVelocity);
    } 
    else {
      flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorShootingVelocity);
    }
  }

  public void stopShooting() {
    beltMotor.set(TalonFXControlMode.Velocity, 0.0);
    flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
  }

  // public void extend() {
  // //solenoid extends
  // extendSolenoid.set(true);
  // retractSolenoid.set(false);
  // }

  // public void retract() {
  // //solenoid compresses
  // extendSolenoid.set(false);
  // retractSolenoid.set(true);
  // }

  public boolean hasStarted() {
      return(beltMotor.get()!=0.0 && flyWheelMotor.get()!=0.0
      );
  }

  public boolean isStopped() {
      return(beltMotor.get()==0 && flyWheelMotor.get()==0
      );
  }

  // public boolean isExtended() {
  //   return(extendSolenoid.get());
  // }

  public void startReceiving() {
    beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.BeltMotorIntakeVelocity);
    flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorIntakeVelocity);
  }
  
  public void stopReceiving(){
    beltMotor.set(TalonFXControlMode.Velocity, 0.0);
    flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
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

