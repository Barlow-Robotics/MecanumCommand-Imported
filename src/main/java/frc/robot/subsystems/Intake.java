// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;



public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  WPI_TalonSRX intakeMotor;
  Solenoid extendSolenoid;
  Solenoid retractSolenoid;
  Compressor compressor; 


  public Intake() {
    intakeMotor = new WPI_TalonSRX(IntakeConstants.ID_intakeMotor);
    retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.Extend_Solenoid);
    extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.Retract_Solenoid);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
    // solenoid extends
    extendSolenoid.set(true);
    retractSolenoid.set(false);
  }

  public void retract() {
    // solenoid compresses
    extendSolenoid.set(false);
    retractSolenoid.set(true);
  }

  public void start() {
    // motor for wheels starts running
//    intakeMotor.set(TalonSRXControlMode.Velocity, IntakeConstants.intakeMotorSpeed);
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.5);
  }

  public void stop() {
    // motor for wheels stops running
//    intakeMotor.set(TalonSRXControlMode.Velocity, 0);
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }


  public boolean isExtended() {
    return(extendSolenoid.get());
  }

  public boolean isStarted() {
    return(intakeMotor.get()!=0);
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
