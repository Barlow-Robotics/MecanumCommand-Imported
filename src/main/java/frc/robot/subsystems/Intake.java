// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.sim.PhysicsSim;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  WPI_TalonFX intakeMotor;
  // Solenoid extendSolenoid; // wpk move to index if they are not already there.
  // Solenoid retractSolenoid;
  // Compressor compressor;

  public Intake() {
    intakeMotor = new WPI_TalonFX(IntakeConstants.ID_IntakeMotor, "Intake Motor");
    // retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
    // Constants.IntakeConstants.Retract_Solenoid);
    // extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
    // Constants.IntakeConstants.Extend_Solenoid);
    // compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    setMotorConfig(intakeMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // public void extend() {
  // // solenoid extends
  // // extendSolenoid.set(true);
  // // retractSolenoid.set(false);
  // }

  // public void retract() {
  // // solenoid compresses
  // // extendSolenoid.set(false);
  // // retractSolenoid.set(true);
  // }

  public void start() {
    // motor for wheels starts running
    // intakeMotor.set(TalonFXControlMode.Velocity,
    // IntakeConstants.intakeMotorSpeed);
    intakeMotor.set(TalonFXControlMode.PercentOutput, Constants.IntakeConstants.IntakeMotorSpeed);
  }

  public void stop() {
    // motor for wheels stops running
    // intakeMotor.set(TalonFXControlMode.Velocity, 0);
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  // public boolean isExtended() {
  // return(extendSolenoid.get());
  // }

  public boolean isStarted() {
    return (intakeMotor.get() != 0);
  }

  private void setMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
    motor.configFactoryDefault();
    // motor.configSelectedFeedbackSensor(
    //     FeedbackDevice.QuadEncoder,
    //     Constants.DriveConstants.mainFeedbackLoop,
    //     Constants.DriveConstants.encoderTimeout);
    motor.configClosedloopRamp(Constants.DriveConstants.closedVoltageRampingConstant);
    motor.configOpenloopRamp(Constants.DriveConstants.manualVoltageRampingConstant);
    // motor.configNominalOutputForward(0);
    // motor.configNominalOutputReverse(0);
    // motor.configPeakOutputForward(1.0);
    // motor.configPeakOutputReverse(-1.0);
    // motor.configMotionCruiseVelocity( (int)
    // (Constants.DriveConstants.unitsPerRotation *
    // Constants.DriveConstants.desiredRPMsForDrive));
    motor.config_kF(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.kF);
    motor.config_kP(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.kP);
    motor.config_kI(Constants.IntakeConstants.PID_id, 0);
    motor.config_kD(Constants.IntakeConstants.PID_id, 0);
    motor.setNeutralMode(NeutralMode.Brake);
  }




  boolean simulationInitialized = false ;

  public void simulationInit() {
    //PhysicsSim.getInstance().addTalonFX(intakeMotor, 0.5, 6800);
  }


  @Override
  public void simulationPeriodic() {
    if (! simulationInitialized) {
      simulationInit();
      simulationInitialized = true ;
    }

    // do sim stuff



  }
}