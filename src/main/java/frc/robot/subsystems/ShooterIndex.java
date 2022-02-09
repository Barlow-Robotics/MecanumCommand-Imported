// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.sim.PhysicsSim; 
import edu.wpi.first.networktables.*;



public class ShooterIndex extends SubsystemBase {
  /** Creates a new Shooter. */

  WPI_TalonFX beltMotor;
  WPI_TalonFX flyWheelMotor;

  boolean beltStarted = false ;
  boolean isShooting = false ;
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

      flyWheelMotor.setInverted(TalonFXInvertType.Clockwise);
  }

  @Override
  public void periodic() {
    report();

    // This method will be called once per scheduler run
    // System.out.println("FlyWheel: " + flyWheelMotor.get());
    // System.out.println("BeltMotor: " + beltMotor.get());
    // if(ShooterConstants.FlyWheelMotorShootingVelocity -
    // flyWheelMotor.getSelectedSensorVelocity() <
    // ShooterConstants.FlyWheelShootingTolerance || beltStarted ) {
    if ( isShooting && (flyWheelMotor.getSelectedSensorVelocity() < -15500.0 ) ) {
      if (!beltStarted) {
        beltMotor.set(TalonFXControlMode.Velocity, -Constants.ShooterConstants.BeltMotorShootingVelocity);
        beltMotor.set(TalonFXControlMode.PercentOutput, -0.2);
        beltStarted = true;
      }
    }
  }

  //   double temp = flyWheelMotor.getSelectedSensorVelocity();
  //   if ( temp  > 20) {
  //     int wpk = 1 ;
  //   }
  // }

  public void startShooting() {
    flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorShootingVelocity);
      //beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.BeltMotorShootingVelocity);
      isShooting = true ;
//    flyWheelMotor.set(TalonFXControlMode.PercentOutput, 0.7);
  }

  public void stopShooting() {
    beltMotor.set(TalonFXControlMode.Velocity, 0.0);
    flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
    beltStarted = false ;
    isShooting = false ;
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
    // beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.BeltMotorIntakeVelocity);
    // flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorIntakeVelocity);
    beltMotor.set(TalonFXControlMode.Velocity, 500);
    flyWheelMotor.set(TalonFXControlMode.Velocity, 500);
    // beltMotor.set(TalonFXControlMode.PercentOutput, 0.3);
    // flyWheelMotor.set(TalonFXControlMode.PercentOutput, 0.3);
  }
  
  public void stopReceiving(){
    beltMotor.set(TalonFXControlMode.Velocity, 0.0);
    flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
  }

  private void setMotorConfig(WPI_TalonFX motor) {
    motor.configFactoryDefault() ;
    // motor.configSelectedFeedbackSensor(
    //     FeedbackDevice.QuadEncoder, 
    //     Constants.IntakeConstants.mainFeedbackLoop,
    //     Constants.IntakeConstants.encoderTimeout
    //     ); 
    // motor.configNominalOutputForward(0);
    // motor.configNominalOutputReverse(0);
    // motor.configPeakOutputForward(1.0);
    // motor.configPeakOutputReverse(-1.0);
    //motor.configMotionCruiseVelocity( (int) (Constants.IntakeConstants.unitsPerRotation * Constants.IntakeConstants.desiredRPMsForDrive));
    motor.config_kF(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.DrivetrainKf);
    motor.config_kP(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.DrivetrainkP);
    motor.config_kI(Constants.IntakeConstants.PID_id, 0);
    motor.config_kD(Constants.IntakeConstants.PID_id, 0);
    motor.configClosedloopRamp(Constants.IntakeConstants.closedVoltageRampingConstant) ;
    motor.configOpenloopRamp(Constants.IntakeConstants.manualVoltageRampingConstant) ;
    motor.setNeutralMode(NeutralMode.Brake);
  }



  private void report() {
    // Report various parameters out to network tables for monitoring purposes
    NetworkTableInstance.getDefault().getEntry("shooter/belt_motor_velocity").setDouble(beltMotor.getSelectedSensorVelocity());
    NetworkTableInstance.getDefault().getEntry("shooter/flywheel_motor_velocity").setDouble(flyWheelMotor.getSelectedSensorVelocity());
}


  boolean simulationInitialized = false ;

  public void simulationInit() {
    PhysicsSim.getInstance().addTalonFX(beltMotor, 0.75, 6800);
    PhysicsSim.getInstance().addTalonFX(flyWheelMotor, 0.75, 6800);
  }

  @Override
  public void simulationPeriodic() {
    if (! simulationInitialized) {
      simulationInit();
      simulationInitialized = true ;
    }
  }


}

