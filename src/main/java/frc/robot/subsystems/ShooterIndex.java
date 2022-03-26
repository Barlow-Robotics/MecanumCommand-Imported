// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import edu.wpi.first.networktables.*;
import frc.robot.sim.PhysicsSim;


public class ShooterIndex extends SubsystemBase {
    /** Creates a new Shooter. */

    WPI_TalonFX beltMotor;
    WPI_TalonFX flyWheelMotor;

    boolean beltStarted = false;
    boolean isShooting = false;

    double commandedPosition = 0.0 ;

    Solenoid extendSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.Lift.ID_Extend_Solenoid);
    Solenoid retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.Lift.ID_Retract_Solenoid);
    Solenoid extendSolenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.Lift.ID_Extend_Solenoid2);
    Solenoid retractSolenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.ShooterConstants.Lift.ID_Retract_Solenoid2);
    Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    Timer transitionTimer ;

    enum TransitionState { Idle, In_Transition } ;

    TransitionState transitionState = TransitionState.Idle ;
 
    public enum LiftPosition { In_Transition, Intake, Shooting } ;

    public ShooterIndex() {
        beltMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_BeltMotor);
        flyWheelMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_FlyWheelMotor);

        setMotorConfig(beltMotor);
        setMotorConfig(flyWheelMotor);

        transitionTimer = new Timer() ;
    }

    @Override
    public void periodic() {
        if ( isShooting && (flyWheelMotor.getSelectedSensorVelocity() > Constants.ShooterConstants.FlyWheelMinShootingSpeed ) ) {
            if (!beltStarted) {
                beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.BeltMotorLowGoalShootingVelocity) ;
                beltStarted = true;
            }
        }
        report();
    }

    public void startShootingLow() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorLowGoalShootingVelocity ) ;
        isShooting = true;
    }

    public void startShootingHigh() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorHighGoalShootingVelocity);
        isShooting = true;
    }

    public void stopShooting() {
        beltMotor.set(TalonFXControlMode.Velocity, 0.0);
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
        beltStarted = false;
        isShooting = false;
    }

    public void GotoShootingPosition() {
        extendSolenoid.set(false);
        extendSolenoid2.set(false);
        retractSolenoid.set(true);
        retractSolenoid2.set(true);
        transitionTimer.reset();
        transitionTimer.start();
        transitionState = TransitionState.In_Transition ;
    }

    public void GotoIntakePosition() {
        extendSolenoid.set(true);
        extendSolenoid2.set(true);
        retractSolenoid.set(false);
        retractSolenoid2.set(false);
        transitionTimer.reset();
        transitionTimer.start();
        transitionState = TransitionState.In_Transition ;
    }

    public LiftPosition getPosition() { //NO MORE MOTOR, NOW PISTON - use sensor or time (not rec) ?

        if ( transitionState == TransitionState.In_Transition) {
            if ( transitionTimer.hasElapsed(Constants.ShooterConstants.ShooterTransitionTimeout) ) {
                transitionState = TransitionState.Idle ;
                transitionTimer.stop();
                transitionTimer.reset() ;
            }
        }
        if (transitionState == TransitionState.Idle) {
            if (!extendSolenoid.get() && retractSolenoid.get() && !extendSolenoid2.get() && retractSolenoid2.get()) {
                return LiftPosition.Shooting;
            } else if (extendSolenoid.get() && !retractSolenoid.get() && extendSolenoid.get() && !retractSolenoid2.get()) {
                return LiftPosition.Intake;
            } else {
                return LiftPosition.In_Transition;
            }
        } else {
            return LiftPosition.In_Transition;
        }
    }

    public boolean hasStarted() {
        return (beltMotor.get() != 0.0 && flyWheelMotor.get() != 0.0);
    }

    public boolean isStopped() {
        return (beltMotor.get() == 0 && flyWheelMotor.get() == 0);
    }

    public void startReceiving() {
        beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.BeltMotorIntakeVelocity) ;
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorIntakeVelocity) ;
    }

    public void stopReceiving() {
        beltMotor.set(TalonFXControlMode.Velocity, 0.0);
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
    }
    
    public void startEjecting() {
        beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.BeltMotorEjectingVelocity);
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorEjectingVelocity);
    }

    public void stopEjecting() {
       beltMotor.set(TalonFXControlMode.Velocity, 0.0);
       flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
    }

    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.IntakeConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.IntakeConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.IntakeConstants.PID_id, Constants.ShooterConstants.kF);
        motor.config_kP(Constants.IntakeConstants.PID_id, Constants.ShooterConstants.kP);
        motor.config_kI(Constants.IntakeConstants.PID_id, Constants.ShooterConstants.kI);
        motor.config_kD(Constants.IntakeConstants.PID_id, Constants.ShooterConstants.kD);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    void report() {
        NetworkTableInstance.getDefault().getEntry("index/belt_motor_speed").setDouble(beltMotor.getSelectedSensorVelocity());
        // NetworkTableInstance.getDefault().getEntry("index/belt_motor_target_velocity").setDouble(beltMotor.getClosedLoopTarget());
        NetworkTableInstance.getDefault().getEntry("index/flywheel_motor_speed").setDouble(flyWheelMotor.getSelectedSensorVelocity());
        // NetworkTableInstance.getDefault().getEntry("index/flywheel_target_velocity").setDouble(flyWheelMotor.getClosedLoopTarget());
        NetworkTableInstance.getDefault().getEntry("index/commandedPosition").setDouble(commandedPosition);

        NetworkTableInstance.getDefault().getEntry("driverStation/is_shooting").setBoolean(isShooting);
        NetworkTableInstance.getDefault().getEntry("driverStation/shoot_orientation").setString(getPosition().name());
    }

    boolean simulationInitialized = false;

    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
        PhysicsSim.getInstance().run();

        // do sim stuff
    }
}
