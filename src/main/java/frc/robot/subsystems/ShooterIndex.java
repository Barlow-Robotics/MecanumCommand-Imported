// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.*;
import frc.robot.sim.PhysicsSim;


public class ShooterIndex extends SubsystemBase {
    /** Creates a new Shooter. */

    WPI_TalonFX beltMotor;
    WPI_TalonFX flyWheelMotor;
    WPI_TalonFX liftMotor ;

    boolean beltStarted = false;
    boolean isShooting = false;

    double commandedPosition = 0.0 ;


    public enum LiftPostion { In_Transiton, Intake, Shooting } ;

    public ShooterIndex() {
        beltMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_ShooterMotor);
        flyWheelMotor = new WPI_TalonFX(Constants.ShooterConstants.ID_FlyWheelMotor);
        liftMotor = new WPI_TalonFX(Constants.ShooterConstants.Lift.ID_Motor) ;

        setMotorConfig(beltMotor);
        setMotorConfig(flyWheelMotor);
        setLiftMotorConfig(liftMotor) ;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        if ( isShooting && (flyWheelMotor.getSelectedSensorVelocity() > Constants.ShooterConstants.FlyWheelMinShootingSpeed ) ) {
            if (!beltStarted) {
                beltMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.BeltMotorShootingVelocity) ;
                beltStarted = true;
            }
        }
        report();
    }

    public void startShooting() {
        flyWheelMotor.set(TalonFXControlMode.Velocity, Constants.ShooterConstants.FlyWheelMotorShootingVelocity ) ;
        isShooting = true;
    }

    public void stopShooting() {
        beltMotor.set(TalonFXControlMode.Velocity, 0.0);
        flyWheelMotor.set(TalonFXControlMode.Velocity, 0.0);
        beltStarted = false;
        isShooting = false;
    }



    public void GotoShootingPosition() {
        // wpk - need to add something to detect where the arm really is.
        liftMotor.set(TalonFXControlMode.MotionMagic, Constants.ShooterConstants.Lift.MotorShootingAngle) ;
        //liftMotor.set(TalonFXControlMode.Position, Constants.ShooterConstants.Lift.MotorShootingAngle) ;
        commandedPosition = Constants.ShooterConstants.Lift.MotorShootingAngle ;
    }


    public void GotoIntakePosition() {
        // wpk - need to add something to detect where the arm really is.
        liftMotor.set(TalonFXControlMode.MotionMagic, Constants.ShooterConstants.Lift.MotorIntakeAngle) ;
        //liftMotor.set(TalonFXControlMode.Position, Constants.ShooterConstants.Lift.MotorIntakeAngle) ;
        commandedPosition = Constants.ShooterConstants.Lift.MotorIntakeAngle ;
    }


    public LiftPostion getPosition() {
//        System.out.println("shooter position is " + liftMotor.getSelectedSensorPosition() ) ;
        if ( liftMotor.getSelectedSensorPosition() > Constants.ShooterConstants.Lift.MotorShootingAngle * 0.98 ) {
            return LiftPostion.Shooting ;
        } else if (liftMotor.getSelectedSensorPosition() < Constants.ShooterConstants.Lift.MotorShootingAngle * 0.02) {
            return LiftPostion.Intake ;
        } else {
            return LiftPostion.In_Transiton ;
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

    
    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.IntakeConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.IntakeConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.kF);
        motor.config_kP(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.kP);
        motor.config_kI(Constants.IntakeConstants.PID_id, 0);
        motor.config_kD(Constants.IntakeConstants.PID_id, 0);
        motor.setNeutralMode(NeutralMode.Brake);
    }


    private void setLiftMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.setInverted(TalonFXInvertType.Clockwise);
        motor.configClosedloopRamp(Constants.IntakeConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.IntakeConstants.manualVoltageRampingConstant);

        motor.config_kF(Constants.ShooterConstants.Lift.PID_ID, Constants.ShooterConstants.Lift.kF);
        motor.config_kP(Constants.ShooterConstants.Lift.PID_ID, Constants.ShooterConstants.Lift.kP);
        motor.config_kI(Constants.ShooterConstants.Lift.PID_ID, Constants.ShooterConstants.Lift.kI);
        motor.config_kD(Constants.ShooterConstants.Lift.PID_ID, Constants.ShooterConstants.Lift.kD);

        motor.configMotionCruiseVelocity(Constants.ShooterConstants.Lift.CruiseVelocity) ;
        motor.configMotionAcceleration(Constants.ShooterConstants.Lift.CruiseVelocity) ;
        motor.configMotionSCurveStrength(Constants.ShooterConstants.Lift.AccelerationSmoothing) ;

        motor.setSelectedSensorPosition(0.0) ;

        motor.setNeutralMode(NeutralMode.Brake);
    }




    void report() {
        NetworkTableInstance.getDefault().getEntry("index/belt_motor_speed").setDouble(beltMotor.getSelectedSensorVelocity());
        NetworkTableInstance.getDefault().getEntry("index/flywheel_motor_speed").setDouble(flyWheelMotor.getSelectedSensorVelocity());
        NetworkTableInstance.getDefault().getEntry("index/lift_motor_speed").setDouble(liftMotor.getSelectedSensorVelocity());
        NetworkTableInstance.getDefault().getEntry("index/lift_motor_position").setDouble(liftMotor.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("index/lift_motor_stator_current").setDouble(liftMotor.getStatorCurrent());
        NetworkTableInstance.getDefault().getEntry("index/lift_motor_supply_current").setDouble(liftMotor.getSupplyCurrent());
        NetworkTableInstance.getDefault().getEntry("index/lift_motor_closed_loop_error").setDouble(liftMotor.getClosedLoopError());
        NetworkTableInstance.getDefault().getEntry("index/lift_motor_closed_loop_target").setDouble(liftMotor.getClosedLoopTarget());
        NetworkTableInstance.getDefault().getEntry("index/commandedPosition").setDouble(commandedPosition);
    }



    boolean simulationInitialized = false;

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(liftMotor, 0.75, 6800, false );
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
