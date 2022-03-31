// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.networktables.*;


public class ArmBar extends SubsystemBase {

    public WPI_TalonFX rightMotor;
    public WPI_TalonFX leftMotor;
    // Add sensors

    DigitalInput hallEffectsA1;
    DigitalInput hallEffectsA2;
    DigitalInput hallEffectsB1;
    DigitalInput hallEffectsB2;

    Servo clawA1Servo ;
    Servo clawA2Servo ;
    Servo clawB1Servo ;
    Servo clawB2Servo ;


    /** Creates a new ArmBar. */
    public ArmBar() {
        rightMotor = new WPI_TalonFX(ArmBarConstants.ID_rightMotor);
        setMotorConfig(rightMotor);

        leftMotor = new WPI_TalonFX(ArmBarConstants.ID_leftMotor);
        setMotorConfig(leftMotor);
        leftMotor.setInverted(TalonFXInvertType.Clockwise);
        // leftMotor.follow(rightMotor);
        // leftMotor.setInverted(TalonFXInvertType.OpposeMaster);

        rightMotor.setSelectedSensorPosition(0.0);
        leftMotor.setSelectedSensorPosition(0.0) ;

        hallEffectsA1 = new DigitalInput(Constants.ArmBarConstants.ID_HallEffectsA1);
        hallEffectsA2 = new DigitalInput(Constants.ArmBarConstants.ID_HallEffectsA2);
        hallEffectsB1 = new DigitalInput(Constants.ArmBarConstants.ID_HallEffectsB1);
        hallEffectsB2 = new DigitalInput(Constants.ArmBarConstants.ID_HallEffectsB2);

        clawA1Servo = new Servo(Constants.ArmBarConstants.ID_ServoA1) ;
        clawA2Servo = new Servo(Constants.ArmBarConstants.ID_ServoA2) ;
        clawB1Servo = new Servo(Constants.ArmBarConstants.ID_ServoB1) ;
        clawB2Servo = new Servo(Constants.ArmBarConstants.ID_ServoB2) ;

        neutralGripperA();
        neutralGripperB();

        setNormalMotionConfig();
        report() ;
    }

    public void resetPosition(double angle) {
        rightMotor.setSelectedSensorPosition(angle, 0,  30);
        leftMotor.setSelectedSensorPosition(angle, 0,  30);
    }

    private void updateHallStatesForDashboard() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault() ;

        inst.getEntry("hall_effects/A1").setBoolean(hallEffectsA1.get()) ;
        inst.getEntry("hall_effects/A2").setBoolean(hallEffectsA2.get()) ;
        inst.getEntry("hall_effects/B1").setBoolean(hallEffectsB1.get()) ;
        inst.getEntry("hall_effects/B2").setBoolean(hallEffectsB2.get()) ;
    }

    public boolean A1HallOpen() {
        return hallEffectsA1.get() ;
    }

    public boolean A2HallOpen() {
        return hallEffectsA2.get() ;
    }

    public boolean B1HallOpen() {
        return hallEffectsB1.get() ;
    }

    public boolean B2HallOpen() {
        return hallEffectsB2.get() ;
    }


    @Override
    public void periodic() {
        
        updateHallStatesForDashboard(); 
        // This method will be called once per scheduler run
        report() ;
    }

    public void rotateGripperArmDegree(double angle) {
        // motor will go until the bar is rotated so that the the original bar position
        // and the new bar position form the desired angle

        double desiredPosition = angle * Constants.ArmBarConstants.UnitsPerArmDegree ;
        rightMotor.selectProfileSlot(Constants.ArmBarConstants.Position_PID_id, 0);
        leftMotor.selectProfileSlot(Constants.ArmBarConstants.Position_PID_id, 0);
        
        rightMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);
        leftMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);
    }


    public void stopMotor() {
        rightMotor.set(TalonFXControlMode.PercentOutput, 0);
        leftMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getArmAngle() {
        double result = rightMotor.getSelectedSensorPosition() / Constants.ArmBarConstants.UnitsPerArmDegree ;
        return result;
    }

    public void releaseGripperA() {
        clawA1Servo.set(0.5) ;
        clawA2Servo.set(0.5) ;

    }

    public void releaseGripperB() {
        clawB1Servo.set(0.0) ;
        clawB2Servo.set(0.5) ;
    }

    public void neutralGripperA() {
        clawA1Servo.set(0.0) ;
        clawA2Servo.set(0.0) ;
    }

    public void neutralGripperB() {
        clawB1Servo.set(0.5) ;
        clawB2Servo.set(0.0) ;
    }


    public boolean gripperAIsClosed() {
        return (!hallEffectsA1.get() && !hallEffectsA2.get());
    }

    public boolean gripperBIsClosed() {
        return (!hallEffectsB1.get() && !hallEffectsB2.get());
    }

    public boolean gripperAIsOpen() {
        return (hallEffectsA1.get() && hallEffectsA2.get());
    }

    public boolean gripperBIsOpen() {
        return (hallEffectsB1.get() && hallEffectsB2.get());
    }



    public void setMotorsNeutral() {
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);
    }


    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.ArmBarConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ArmBarConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.ArmBarConstants.Position_PID_id, Constants.ArmBarConstants.Position_kF, 30);
        motor.config_kP(Constants.ArmBarConstants.Position_PID_id, Constants.ArmBarConstants.Position_kP, 30);
        motor.config_kI(Constants.ArmBarConstants.Position_PID_id, Constants.ArmBarConstants.Position_kI, 30);
        motor.config_kD(Constants.ArmBarConstants.Position_PID_id, Constants.ArmBarConstants.Position_kD, 30);

        motor.configMotionCruiseVelocity(Constants.ArmBarConstants.CruiseVelocity) ;
        motor.configMotionAcceleration(Constants.ArmBarConstants.MaxAcceleration) ;
        motor.configMotionSCurveStrength(Constants.ArmBarConstants.AccelerationSmoothing) ;


        // motor.config_kF(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kF);
        // motor.config_kP(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kP);
        // motor.config_kI(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kD);
        // motor.config_kD(Constants.ArmBarConstants.Velocity_PID_id, 0);

        motor.configNominalOutputForward(0, 30);
		motor.configNominalOutputReverse(0, 30);
		motor.configPeakOutputForward(1, 30);
		motor.configPeakOutputReverse(-1, 30);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		motor.configAllowableClosedloopError(Constants.ArmBarConstants.Position_PID_id, 0, 30);
        motor.setNeutralMode(NeutralMode.Coast);
    }


    public void setNormalMotionConfig() {
        rightMotor.configMotionCruiseVelocity(Constants.ArmBarConstants.CruiseVelocity) ;
        rightMotor.configMotionAcceleration(Constants.ArmBarConstants.MaxAcceleration) ;
        rightMotor.configMotionSCurveStrength(Constants.ArmBarConstants.AccelerationSmoothing) ;
    }

    // public void setSlowMotionConfig() {
    //     rightMotor.configMotionCruiseVelocity(Constants.ArmBarConstants.SlowCruiseVelocity) ;
    //     rightMotor.configMotionAcceleration(Constants.ArmBarConstants.SlowMaxAcceleration) ;
    //     rightMotor.configMotionSCurveStrength(Constants.ArmBarConstants.AccelerationSmoothing) ;
    // }


    void report(TalonFX motor, String motorName) {
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_speed").setDouble(motor.getSelectedSensorVelocity());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_speed_dps").setDouble(motor.getSelectedSensorVelocity() / Constants.ArmBarConstants.DegreePerSecond);
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_position").setDouble(motor.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_stator_current").setDouble(motor.getStatorCurrent());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_supply_current").setDouble(motor.getSupplyCurrent());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_closed_loop_error").setDouble(motor.getClosedLoopError());
        if ( motor.getControlMode() == ControlMode.MotionMagic) {
            NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_closed_loop_target").setDouble(motor.getClosedLoopTarget());
            NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_closed_loop_error").setDouble(motor.getClosedLoopError());
        } else {
            NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_closed_loop_target").setDouble(0.0);
        }
    }

    public void report() {
        report( rightMotor, "rightMotor") ;
        report( leftMotor, "leftMotor") ;
        
    }


    boolean simulationInitialized = false;

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(rightMotor, 0.75, 6800 );
        PhysicsSim.getInstance().addTalonFX(leftMotor, 0.75, 6800 );
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
        PhysicsSim.getInstance().run();

    }

}
