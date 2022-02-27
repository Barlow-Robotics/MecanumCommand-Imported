// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.networktables.*;


public class ArmBar extends SubsystemBase {

    public WPI_TalonFX rightMotor;
    public WPI_TalonFX leftMotor;
    // Add sensors

    public DigitalInput hallEffectsA1;
    public DigitalInput hallEffectsA2;
    public DigitalInput hallEffectsB1;
    public DigitalInput hallEffectsB2;

    public DigitalOutput solenoidA1;
    public DigitalOutput solenoidA2;
    public DigitalOutput solenoidB1;
    public DigitalOutput solenoidB2;


    /** Creates a new ArmBar. */
    public ArmBar() {
        rightMotor = new WPI_TalonFX(ArmBarConstants.ID_rightMotor);
        // rightMotor.set(TalonSRXControlMode.Velocity, 0.0);
        setMotorConfig(rightMotor);

        leftMotor = new WPI_TalonFX(ArmBarConstants.ID_leftMotor);
        setMotorConfig(leftMotor);
        leftMotor.follow(rightMotor);
        leftMotor.follow(rightMotor);
        leftMotor.setInverted(TalonFXInvertType.OpposeMaster);

        rightMotor.setSelectedSensorPosition(0.0);

        hallEffectsA1 = new DigitalInput(Constants.ArmBarConstants.ID_HallEffectsA1);
        hallEffectsA2 = new DigitalInput(Constants.ArmBarConstants.ID_HallEffectsA2);
        hallEffectsB1 = new DigitalInput(Constants.ArmBarConstants.ID_HallEffectsB1);
        hallEffectsB2 = new DigitalInput(Constants.ArmBarConstants.ID_HallEffectsB2);

        solenoidA1 = new DigitalOutput(Constants.ArmBarConstants.ID_SolenoidA1) ;
        solenoidA2 = new DigitalOutput(Constants.ArmBarConstants.ID_SolenoidA2) ;
        solenoidB1 = new DigitalOutput(Constants.ArmBarConstants.ID_SolenoidB1) ;
        solenoidB2 = new DigitalOutput(Constants.ArmBarConstants.ID_SolenoidB2) ;

        neutralGripperA();
        neutralGripperB();

        setNormalMotionConfig();

        // solenoidA1 = new Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.ArmBarConstants.ID_HallEffectsA1);
        // solenoidA2 = new Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.ArmBarConstants.ID_HallEffectsA2);
        // solenoidB1 = new Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.ArmBarConstants.ID_HallEffectsB1);
        // solenoidB2= new Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.ArmBarConstants.ID_HallEffectsB2);

        report() ;

    }


    public void ResetPosition(double angle) {
        rightMotor.setSelectedSensorPosition(angle, 0,  30);
    }

    @Override
    public void periodic() {
        
        // This method will be called once per scheduler run
        report() ;
    }

    public void rotateGripperArmDegree(double angle) {
        // motor will go until the bar is rotated so that the the original bar position
        // and the new bar position form the desired angle
        // double desiredPosition = (angle / 360) * Constants.ArmBarConstants.GearboxGearRatio
        //         * Constants.ArmBarConstants.UnitsPerRotation;

        double desiredPosition = angle * Constants.ArmBarConstants.UnitsPerArmDegree ;
        rightMotor.selectProfileSlot(Constants.ArmBarConstants.Position_PID_id, 0);
        rightMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);

        // rightMotor.setSelectedSensorPosition(0.0); do we need this / where would it
        // go if we do
    }


    public void stopMotor() {
        rightMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getArmAngle() {
        double result = rightMotor.getSelectedSensorPosition() / Constants.ArmBarConstants.UnitsPerArmDegree ;
        return result;
    }

    public void releaseGripperA() {
        solenoidA1.set(true);
        solenoidA2.set(true);
    }

    public void releaseGripperB() {
        solenoidB1.set(true);
        solenoidB2.set(true);
    }


    public void neutralGripperA() {
        solenoidA1.set(false);
        solenoidA2.set(false);
    }

    public void neutralGripperB() {
        solenoidB1.set(false);
        solenoidB2.set(false);
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


    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.ArmBarConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ArmBarConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.ArmBarConstants.Position_PID_id, Constants.ArmBarConstants.Position_kF, 30);
        motor.config_kP(Constants.ArmBarConstants.Position_PID_id, Constants.ArmBarConstants.Position_kP, 30);
        motor.config_kI(Constants.ArmBarConstants.Position_PID_id, 0, 30);
        motor.config_kD(Constants.ArmBarConstants.Position_PID_id, 0, 30);

        motor.configMotionCruiseVelocity(Constants.ArmBarConstants.CruiseVelocity) ;
        motor.configMotionAcceleration(Constants.ArmBarConstants.CruiseVelocity) ;
        motor.configMotionSCurveStrength(Constants.ArmBarConstants.AccelerationSmoothing) ;


        motor.config_kF(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kF);
        motor.config_kP(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kP);
        motor.config_kI(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kD);
        motor.config_kD(Constants.ArmBarConstants.Velocity_PID_id, 0);

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
//        motor.setNeutralMode(NeutralMode.Coast);
    }


    public void setNormalMotionConfig() {
        rightMotor.configMotionCruiseVelocity(Constants.ArmBarConstants.CruiseVelocity) ;
        rightMotor.configMotionAcceleration(Constants.ArmBarConstants.MaxAcceleration) ;
        rightMotor.configMotionSCurveStrength(Constants.ArmBarConstants.AccelerationSmoothing) ;
    }

    public void setSlowMotionConfig() {
        rightMotor.configMotionCruiseVelocity(Constants.ArmBarConstants.SlowCruiseVelocity) ;
        rightMotor.configMotionAcceleration(Constants.ArmBarConstants.SlowMaxAcceleration) ;
        rightMotor.configMotionSCurveStrength(Constants.ArmBarConstants.AccelerationSmoothing) ;
    }


    void report(TalonFX motor, String motorName) {
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_speed").setDouble(motor.getSelectedSensorVelocity());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_position").setDouble(motor.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_stator_current").setDouble(motor.getStatorCurrent());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_supply_current").setDouble(motor.getSupplyCurrent());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_closed_loop_error").setDouble(motor.getClosedLoopError());
        NetworkTableInstance.getDefault().getEntry("arm_bar/" + motorName + "_closed_loop_target").setDouble(motor.getClosedLoopTarget());
    }

    void report() {
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

        // do sim stuff

    }

}
