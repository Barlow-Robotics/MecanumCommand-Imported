// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmBarConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.sim.PhysicsSim;
import edu.wpi.first.networktables.*;


public class ArmBar extends SubsystemBase {

    public WPI_TalonFX armBarMotor;
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
        armBarMotor = new WPI_TalonFX(ArmBarConstants.ID_ArmBarMotor);
        armBarMotor.setSelectedSensorPosition(0.0);
        // armBarMotor.set(TalonSRXControlMode.Velocity, 0.0);
        setMotorConfig(armBarMotor);

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


    public void ResetPosition() {
        armBarMotor.setSelectedSensorPosition(0.0);
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
        armBarMotor.selectProfileSlot(Constants.ArmBarConstants.Position_PID_id, 0);
        armBarMotor.set(TalonFXControlMode.MotionMagic, desiredPosition);

        // armBarMotor.setSelectedSensorPosition(0.0); do we need this / where would it
        // go if we do
    }


    public void stopMotor() {
        armBarMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getArmAngle() {
        double result = armBarMotor.getSelectedSensorPosition() / Constants.ArmBarConstants.UnitsPerArmDegree ;
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

        motor.setNeutralMode(NeutralMode.Brake);
//        motor.setNeutralMode(NeutralMode.Coast);
    }


    public void setNormalMotionConfig() {
        armBarMotor.configMotionCruiseVelocity(Constants.ArmBarConstants.CruiseVelocity) ;
        armBarMotor.configMotionAcceleration(Constants.ArmBarConstants.MaxAcceleration) ;
        armBarMotor.configMotionSCurveStrength(Constants.ArmBarConstants.AccelerationSmoothing) ;
    }

    public void setSlowMotionConfig() {
        armBarMotor.configMotionCruiseVelocity(Constants.ArmBarConstants.SlowCruiseVelocity) ;
        armBarMotor.configMotionAcceleration(Constants.ArmBarConstants.SlowMaxAcceleration) ;
        armBarMotor.configMotionSCurveStrength(Constants.ArmBarConstants.AccelerationSmoothing) ;
    }


    void report() {
        NetworkTableInstance.getDefault().getEntry("arm_bar/motor_speed").setDouble(armBarMotor.getSelectedSensorVelocity());
        NetworkTableInstance.getDefault().getEntry("arm_bar/motor_position").setDouble(armBarMotor.getSelectedSensorPosition());
        NetworkTableInstance.getDefault().getEntry("arm_bar/motor_stator_current").setDouble(armBarMotor.getStatorCurrent());
        NetworkTableInstance.getDefault().getEntry("arm_bar/motor_supply_current").setDouble(armBarMotor.getSupplyCurrent());
        NetworkTableInstance.getDefault().getEntry("arm_bar/motor_closed_loop_error").setDouble(armBarMotor.getClosedLoopError());
        NetworkTableInstance.getDefault().getEntry("arm_bar/motor_closed_loop_target").setDouble(armBarMotor.getClosedLoopTarget());
    }



    boolean simulationInitialized = false;

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(armBarMotor, 0.75, 6800 );
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
