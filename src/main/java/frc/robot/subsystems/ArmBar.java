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

public class ArmBar extends SubsystemBase {

    public WPI_TalonFX armBarMotor;
    // Add sensors

    public DigitalInput hallEffectsA1;
    public DigitalInput hallEffectsA2;
    public DigitalInput hallEffectsB1;
    public DigitalInput hallEffectsB2;

    public DigitalOutput solenoidA1;

    // public Solenoid solenoidA1;
    // public Solenoid solenoidA2;
    // public Solenoid solenoidB1;
    // public Solenoid solenoidB2;

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

        // solenoidA1 = new Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.ArmBarConstants.ID_HallEffectsA1);
        // solenoidA2 = new Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.ArmBarConstants.ID_HallEffectsA2);
        // solenoidB1 = new Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.ArmBarConstants.ID_HallEffectsB1);
        // solenoidB2= new Solenoid(PneumaticsModuleType.CTREPCM,
        // Constants.ArmBarConstants.ID_HallEffectsB2);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void rotateGripperArmDegree(double angle) {
        // motor will go until the bar is rotated so that the the original bar position
        // and the new bar position form the desired angle
        double desiredPosition = (angle / 360) * Constants.ArmBarConstants.GearboxGearRatio
                * Constants.ArmBarConstants.UnitsPerRotation;

        armBarMotor.selectProfileSlot(Constants.ArmBarConstants.Position_PID_id, 0);
        armBarMotor.set(TalonFXControlMode.Position, desiredPosition);
        // armBarMotor.setSelectedSensorPosition(0.0); do we need this / where would it
        // go if we do
    }

    public double getArmAngle() {
        double result = 360 * armBarMotor.getSelectedSensorPosition() 
                          / ( Constants.ArmBarConstants.UnitsPerRotation * Constants.ArmBarConstants.GearboxGearRatio * Constants.ArmBarConstants.ChainGearRatio);

        if ( result > 10) {
            int wpk =1 ;
        }
        return result;
    }

    public void releaseGripperA() {
        // solenoidA1.set(true);
        // solenoidA2.set(true);
    }

    public void releaseGripperB() {
        // solenoidB1.set(true);
        // solenoidB2.set(true);
    }

    public boolean gripperAIsClosed() {
        // Check for halleffect
        return (!hallEffectsA1.get() && !hallEffectsA2.get());
    }

    public boolean gripperBIsClosed() {
        // Check for halleffect
        return (!hallEffectsB1.get() && !hallEffectsB2.get());
    }


    private void setMotorConfig(WPI_TalonFX motor) {
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.ArmBarConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.ArmBarConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.ArmBarConstants.Position_PID_id, Constants.ArmBarConstants.Position_kF);
        motor.config_kP(Constants.ArmBarConstants.Position_PID_id, Constants.ArmBarConstants.Position_kP);
        motor.config_kI(Constants.ArmBarConstants.Position_PID_id, 0);
        motor.config_kD(Constants.ArmBarConstants.Position_PID_id, 0);

        motor.config_kF(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kF);
        motor.config_kP(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kP);
        motor.config_kI(Constants.ArmBarConstants.Velocity_PID_id, Constants.ArmBarConstants.Velocity_kD);
        motor.config_kD(Constants.ArmBarConstants.Velocity_PID_id, 0);

        motor.setNeutralMode(NeutralMode.Brake);
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
