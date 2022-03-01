// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

import edu.wpi.first.networktables.*;

public class Intake extends SubsystemBase {

    WPI_TalonFX intakeMotor;

    /** Creates a new FalconTest. */
    public Intake() {
        intakeMotor = new WPI_TalonFX(22);
        setMotorConfig(intakeMotor);
        intakeMotor.setInverted(TalonFXInvertType.Clockwise);
    }

    public void startIntake() {
        intakeMotor.set(TalonFXControlMode.Velocity, Constants.IntakeConstants.IntakeMotorSpeed);
    }

    public void stopIntake() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        report();
    }

    private void setMotorConfig(WPI_TalonFX motor) { // changed to TalonFX for intake
        motor.configFactoryDefault();
        motor.configClosedloopRamp(Constants.IntakeConstants.closedVoltageRampingConstant);
        motor.configOpenloopRamp(Constants.IntakeConstants.manualVoltageRampingConstant);
        motor.config_kF(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.kF);
        motor.config_kP(Constants.IntakeConstants.PID_id, Constants.IntakeConstants.kP);
        motor.config_kI(Constants.IntakeConstants.PID_id, 0);
        motor.config_kD(Constants.IntakeConstants.PID_id, 0);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    void report() {
        NetworkTableInstance.getDefault().getEntry("intake/motor_speed")
                .setDouble(intakeMotor.getSelectedSensorVelocity());
    }

    boolean simulationInitialized = false;

    public void simulationInit() {
        // PhysicsSim.getInstance().addTalonFX(intakeMotor, 0.5, 6800);
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationInitialized) {
            simulationInit();
            simulationInitialized = true;
        }
    }
}