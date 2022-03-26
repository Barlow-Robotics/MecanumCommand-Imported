// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterIndex;
import frc.robot.subsystems.Intake;

public class StartIntake extends CommandBase {

    private ShooterIndex m_shooter;
    private Intake m_intake;

    /** Creates a new StartIntake. */
    public StartIntake(ShooterIndex s, Intake i) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooter = s;
        m_intake = i;
        addRequirements(m_shooter);
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.startIntake();
        m_shooter.startReceiving();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_intake.stopIntake();
        // m_shooter.stopReceiving();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return false;
        return true ;
    }
}
