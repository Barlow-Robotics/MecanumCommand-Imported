// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterIndex;

public class GotoShootingPosition extends CommandBase {

    private ShooterIndex m_shooter;

    /** Creates a new StartIntake. */
    // yes, we're using SHOOTERINDEX to extend intake
    public GotoShootingPosition(ShooterIndex i) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_shooter = i;
        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    // @Override
    public void execute() {
        m_shooter.GotoShootingPosition();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_shooter.getPosition() == ShooterIndex.LiftPosition.Shooting) {
            return true;
        } else {
            return false;
        }
    }
}