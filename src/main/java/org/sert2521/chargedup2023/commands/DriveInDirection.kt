package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.subsystems.Drivetrain

class DriveInDirection(private val driveVector: Translation2d) : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveVector.x, driveVector.y, 0.0, Drivetrain.getPose().rotation))
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}