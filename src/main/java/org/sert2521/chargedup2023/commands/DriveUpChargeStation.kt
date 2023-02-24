package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain

class DriveUpChargeStation : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    override fun execute() {
        val driveVector = Drivetrain.getTiltDirection() * TunedConstants.balanceDriveUpSpeed
        Drivetrain.drive(ChassisSpeeds(driveVector.x, driveVector.y, 0.0))
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}