package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import kotlin.math.abs

class OntoChargeStation(private val driveVector: Translation2d) : CommandBase() {
    private val tiltFilter = LinearFilter.movingAverage(25)

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        tiltFilter.reset()
    }

    override fun execute() {
        Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveVector.x, driveVector.y, 0.0, Drivetrain.getPose().rotation))
    }

    override fun isFinished(): Boolean {
        return abs(tiltFilter.calculate(Drivetrain.getTilt())) >= TunedConstants.balanceAngleStart
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}