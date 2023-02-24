package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import kotlin.math.abs

class Balance : CommandBase() {
    private var prevTilt = 0.0
    private var prevTime = 0.0
    private var hasTipped = false

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        prevTime = Timer.getFPGATimestamp()
        prevTilt = Drivetrain.getTilt()
        hasTipped = false
    }

    override fun execute() {
        val currentTime = Timer.getFPGATimestamp()
        val diffTime = (currentTime - prevTime)
        prevTime = currentTime

        val tilt = Drivetrain.getTilt()
        val tiltRate = (tilt - prevTilt) / diffTime
        prevTilt = tilt

        val angleInBounds = abs(tilt) <= TunedConstants.balanceAngleTolerance
        if (angleInBounds) {
            Drivetrain.enterBrakePos()
            hasTipped = true
        } else {
            val balanceSignificantRate = if (hasTipped) { TunedConstants.balanceAngleSignificantRateEnd } else { TunedConstants.balanceAngleSignificantRateStart }
            if (tiltRate <= -balanceSignificantRate) {
                Drivetrain.enterBrakePos()
            } else {
                val balanceSpeed = if (hasTipped) { TunedConstants.balanceSpeedEnd } else { TunedConstants.balanceSpeedStart }
                val driveVector = -Drivetrain.getTiltDirection() * balanceSpeed
                Drivetrain.drive(ChassisSpeeds(driveVector.x, driveVector.y, 0.0))
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}