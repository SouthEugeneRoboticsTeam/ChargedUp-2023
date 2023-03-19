package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.subsystems.LEDs
import kotlin.math.abs

class Balance : CommandBase() {
    private var prevTilt = 0.0
    private var prevTime = 0.0
    private var tiltRateFilter = LinearFilter.movingAverage(TunedConstants.filterTaps)

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        prevTime = Timer.getFPGATimestamp()
        prevTilt = Drivetrain.getTilt()

        tiltRateFilter.reset()

        LedSolid(10, 255, 255)
    }

    override fun execute() {
        val currentTime = Timer.getFPGATimestamp()
        val diffTime = (currentTime - prevTime)
        prevTime = currentTime

        val tilt = Drivetrain.getTilt()
        val tiltRate = tiltRateFilter.calculate(abs((tilt - prevTilt) / diffTime))
        prevTilt = tilt

        val angleInBounds = abs(tilt) <= TunedConstants.balanceAngleTolerance
        if (angleInBounds) {
            Drivetrain.enterBrakePos()
        } else {
            if (tiltRate >= TunedConstants.balanceAngleSignificantRate) {
                Drivetrain.enterBrakePos()
            } else {
                val driveVector = Drivetrain.getTiltDirection() * TunedConstants.balanceSpeed
                Drivetrain.drive(ChassisSpeeds(driveVector.x, driveVector.y, 0.0))
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()

        LedFlash(60, 255, 255, 1.6)
    }
}