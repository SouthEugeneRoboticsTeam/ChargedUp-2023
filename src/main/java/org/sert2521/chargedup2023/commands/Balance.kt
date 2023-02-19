package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import kotlin.math.abs
import kotlin.math.sign

class Balance : CommandBase() {
    // This doesn't exactly make sense
    private val tiltPID = ProfiledPIDController(TunedConstants.balanceAngleP, TunedConstants.balanceAngleI, TunedConstants.balanceAngleD, TrapezoidProfile.Constraints(TunedConstants.balanceAngleMaxV, TunedConstants.balanceAngleMaxA))
    private val tiltFilter = LinearFilter.movingAverage(25)
    private val tiltRateFilter = LinearFilter.movingAverage(25)

    private var prevTilt = 0.0
    private var prevTime = 0.0

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        val tilt = Drivetrain.getTilt()
        tiltPID.reset(tilt)

        tiltFilter.reset()
        tiltRateFilter.reset()

        prevTilt = tilt
        prevTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        val tilt = Drivetrain.getTilt()
        val currentTime = Timer.getFPGATimestamp()
        val diffTime = (currentTime - prevTime)
        prevTime = currentTime

        val tiltRate = tiltRateFilter.calculate((tilt - prevTilt) / diffTime)
        val tiltFiltered = tiltFilter.calculate(tilt)
        prevTilt = tilt

        if ((sign(tiltRate) != sign(tiltFiltered) && abs(tiltRate) >= TunedConstants.balanceAngleSignificantRate) || abs(tiltFiltered) <= TunedConstants.balanceAngleTolerance) {
            tiltPID.calculate(tilt, tilt)
            Drivetrain.enterBrakePos()
        } else {
            val speed = tiltPID.calculate(tilt, 0.0)
            val driveVector = -Drivetrain.getTiltDirection() * speed
            Drivetrain.drive(ChassisSpeeds(driveVector.x, driveVector.y, 0.0))
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}