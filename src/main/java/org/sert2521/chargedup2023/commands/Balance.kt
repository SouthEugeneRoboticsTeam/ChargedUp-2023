package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import kotlin.math.abs

class Balance : CommandBase() {
    private val anglePID = ProfiledPIDController(TunedConstants.balanceAngleP, TunedConstants.balanceAngleI, TunedConstants.balanceAngleD, TrapezoidProfile.Constraints(TunedConstants.balanceAngleMaxV, TunedConstants.balanceAngleMaxA))
    private val tiltFilter = LinearFilter.movingAverage(25)

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        val angle = Drivetrain.getTilt()
        anglePID.reset(angle)

        tiltFilter.reset()
    }

    override fun execute() {
        val tilt = Drivetrain.getTilt()

        if (abs(tiltFilter.calculate(tilt)) <= TunedConstants.balanceAngleTolerance) {
            anglePID.calculate(tilt, tilt)
            Drivetrain.enterBrakePos()
        } else {
            val speed = anglePID.calculate(tilt)
            val driveVector = Drivetrain.getTiltDirection() * speed
            Drivetrain.drive(ChassisSpeeds(driveVector.x, driveVector.y, 0.0))
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()
    }
}