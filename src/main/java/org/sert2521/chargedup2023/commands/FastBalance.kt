package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import kotlin.math.abs

class FastBalance : CommandBase() {
    private var prevTilt = 0.0
    private var prevTime = 0.0
    private var tiltRateFilter = LinearFilter.movingAverage(TunedConstants.standardFilterTaps)

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        prevTime = Timer.getFPGATimestamp()
        prevTilt = Drivetrain.getTilt()

        tiltRateFilter.reset()

        //LedSolid(10, 255, 255).schedule()
    }

    override fun execute() {
        val currentTime = Timer.getFPGATimestamp()
        val diffTime = (currentTime - prevTime)
        prevTime = currentTime

        var tilt = Drivetrain.getTilt()
        val tiltIsBackward = true
        if (tiltIsBackward){
            tilt *= -1.0
            prevTilt *= -1.0
        }
        var tiltRate = tiltRateFilter.calculate((tilt - prevTilt) / diffTime)
        prevTilt = abs(tilt)

        val angleInBounds = abs(tilt) <= TunedConstants.balanceAngleTolerance

        val targetTiltRate = TunedConstants.fastBalanceA*(abs(abs((tilt)/TunedConstants.fastBalanceB)+TunedConstants.fastBalanceC)+TunedConstants.fastBalanceC)*(tilt)/(abs(tilt))

        if (tiltRate<targetTiltRate+TunedConstants.fastBalanceAcceptableRange && tiltRate>targetTiltRate-TunedConstants.fastBalanceAcceptableRange){
            Drivetrain.stop()
        }else if (tiltRate<targetTiltRate-TunedConstants.fastBalanceAcceptableRange){
            Drivetrain.drive(ChassisSpeeds(0.0, TunedConstants.fastBalanceCorrectionSpeed, 0.0))
        }else{
            Drivetrain.drive(ChassisSpeeds(0.0, -TunedConstants.fastBalanceCorrectionSpeed, 0.0))
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.stop()

        //LedFlash(60, 255, 255, 1.6).schedule()
    }
}