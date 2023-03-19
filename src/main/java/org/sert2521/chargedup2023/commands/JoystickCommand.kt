package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.Input
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

abstract class JoystickCommand : CommandBase() {
    private var x = 0.0
    private var y = 0.0
    private var prevTime = 0.0

    override fun initialize() {
        x = 0.0
        y = 0.0

        prevTime = Timer.getFPGATimestamp()
    }

    // Must be run periodically
    fun readJoystick(): Translation3d {
        val currentTime = Timer.getFPGATimestamp()
        val diffTime = currentTime - prevTime
        prevTime = currentTime

        val fast = Input.getFast()
        var currX = Input.getX()
        var currY = Input.getY()

        val sqrMagnitude = currX.pow(2) + currY.pow(2)
        if (sqrMagnitude > 1) {
            val magnitude = sqrt(sqrMagnitude)
            currX /= magnitude
            currY /= magnitude
        } else if (sqrMagnitude < ConfigConstants.powerDeadband.pow(2)) {
            currX = 0.0
            currY = 0.0
        }

        val trueSpeed = ConfigConstants.driveSpeed - (ConfigConstants.driveSpeedup * fast)
        currX *= trueSpeed
        currY *= trueSpeed

        val diffX = x - currX
        val diffY = y - currY
        val rateChangeSqr = diffX.pow(2) + diffY.pow(2)
        if (rateChangeSqr <= ConfigConstants.joystickChangeSpeed.pow(2)) {
            x = currX
            y = currY
        } else {
            val rateChange = sqrt(rateChangeSqr) / diffTime
            x -= diffX / diffTime * ConfigConstants.joystickChangeSpeed / rateChange
            y -= diffY / diffTime * ConfigConstants.joystickChangeSpeed / rateChange
        }

        var rot = Input.getRot()
        if (abs(rot) < ConfigConstants.rotDeadband) {
            rot = 0.0
        }
        rot *= (ConfigConstants.rotSpeed - (ConfigConstants.rotSpeedup * fast))

        return Translation3d(x, y, rot)
    }
}