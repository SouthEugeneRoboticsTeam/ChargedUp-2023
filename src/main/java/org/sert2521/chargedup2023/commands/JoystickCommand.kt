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
    // This smoothly moves the output around following the input
    // It is essentially a 2d slew rate limiter with a different limit out than in
    fun readJoystick(): Translation3d {
        val currentTime = Timer.getFPGATimestamp()
        val diffTime = currentTime - prevTime
        prevTime = currentTime

        val fast = Input.getFast()
        var currX = Input.getX()
        var currY = Input.getY()

        // Checks if the joystick is outputting a magnitude greater than 1 and if it is normalizes the input
        // Otherwise it deadbands the input
        val sqrMagnitude = currX.pow(2) + currY.pow(2)
        if (sqrMagnitude > 1) {
            val magnitude = sqrt(sqrMagnitude)
            currX /= magnitude
            currY /= magnitude
        } else if (sqrMagnitude <= ConfigConstants.powerDeadband.pow(2)) {
            currX = 0.0
            currY = 0.0
        }

        // Converts the x and y input into m/s so the rate limiters apply in m/s
        val trueSpeed = ConfigConstants.driveSpeed - (ConfigConstants.driveSpeedup * fast)
        currX *= trueSpeed
        currY *= trueSpeed

        // Gets the vector from the input to the current output
        val changeX = currX - x
        val changeY = currY - y
        val change = sqrt(changeX.pow(2) + changeY.pow(2))

        val normalizedChangeX = changeX / change
        val normalizedChangeY = changeY / change

        // This takes the dot product of the normalized change vector and the normalized current output vector
        // Then it uses that to create a weighted average based on the slowdown speed and speedup speed
        val outwardDot = (x * normalizedChangeX + y * normalizedChangeY) / sqrt(x.pow(2) + y.pow(2))
        val maxChangeRate = (ConfigConstants.driveSpeedupChangeSpeed * (outwardDot + 1.0) + ConfigConstants.driveSlowdownChangeSpeed * (1.0 - outwardDot)) / 2.0

        // Moves the current output to the input on just sets it if it would overshoot
        if (change <= maxChangeRate) {
            x = currX
            y = currY
        } else {
            x += normalizedChangeX * maxChangeRate * diffTime
            y += normalizedChangeY * maxChangeRate * diffTime
        }

        var rot = Input.getRot()
        if (abs(rot) <= ConfigConstants.rotDeadband) {
            rot = 0.0
        }
        rot *= (ConfigConstants.rotSpeed - (ConfigConstants.rotSpeedup * fast))

        return Translation3d(x, y, rot)
    }
}