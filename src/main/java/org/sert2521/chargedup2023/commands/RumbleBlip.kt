package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.Input

class RumbleBlip(private val strength: Double, private val duration: Double) : CommandBase() {
    private var startTime = 0.0
    private var x = 0.0

    override fun initialize() {
        startTime = Timer.getFPGATimestamp()
    }

    override fun execute() {
        val x = (Timer.getFPGATimestamp() - startTime) / duration
        Input.rumble(4.0 * strength * x * (1 - x))
    }

    override fun isFinished(): Boolean {
        return x > 1.0
    }

    override fun end(interrupted: Boolean) {
        Input.rumble(0.0)
    }
}