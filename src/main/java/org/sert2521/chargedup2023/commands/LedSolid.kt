package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.subsystems.LEDs

class LedSolid(private val h: Int, private val s: Int, private val v: Int) : CommandBase() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(LEDs)
    }

    override fun initialize() {}

    override fun execute() {
        LEDs.setAllLEDHSV(h, s, v)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
