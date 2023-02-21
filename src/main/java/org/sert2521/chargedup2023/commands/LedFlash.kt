package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.subsystems.LEDs

class LedFlash(private val h: Int, private val s: Int, private val v: Int) : CommandBase() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(LEDs)
    }

    override fun initialize() {
        LEDs.reset()
    }

    override fun execute() {
        if (Timer.getFPGATimestamp().mod(2.0) == 1.0){
            LEDs.setAllLEDRGB(h, s, v)
        }else if (Timer.getFPGATimestamp().mod( 2.0) == 0.0){
            LEDs.reset()
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {
        LEDs.reset()

    }
}
