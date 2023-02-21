package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.subsystems.LEDSides
import org.sert2521.chargedup2023.subsystems.LEDs

class LedIdle : CommandBase() {


    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(LEDs)
    }

    override fun initialize() {
        LEDs.reset()
    }

    override fun execute() {
        for (i in 0..PhysicalConstants.ledLeftLength){
            LEDs.setLEDHSV(LEDSides.LEFT, i, (((Timer.getFPGATimestamp().mod(1.0)*360)+i*4).toInt()).mod(PhysicalConstants.ledLeftLength), 100, 100)
        }
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false
    }

    override fun end(interrupted: Boolean) {}
}
