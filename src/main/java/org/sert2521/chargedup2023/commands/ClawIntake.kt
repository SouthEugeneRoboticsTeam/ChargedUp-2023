package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.subsystems.Claw
import kotlin.math.abs



//kai moment
class ClawIntake(private val outtakeSpeed:Double) : CommandBase() {
    init {
        addRequirements(Claw)
    }

    override fun initialize() {
        // Fix this
        Claw.setMotor(outtakeSpeed)
        if (outtakeSpeed>0.0){
            LedSolid(0, 255, 255)
        }else{
            LedSolid(60, 255, 255)
        }
    }

    override fun end(interrupted: Boolean) {
        Claw.stop()
        LedIdle()
    }
}