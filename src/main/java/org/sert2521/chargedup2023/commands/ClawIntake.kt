package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.subsystems.Claw
import org.sert2521.chargedup2023.subsystems.GamePieces

//kai moment
class ClawIntake(private val gamePiece: GamePieces) : CommandBase() {

    init {
        addRequirements(Claw)
    }
    override fun execute() {


        if (gamePiece == GamePieces.CUBE){
            Claw.setMotor(0.5)
        }else{
            Claw.setMotor(-0.5)
        }

    }

    override fun end(interrupted: Boolean) {
        Claw.stop()
    }
}