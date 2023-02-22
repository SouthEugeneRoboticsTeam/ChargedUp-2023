package org.sert2521.chargedup2023.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.subsystems.Claw

enum class GamePieces {
    CONE,
    CUBE
}

//kai moment
class ClawIntake(private val gamePiece: GamePieces, private val outtake: Boolean) : CommandBase() {
    init {
        addRequirements(Claw)
    }

    override fun initialize() {
        // Fix this
        if (outtake) {
            if (gamePiece == GamePieces.CUBE) {
                Claw.setMotor(0.7)
            } else {
                Claw.setMotor(0.7)
            }
        } else {
            if (gamePiece == GamePieces.CUBE) {
                Claw.setMotor(-0.9)
            } else {
                Claw.setMotor(-0.9)
            }
        }
    }

    override fun end(interrupted: Boolean) {
        Claw.stop()
    }
}