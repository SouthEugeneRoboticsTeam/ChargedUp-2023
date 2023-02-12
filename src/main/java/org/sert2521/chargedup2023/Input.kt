package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.sert2521.chargedup2023.commands.ClawIntake
import org.sert2521.chargedup2023.subsystems.GamePieces

object Input {
    private val driverController = XboxController(0)

    private val intakeSetOne = JoystickButton(driverController, 3)

    private val intakeSetTwo = JoystickButton(driverController, 2)

    private val outtake = JoystickButton(driverController, 1)

    private var lastPiece = GamePieces.CUBE

    init {
        //Intaking a cone is the same as outtaking a cube
        intakeSetOne.onTrue(ClawIntake(GamePieces.CONE))
        intakeSetOne.onTrue(InstantCommand({ lastPiece = GamePieces.CUBE }))

        intakeSetTwo.onTrue(ClawIntake(GamePieces.CUBE))
        intakeSetTwo.onTrue(InstantCommand({ lastPiece = GamePieces.CONE }))


        var clawCommandDirection: ClawIntake? = null

        outtake.onTrue(InstantCommand({clawCommandDirection = ClawIntake(lastPiece); println(lastPiece); clawCommandDirection?.schedule()}))
        outtake.onFalse(InstantCommand({clawCommandDirection?.cancel(); clawCommandDirection = null}))

    }
}