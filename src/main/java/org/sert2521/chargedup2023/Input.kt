package org.sert2521.chargedup2023

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.sert2521.chargedup2023.commands.ClawIntake
import org.sert2521.chargedup2023.commands.SetElevator
import org.sert2521.chargedup2023.subsystems.Drivetrain
import org.sert2521.chargedup2023.subsystems.GamePieces

object Input {
    // Replace with constants
    private val driverController = XboxController(0)

    private val resetAngle = JoystickButton(driverController, 4)

    private val intakeSetOne = JoystickButton(driverController, 3)
    private val intakeSetTwo = JoystickButton(driverController, 2)
    private val outtake = JoystickButton(driverController, 1)

    private val liftMid = JoystickButton(driverController, 5)
    private val liftBottom = JoystickButton(driverController, 6)

    private var lastPiece = GamePieces.CUBE

    init {
        resetAngle.onTrue(InstantCommand({ Drivetrain.setNewPose(Pose2d()) }))

        //Intaking a cone is the same as outtaking a cube
        intakeSetOne.onTrue(ClawIntake(GamePieces.CONE))
        intakeSetOne.onTrue(InstantCommand({ lastPiece = GamePieces.CUBE }))

        intakeSetTwo.onTrue(ClawIntake(GamePieces.CUBE))
        intakeSetTwo.onTrue(InstantCommand({ lastPiece = GamePieces.CONE }))

        var clawCommandDirection: ClawIntake? = null

        outtake.onTrue(InstantCommand({ clawCommandDirection = ClawIntake(lastPiece); println(lastPiece); clawCommandDirection?.schedule() }))
        outtake.onFalse(InstantCommand({ clawCommandDirection?.cancel(); clawCommandDirection = null }))

        // Replace with constants
        liftBottom.onTrue(SetElevator(0.02, 0.01))
        liftMid.onTrue(SetElevator(0.19, 0.65))
    }

    fun getX(): Double {
        return -driverController.leftX
    }

    fun getY(): Double {
        return -driverController.leftY
    }

    fun getRot(): Double {
        return driverController.rightX
    }
}