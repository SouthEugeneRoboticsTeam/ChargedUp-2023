package org.sert2521.chargedup2023

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Joystick
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
    private val gunnerController = Joystick(1)

    private val resetAngle = JoystickButton(driverController, 4)

    private val intakeSetOne = JoystickButton(gunnerController, 13)
    private val intakeSetTwo = JoystickButton(gunnerController, 12)
    private val outtake = JoystickButton(gunnerController, 11)

    private val liftDrive = JoystickButton(gunnerController, 5)
    private val liftConeHigh = JoystickButton(gunnerController, 6)
    private val liftCubeHigh = JoystickButton(gunnerController, 7)
    private val liftMid = JoystickButton(gunnerController, 8)
    private val liftLow = JoystickButton(gunnerController, 9)
    private val liftIntake = JoystickButton(gunnerController, 10)

    private var lastPiece = GamePieces.CUBE

    init {
        // Replace numbers with constants
        resetAngle.onTrue(InstantCommand({ Drivetrain.setNewPose(Pose2d()) }))

        //Intaking a cone is the same as outtaking a cube
        intakeSetOne.whileTrue(ClawIntake(GamePieces.CONE, 0.7))
        intakeSetOne.onTrue(InstantCommand({ lastPiece = GamePieces.CUBE }))

        intakeSetTwo.whileTrue(ClawIntake(GamePieces.CUBE, 0.7))
        intakeSetTwo.onTrue(InstantCommand({ lastPiece = GamePieces.CONE }))

        var clawCommandDirection: ClawIntake? = null

        outtake.onTrue(InstantCommand({ clawCommandDirection = ClawIntake(lastPiece, if (lastPiece == GamePieces.CONE) { 0.7 } else { 0.2 }); clawCommandDirection?.schedule() }))
        outtake.onFalse(InstantCommand({ clawCommandDirection?.cancel(); clawCommandDirection = null }))

        liftDrive.onTrue(SetElevator(0.0, 0.93, false))
        liftConeHigh.onTrue(SetElevator(0.18, 0.61, false))
        liftCubeHigh.onTrue(SetElevator(0.18, 0.55, false))
        liftMid.onTrue(SetElevator(0.065, 0.57, false))
        liftLow.onTrue(SetElevator(0.0, 0.22, false))
        liftIntake.onTrue(SetElevator(0.0, 0.01, false))
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