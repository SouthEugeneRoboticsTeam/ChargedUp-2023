package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.*
import org.sert2521.chargedup2023.subsystems.Drivetrain

object Input {
    // Replace with constants
    private val driverController = XboxController(0)

    private val resetAngle = JoystickButton(driverController, 4)
    private val slowButton = JoystickButton(driverController, 5)

    var slowMode = false

    init {
        resetAngle.onTrue(InstantCommand({
            Drivetrain.setNewPose(Pose2d())
            Drivetrain.setNewVisionPose(Pose2d())
        }))

        slowButton.onTrue(InstantCommand({ slowMode = !slowMode }))
    }

    fun getBrakePos(): Boolean {
        return driverController.xButton
    }
    fun getFast(): Double {
        return if (!slowMode) {
            driverController.leftTriggerAxis
        } else {
            1.0
        }
    }

    fun getX(): Double {
        return -driverController.leftY
    }

    fun getY(): Double {
        return -driverController.leftX
    }

    fun getRot(): Double {
        return -driverController.rightX
    }

    fun getColor(): Alliance {
        return DriverStation.getAlliance()
    }
}