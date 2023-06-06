package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.sert2521.chargedup2023.commands.*
import org.sert2521.chargedup2023.commands.ClawIntake
import org.sert2521.chargedup2023.commands.SetElevator
import org.sert2521.chargedup2023.subsystems.Claw
import org.sert2521.chargedup2023.subsystems.Drivetrain
import kotlin.math.PI

object Input {
    // Replace with constants
    private val driverController = XboxController(0)

    private val outtake = JoystickButton(driverController, 5)
    private val intake = JoystickButton(driverController, 6)

    private val resetRot = JoystickButton(driverController, 7)
    private val resetRot2 = JoystickButton(driverController, 8)

    init {
        outtake.whileTrue(ClawIntake(1.0))


        intake.whileTrue(ClawIntake(-1.0))

        resetRot.and(resetRot2).whileTrue(InstantCommand({ Drivetrain.setNewPose(Pose2d()) }))
    }

    fun getBrakePos(): Boolean {
        return false
    }

    fun getIntake(): Boolean {
        return driverController.rightBumper
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

    fun getAngle(): Double {
        return if (driverController.bButton) { 1.0 } else { 0.0 } + if (driverController.aButton) { -1.0 } else { 0.0 }
    }

    fun getExtend(): Double {
        return if (driverController.yButton) { 1.0 } else { 0.0 } + if (driverController.xButton) { -1.0 } else { 0.0 }
    }

    // This kinda violates the spirit of Input and Output
    fun rumble(amount: Double) {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount)
    }
}