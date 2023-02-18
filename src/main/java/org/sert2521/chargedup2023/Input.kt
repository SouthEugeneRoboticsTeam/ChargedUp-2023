package org.sert2521.chargedup2023

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.sert2521.chargedup2023.ConfigConstants.autoConstraints
import org.sert2521.chargedup2023.commands.ClawIntake
import org.sert2521.chargedup2023.commands.GamePieces
import org.sert2521.chargedup2023.commands.SetElevator
import org.sert2521.chargedup2023.subsystems.Drivetrain

object Input {
    // Replace with constants
    private val driverController = XboxController(0)
    private val gunnerController = Joystick(1)

    private val resetAngle = JoystickButton(driverController, 4)

    private val intakeSetOne = JoystickButton(gunnerController, 15)
    private val intakeSetTwo = JoystickButton(gunnerController, 14)
    private val outtake = JoystickButton(gunnerController, 13)

    private val liftDrive = JoystickButton(gunnerController, 5)
    private val liftConeHigh = JoystickButton(gunnerController, 6)
    private val liftCubeHigh = JoystickButton(gunnerController, 7)
    private val liftMid = JoystickButton(gunnerController, 8)
    private val liftLow = JoystickButton(gunnerController, 9)
    private val liftIntakeDown = JoystickButton(gunnerController, 10)
    private val liftIntakeCube = JoystickButton(gunnerController, 11)
    private val liftIntakeCone = JoystickButton(gunnerController, 12)

    private var lastPiece = GamePieces.CUBE

    private val autoBuilder = SwerveAutoBuilder(
        Drivetrain::pose,
        Drivetrain::setNewPose,
        PIDConstants(TunedConstants.swerveAutoPowerP, TunedConstants.swerveAutoPowerI, TunedConstants.swerveAutoPowerD),
        PIDConstants(TunedConstants.swerveAutoAngleP, TunedConstants.swerveAutoAngleI, TunedConstants.swerveAutoAngleD),
        Drivetrain::drive,
        ConfigConstants.eventMap,
        //figure this out
        true,
        Drivetrain
    )

    init {
        // Replace numbers with constants
        resetAngle.onTrue(InstantCommand({ Drivetrain.setNewPose(Pose2d()) }))

        //Intaking a cone is the same as outtaking a cube
        intakeSetOne.whileTrue(ClawIntake(GamePieces.CONE, false))
        intakeSetOne.onTrue(InstantCommand({ lastPiece = GamePieces.CUBE }))

        intakeSetTwo.whileTrue(ClawIntake(GamePieces.CUBE, false))
        intakeSetTwo.onTrue(InstantCommand({ lastPiece = GamePieces.CONE }))

        var clawCommandDirection: ClawIntake? = null

        outtake.onTrue(InstantCommand({ clawCommandDirection = ClawIntake(lastPiece, true); clawCommandDirection?.schedule() }))
        outtake.onFalse(InstantCommand({ clawCommandDirection?.cancel(); clawCommandDirection = null }))

        // Add feedforward
        liftDrive.onTrue(SetElevator(0.0, 0.93, false))
        // This 0.20 is in place of a feedforward
        liftConeHigh.onTrue(SetElevator(0.20, 0.7, false))
        liftCubeHigh.onTrue(SetElevator(0.18, 0.55, false))
        liftMid.onTrue(SetElevator(0.065, 0.57, false))
        liftLow.onTrue(SetElevator(0.0, 0.22, false))
        liftIntakeDown.onTrue(SetElevator(0.0, 0.01, false))
        liftIntakeCube.onTrue(SetElevator(0.0, 0.05, false))
        liftIntakeCone.onTrue(SetElevator(0.0, 0.13, false))
    }

    fun getAuto(): Command? {
        return autoBuilder.fullAuto(PathPlanner.loadPath("Test", autoConstraints))
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
}