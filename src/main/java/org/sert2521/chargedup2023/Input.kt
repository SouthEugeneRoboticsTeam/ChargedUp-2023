package org.sert2521.chargedup2023

import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import org.sert2521.chargedup2023.commands.*
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.io.File

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

    private val autoChooser = SendableChooser<Command?>()
    private val autoBuilder = SwerveAutoBuilder(
        Drivetrain::getPose,
        Drivetrain::setNewPose,
        PIDConstants(TunedConstants.swerveAutoDistanceP, TunedConstants.swerveAutoDistanceI, TunedConstants.swerveAutoDistanceD),
        PIDConstants(TunedConstants.swerveAutoAngleP, TunedConstants.swerveAutoAngleI, TunedConstants.swerveAutoAngleD),
        Drivetrain::drive,
        ConfigConstants.eventMap,
        true,
        Drivetrain
    )

    init {
        // Put these strings in constants maybe
        autoChooser.setDefaultOption("Nothing", null)
        val pathFiles = File(ConfigConstants.pathsPath).listFiles()
        if (pathFiles != null) {
            for (pathFile in pathFiles) {
                // Make it not adding paths by the +
                autoChooser.addOption(pathFile.nameWithoutExtension, autoBuilder.fullAuto(PathPlanner.loadPathGroup( pathFile.nameWithoutExtension, ConfigConstants.autoConstraints)))
            }
        }

        SmartDashboard.putData("Auto Chooser", autoChooser)

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
        liftDrive.onTrue(SetElevator(PhysicalConstants.elevatorExtensionDrive, PhysicalConstants.elevatorAngleDrive, false))
        // This 0.20 is in place of a feedforward
        liftConeHigh.onTrue(SetElevator(PhysicalConstants.elevatorExtensionConeHigh, PhysicalConstants.elevatorAngleConeHigh, false))
        liftCubeHigh.onTrue(SetElevator(PhysicalConstants.elevatorExtensionCubeHigh, PhysicalConstants.elevatorAngleCubeHigh, false))
        liftMid.onTrue(SetElevator(PhysicalConstants.elevatorExtensionMid, PhysicalConstants.elevatorAngleMid, false))
        liftLow.onTrue(SetElevator(PhysicalConstants.elevatorExtensionLow, PhysicalConstants.elevatorAngleLow, false))
        liftIntakeDown.onTrue(SetElevator(PhysicalConstants.elevatorExtensionConeTippedIntake, PhysicalConstants.elevatorAngleConeTippedIntake, false))
        liftIntakeCube.onTrue(SetElevator(PhysicalConstants.elevatorExtensionCubeIntake, PhysicalConstants.elevatorAngleCubeIntake, false))
        liftIntakeCone.onTrue(SetElevator(PhysicalConstants.elevatorExtensionConeUpIntake, PhysicalConstants.elevatorAngleConeUpIntake, false))
    }

    fun getAuto(): Command? {
        return autoChooser.selected
    }

    fun getBrakePos(): Boolean {
        return driverController.xButton
    }

    fun getSlow(): Double {
        return driverController.rightTriggerAxis
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