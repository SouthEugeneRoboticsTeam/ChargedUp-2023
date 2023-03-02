package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import org.sert2521.chargedup2023.commands.*
import org.sert2521.chargedup2023.commands.ClawIntake
import org.sert2521.chargedup2023.commands.GamePieces
import org.sert2521.chargedup2023.commands.SetElevator
import org.sert2521.chargedup2023.subsystems.Claw
import org.sert2521.chargedup2023.subsystems.Drivetrain
import kotlin.math.PI

object Input {
    // Replace with constants
    private val driverController = XboxController(0)
    private val gunnerController = Joystick(1)

    private val resetAngle = JoystickButton(driverController, 4)

    //private val intakeSetOne = JoystickButton(gunnerController, 15)
    private val intakeSetTwo = JoystickButton(gunnerController, 14)
    private val outtake = JoystickButton(gunnerController, 13)

    private val liftDrive = JoystickButton(gunnerController, 5)
    private val liftConeHigh = JoystickButton(gunnerController, 6)
    private val liftCubeHigh = JoystickButton(gunnerController, 7)
    private val liftMid = JoystickButton(gunnerController, 8)
    private val liftLow = JoystickButton(gunnerController, 9)
    private val liftIntakeTippedCone = JoystickButton(gunnerController, 10)
    private val liftIntakeCube = JoystickButton(gunnerController, 16)
    private val liftIntakeCone = JoystickButton(gunnerController, 15)
    private val liftSingleSubstation = JoystickButton(gunnerController, 11)

    private var lastPiece = GamePieces.CUBE

    private val autoChooser = SendableChooser<MutableList<PathPlannerTrajectory?>?>()
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

    private val ledCube = JoystickButton(gunnerController, 3)

    private val ledCone = JoystickButton(gunnerController, 4)


    init {
        // Put these strings in constants maybe
        autoChooser.setDefaultOption("Nothing", null)
        for (name in ConfigConstants.pathNames) {
            autoChooser.addOption(name, PathPlanner.loadPathGroup(name, ConfigConstants.autoConstraints))
        }

        autoChooser.addOption("1 Piece Balance Middle", mutableListOf(null))

        SmartDashboard.putData("Auto Chooser", autoChooser)

        // Replace numbers with constants
        resetAngle.onTrue(InstantCommand({ Drivetrain.setNewPose(Pose2d()) }))

        //Intaking a cone is the same as outtaking a cube
        //intakeSetOne.whileTrue(ClawIntake(GamePieces.CONE, false))
        //intakeSetOne.onTrue(InstantCommand({ lastPiece = GamePieces.CUBE }))

        intakeSetTwo.whileTrue(ClawIntake(GamePieces.CUBE, false))
        intakeSetTwo.onTrue(InstantCommand({ lastPiece = GamePieces.CONE }))

        var clawCommandDirection: ClawIntake? = null

        outtake.onTrue(InstantCommand({ clawCommandDirection = ClawIntake(lastPiece, true); clawCommandDirection?.schedule() }))
        outtake.onFalse(InstantCommand({ clawCommandDirection?.cancel(); clawCommandDirection = null }))

        liftDrive.onTrue(SetElevator(PhysicalConstants.elevatorExtensionDrive, PhysicalConstants.elevatorAngleDrive, false))
        liftConeHigh.onTrue(SetElevator(PhysicalConstants.elevatorExtensionConeHigh, PhysicalConstants.elevatorAngleConeHigh, false))
        liftCubeHigh.onTrue(SetElevator(PhysicalConstants.elevatorExtensionCubeHigh, PhysicalConstants.elevatorAngleCubeHigh, false))
        liftMid.onTrue(SetElevator(PhysicalConstants.elevatorExtensionMid, PhysicalConstants.elevatorAngleMid, false))
        liftLow.onTrue(SetElevator(PhysicalConstants.elevatorExtensionLow, PhysicalConstants.elevatorAngleLow, false))
        liftIntakeTippedCone.onTrue(SetElevator(PhysicalConstants.elevatorExtensionConeTippedIntake, PhysicalConstants.elevatorAngleConeTippedIntake, false))
        liftIntakeCube.onTrue(SetElevator(PhysicalConstants.elevatorExtensionCubeIntake, PhysicalConstants.elevatorAngleCubeIntake, false))
        liftIntakeCone.onTrue(SetElevator(PhysicalConstants.elevatorExtensionConeUpIntake, PhysicalConstants.elevatorAngleConeUpIntake, false))
        liftSingleSubstation.onTrue(SetElevator(PhysicalConstants.elevatorExtensionSingleSubstation, PhysicalConstants.elevatorAngleSingleSubstation, false))

        val currentCubePattern = LedFlash(PhysicalConstants.ledPurpleHSV[0], PhysicalConstants.ledPurpleHSV[1], PhysicalConstants.ledPurpleHSV[2], 1.0)
        val currentConePattern = LedFlash(PhysicalConstants.ledYellowHSV[0], PhysicalConstants.ledYellowHSV[1], PhysicalConstants.ledYellowHSV[2], 1.0)

        ledCube.toggleOnTrue(currentCubePattern)
        ledCone.toggleOnTrue(currentConePattern)
    }

    fun getAuto(): Command? {
        val selected = autoChooser.selected
        return if (selected == null) {
            null
        } else {
            if (selected[0] == null) {
                SequentialCommandGroup(
                    InstantCommand({ Drivetrain.setNewPose(Pose2d(0.0, 0.0, Rotation2d(PI))) }),
                    SetElevator(PhysicalConstants.elevatorExtensionDrive, PhysicalConstants.elevatorAngleDrive, true),
                    SetElevator(PhysicalConstants.elevatorExtensionConeHigh, PhysicalConstants.elevatorAngleConeHigh, true),
                    ClawIntake(GamePieces.CONE, true).withTimeout(0.37),
                    InstantCommand({  }, Claw),
                    SetElevator(PhysicalConstants.elevatorExtensionDrive, PhysicalConstants.elevatorAngleDrive, true),
                    OntoChargeStation(Translation2d(1.0, 0.0)),
                    DriveInDirection(Translation2d(1.0, 0.0)).withTimeout(3.3),
                    OntoChargeStation(Translation2d(-1.0, 0.0)),
                    DriveUpChargeStation().withTimeout(1.4),
                    Balance())
            } else {
                autoBuilder.fullAuto(selected)
            }
        }
    }

    fun getBrakePos(): Boolean {
        return driverController.xButton
    }

    fun getFast(): Double {
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

    fun getAutoAlign(): Boolean {
        return driverController.aButton
    }
}