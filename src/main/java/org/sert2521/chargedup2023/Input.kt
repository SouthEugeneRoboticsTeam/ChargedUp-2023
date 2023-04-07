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
import edu.wpi.first.wpilibj.XboxController.Button
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
    private val gunnerController = Joystick(1)

    private val resetAngle = JoystickButton(driverController, 4)
    private val slowButton = JoystickButton(driverController, 5)
    private val coneAlignButton = JoystickButton(driverController, 6)
    private val coneAlignConeRot = JoystickButton(driverController, 1)

    private val outtake = JoystickButton(gunnerController, 13)
    private val intake = JoystickButton(gunnerController, 14)

    private val liftDrive = JoystickButton(gunnerController, 5)
    private val liftConeHigh = JoystickButton(gunnerController, 6)
    private val liftCubeHigh = JoystickButton(gunnerController, 7)
    private val liftMid = JoystickButton(gunnerController, 8)
    private val liftLow = JoystickButton(gunnerController, 9)
    private val liftIntakeTippedCone = JoystickButton(gunnerController, 10)
    private val liftIntakeCube = JoystickButton(gunnerController, 16)
    private val liftIntakeCone = JoystickButton(gunnerController, 15)
    private val liftSingleSubstation = JoystickButton(gunnerController, 11)

    // Has to do this function thing so the robot can do andThen(auto) more than once
    private val autoChooser = SendableChooser<() -> Command?>()
    private val autoBuilder = SwerveAutoBuilder(
        Drivetrain::getPose,
        { Drivetrain.setNewPose(it); Drivetrain.setNewVisionPose(it) },
        PIDConstants(TunedConstants.swerveAutoDistanceP, TunedConstants.swerveAutoDistanceI, TunedConstants.swerveAutoDistanceD),
        PIDConstants(TunedConstants.swerveAutoAngleP, TunedConstants.swerveAutoAngleI, TunedConstants.swerveAutoAngleD),
        Drivetrain::drive,
        ConfigConstants.eventMap,
        true,
        Drivetrain
    )

    private val ledCube = JoystickButton(gunnerController, 3)

    private val ledCone = JoystickButton(gunnerController, 4)

    var slowMode = false

    init {
        // Put these strings in constants maybe
        autoChooser.setDefaultOption("Nothing") { null }
        for (path in ConfigConstants.paths) {
            autoChooser.addOption(path.first) { autoBuilder.fullAuto(path.second) }
        }

        // Fix this nonsense
        autoChooser.addOption("Center 1 Over And Back Balance") { SequentialCommandGroup(
            InstantCommand({ Drivetrain.setNewPose(Pose2d(0.0, 0.0, Rotation2d(PI))) }),
            SetElevator(PhysicalConstants.elevatorExtensionDrive, PhysicalConstants.elevatorAngleDrive, true),
            SetElevator(PhysicalConstants.elevatorExtensionConeHigh, PhysicalConstants.elevatorAngleConeHigh, true).andThen(SetElevator(PhysicalConstants.elevatorExtensionConeHigh, PhysicalConstants.elevatorAngleConeHigh, false).withTimeout(0.25)),
            ClawIntake(0.7).withTimeout(0.4),
            InstantCommand({ }, Claw),
            SetElevator(PhysicalConstants.elevatorExtensionSingleSubstation, PhysicalConstants.elevatorAngleSingleSubstation, true),
            OntoChargeStation(Translation2d(1.5, 0.0)),
            DriveInDirection(Translation2d(1.5, 0.0)).withTimeout(2.7),
            DriveInDirection(Translation2d(-1.5, 0.0)).withTimeout(2.1),
            Balance()) } // No

        SmartDashboard.putData("Auto Chooser", autoChooser)

        // Replace numbers with constants

        // Clamp to reasonable positions
        resetAngle.onTrue(InstantCommand({
            Drivetrain.setNewPose(Pose2d())
            Drivetrain.setNewVisionPose(Pose2d())
        }))
        coneAlignButton.whileTrue(VisionAlignCone())
        //coneAlignConeRot.whileTrue(VisionAlignConeRot())
        Trigger { driverController.leftTriggerAxis > 0.5 }.whileTrue(VisionAlignSubstation())
        Trigger { driverController.rightTriggerAxis > 0.5 }.whileTrue(JoystickDrive(false))

        // Intaking a cone is the same as outtaking a cube
        //intakeSetOne.whileTrue(ClawIntake(1.0))

        outtake.whileTrue(ClawIntake(1.0))

        intake.whileTrue(ClawIntake(-1.0))

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

        slowButton.onTrue(InstantCommand({ slowMode = !slowMode }))
    }

    fun getAuto(): Command? {
        val selected = autoChooser.selected
        return if (selected == null) {
            null
        } else {
            selected()
        }
    }

    fun getBrakePos(): Boolean {
        return driverController.xButton
    }

    // Rename fast stuff because it actually slows it
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

    fun getSlider(): Double {
        return gunnerController.getRawAxis(3)
    }

    fun getColor(): Alliance {
        return DriverStation.getAlliance()
    }

    // This kinda violates the spirit of Input and Output
    fun rumble(amount: Double) {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount)
    }
}