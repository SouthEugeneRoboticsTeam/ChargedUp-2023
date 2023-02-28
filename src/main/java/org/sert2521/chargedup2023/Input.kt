package org.sert2521.chargedup2023

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import org.sert2521.chargedup2023.commands.*
import org.sert2521.chargedup2023.commands.ClawIntake
import org.sert2521.chargedup2023.commands.GamePieces
import org.sert2521.chargedup2023.commands.SetElevator
import org.sert2521.chargedup2023.subsystems.Drivetrain

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
    private val liftSingleSubstation = JoystickButton(gunnerController, 12)

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

    private val ledCube = JoystickButton(gunnerController, 3)

    private val ledCone = JoystickButton(gunnerController, 4)


    init {
        // Put these strings in constants maybe
        autoChooser.setDefaultOption("Nothing", null)
        for (name in ConfigConstants.pathNames) {
            autoChooser.addOption(name, autoBuilder.fullAuto(PathPlanner.loadPathGroup(name, ConfigConstants.autoConstraints)))
        }

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

        // Add feedforward
        liftDrive.onTrue(SetElevator(PhysicalConstants.elevatorExtensionDrive, PhysicalConstants.elevatorAngleDrive, false))
        // This 0.20 is in place of a feedforward
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

    fun getAutoAlign(): Boolean {
        return driverController.aButton
    }
}