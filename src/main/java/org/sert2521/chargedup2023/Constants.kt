package org.sert2521.chargedup2023

import com.pathplanner.lib.PathConstraints
import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.sert2521.chargedup2023.commands.ClawIntake
import org.sert2521.chargedup2023.commands.GamePieces
import org.sert2521.chargedup2023.commands.SetElevator
import org.sert2521.chargedup2023.subsystems.Claw
import kotlin.math.PI

class SwerveModuleData(val position: Translation2d, val powerMotorID: Int, val angleMotorID: Int, val angleEncoderID: Int, val angleOffset: Double, val inverted: Boolean)

object PhysicalConstants {
    const val elevatorExtensionConversion = ((8.375) - (40.5625)) / (-1.071426391601562 - 70.07333374023438) / 100.0
    const val elevatorAngleConversion = -2 * PI
    const val elevatorFlipOffset = 0.085541770703388 - PI
    const val elevatorAngleOffset = -0.638255487307134

    const val elevatorExtensionTop = 0.198848411440849
    const val elevatorExtensionBottom = 0.226211532950401 / 100.0
    const val elevatorExtensionMinAngle = 0.05

    const val elevatorAngleTop = 0.95//1.05//1.09
    const val elevatorAngleBottom = 0.02//-0.01

    const val halfSideLength = 0.286378246381

    const val powerEncoderMultiplierPosition = PI * 0.1016 / 8.14
    const val powerEncoderMultiplierVelocity = PI * 0.1016 / (8.14 * 60)

    const val angleEncoderMultiplier = 0.01745329251

    val tagPose = Pose3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0))
    val cameraTrans = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0))
}

object TunedConstants {
    const val elevatorExtensionP = 100.0
    const val elevatorExtensionI = 0.0
    const val elevatorExtensionD = 0.0

    const val elevatorExtensionMaxV = 0.7
    const val elevatorExtensionMaxA = 0.7

    const val elevatorExtensionMinAngleTarget = 0.1
    const val elevatorExtensionTolerance = 0.015

    const val elevatorAngleP = 60.0
    const val elevatorAngleI = 0.0
    const val elevatorAngleD = 0.0

    const val elevatorAngleMaxV = 5.0
    const val elevatorAngleMaxA = 9.0

    const val elevatorAngleTolerance = 0.025

    // Sysid these all
    const val swervePowerS = 0.3
    const val swervePowerV = 3.0
    const val swervePowerA = 0.0

    // Sysid these all
    const val swervePowerP = 2.0
    const val swervePowerI = 0.0
    const val swervePowerD = 0.0

    const val swerveAngleP = 0.5
    const val swerveAngleI = 0.0
    const val swerveAngleD = 0.0

    const val swerveAutoDistanceP = 0.5
    const val swerveAutoDistanceI = 0.0
    const val swerveAutoDistanceD = 0.0

    const val swerveAutoAngleP = 3.0
    const val swerveAutoAngleI = 0.0
    const val swerveAutoAngleD = 0.0

    const val balanceAngleP = 0.0
    const val balanceAngleI = 0.0
    const val balanceAngleD = 0.0

    const val balanceAngleMaxV = 0.0
    const val balanceAngleMaxA = 0.0

    const val balanceAngleTolerance = 0.0
    const val balanceAngleStart = 0.0

    val stateDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0)
    val globalDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0)
}

object ConfigConstants {
    const val extensionResetVoltage = -1.0

    const val drivetrainOptimized = true

    const val powerDeadband = 0.1
    const val rotDeadband = 0.1
    const val joystickDeadband = 0.1

    const val driveSpeed = 1.5
    const val rotSpeed = 2.0

    const val joystickChangeSpeed = 0.4

    val eventMap = mapOf("Elevator Drive" to SetElevator(0.0, 0.93, true),
        "Elevator Cone Outtake" to SetElevator(0.20, 0.7, true),
        "Claw Cone Outtake" to ClawIntake(GamePieces.CONE, true).withTimeout(1.0),
        "Elevator Cube Intake" to SetElevator(0.0, 0.05, true),
        "Claw Cube Intake" to ClawIntake(GamePieces.CUBE, false),
        "Claw Stop" to InstantCommand({  }, Claw),
        "Elevator Cube Outtake" to SetElevator(0.18, 0.55, true).withTimeout(1.0),
        "Claw Cube Outtake" to ClawIntake(GamePieces.CUBE, true))
    val autoConstraints = PathConstraints(0.5, 0.5)
}

object ElectronicIDs {
    const val clawMotorId = 9
    const val elevatorMotorOne = 6
    const val elevatorMotorTwo = 10
    const val elevatorAngleMotor = 5

    const val elevatorEncoder = 5

    const val elevatorUpperExtension = 2
    const val elevatorLowerExtension = 1

    val swerveModuleData = mutableListOf(
        SwerveModuleData(Translation2d(PhysicalConstants.halfSideLength, -PhysicalConstants.halfSideLength), 4, 3, 14, -2.27 + PI / 2 + 4.62, true),
        SwerveModuleData(Translation2d(-PhysicalConstants.halfSideLength, -PhysicalConstants.halfSideLength), 2, 1, 16, -1.63 - PI + 4.79, true),
        SwerveModuleData(Translation2d(PhysicalConstants.halfSideLength, PhysicalConstants.halfSideLength), 12, 11, 13, -0.76 + PI / 2 - 1.43, true),
        SwerveModuleData(Translation2d(-PhysicalConstants.halfSideLength, PhysicalConstants.halfSideLength), 7, 8, 15, -4.10 - PI / 2 + 4.95, true))

    const val camName = ""

}
