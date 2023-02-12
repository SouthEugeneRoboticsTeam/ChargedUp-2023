package org.sert2521.chargedup2023

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import org.sert2521.chargedup2023.PhysicalConstants.halfSideLength
import kotlin.math.PI

class SwerveModuleData(val position: Translation2d, val powerMotorID: Int, val angleMotorID: Int, val angleEncoderID: Int, val angleOffset: Double, val inverted: Boolean)

object PhysicalConstants {
    const val halfSideLength = 0.286378246381

    const val powerEncoderMultiplierPosition = 0.0000191464923893
    const val powerEncoderMultiplierVelocity = 0.000191464923893

    const val angleEncoderMultiplier = 0.01745329251

    val tagPose = Pose3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0))
    val cameraTrans = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0))
}

object TunedConstants {
    const val swervePowerS = 1.20983
    const val swervePowerV = 4.601311978
    const val swervePowerA = 0.159883071

    const val swervePowerP = 2.730003958
    const val swervePowerI = 0.0
    const val swervePowerD = 0.0

    const val swerveAngleP = 0.3
    const val swerveAngleI = 0.0
    const val swerveAngleD = 0.0

    val stateDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0)
    val globalDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0)
}

object ConfigConstants {
    const val drivetrainOptimized = true

    const val powerDeadband = 0.1
    const val rotDeadband = 0.1
    const val joystickDeadband = 0.1

    const val driveSpeed = 1.0
    const val rotSpeed = 1.0

    const val joystickChangeSpeed = 0.4
}

object ElectronicIDs {
    val swerveModuleData = mutableListOf(
        SwerveModuleData(Translation2d(halfSideLength, -halfSideLength), 4, 3, 14, -2.27 + PI, true),
        SwerveModuleData(Translation2d(-halfSideLength, -halfSideLength), 2, 1, 16, -1.63 - PI / 2, true),
        SwerveModuleData(Translation2d(halfSideLength, halfSideLength), 12, 11, 13, -0.76 + PI, true),
        SwerveModuleData(Translation2d(-halfSideLength, halfSideLength), 7, 8, 15, -4.10, true))

    const val camName = ""
}
