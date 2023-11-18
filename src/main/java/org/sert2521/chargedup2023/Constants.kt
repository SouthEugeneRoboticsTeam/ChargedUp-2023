package org.sert2521.chargedup2023

import edu.wpi.first.math.MatBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import kotlin.math.*

class SwerveModuleData(val position: Translation2d, val powerMotorID: Int, val angleMotorID: Int, val angleEncoderID: Int, val angleOffset: Double, val inverted: Boolean)

object PhysicalConstants {
    const val halfSideLength = 0.286378246381
    const val powerEncoderMultiplierPosition = PI * 0.1016 / 8.14
    const val powerEncoderMultiplierVelocity = powerEncoderMultiplierPosition / 60.0
    const val angleEncoderMultiplier = 0.01745329251
}
object TunedConstants {
    const val swervePowerS = 0.3
    const val swervePowerV = 3.0
    const val swervePowerA = 0.0

    const val swervePowerP = 2.0
    const val swervePowerI = 0.0
    const val swervePowerD = 0.0

    const val swerveAngleP = 0.5
    const val swerveAngleI = 0.0
    const val swerveAngleD = 0.0

    val encoderDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(1.0, 1.0, 0.01)
    val defaultVisionDeviations: Matrix<N3, N1> = MatBuilder(Nat.N3(), Nat.N1()).fill(1.0, 1.0, 1000.0)
}

object ConfigConstants {
    const val drivetrainOptimized = true
    const val powerDeadband = 0.075
    const val rotDeadband = 0.075

    const val driveSpeed = 3.5
    const val driveSpeedup = 2.75
    const val rotSpeed = 3.5
    const val rotSpeedup = 2.75

    const val driveSpeedupChangeSpeed = 6.5
    const val driveSlowdownChangeSpeed = 36.0
}

object ElectronicIDs {
    val swerveModuleData = listOf(
        SwerveModuleData(Translation2d(PhysicalConstants.halfSideLength, -PhysicalConstants.halfSideLength), 4, 5, 14, -2.27 + PI / 2 + 4.62 + 1.54 - PI / 2, true),
        SwerveModuleData(Translation2d(-PhysicalConstants.halfSideLength, -PhysicalConstants.halfSideLength), 1, 2, 16, -1.63 - PI + 4.79 + 1.61 - PI / 2, true),
        SwerveModuleData(Translation2d(PhysicalConstants.halfSideLength, PhysicalConstants.halfSideLength), 12, 11, 13, -0.76 + PI / 2 - 1.43 + 1.57 - PI / 2, true),
        SwerveModuleData(Translation2d(-PhysicalConstants.halfSideLength, PhysicalConstants.halfSideLength), 7, 8, 15, -4.10 - PI / 2 + 5.12 + 1.75 - PI / 2, true)
    )
}
