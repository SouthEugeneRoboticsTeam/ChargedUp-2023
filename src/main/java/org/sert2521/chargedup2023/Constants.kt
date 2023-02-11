package org.sert2521.chargedup2023

import kotlin.math.PI

object PhysicalConstants {
    const val elevatorExtensionConversion = ((8.375) - (40.5625)) / (-1.071426391601562 - 70.07333374023438) / 100.0
    const val elevatorAngleConversion = 2.0 * PI / 2048.0

    const val elevatorExtensionTop = 18.807661056518555 / 100.0
    const val elevatorExtensionBottom = 0.226211532950401 / 100.0

    const val elevatorAngleTop = 0.9
    const val elevatorAngleBottom = 0.006902913545485
}

object TunedConstants {
    const val extensionResetSpeed = -0.1
    const val angleResetSpeed = -0.05

    const val elevatorExtensionP = 4.5
    const val elevatorExtensionI = 0.0
    const val elevatorExtensionD = 0.0

    const val elevatorAngleS = 0.0
    const val elevatorAngleG = 0.15
    const val elevatorAngleV = 0.0

    const val elevatorAngleP = 2.5
    const val elevatorAngleI = 0.0
    const val elevatorAngleD = 0.0
}

object ElectronicIDs {
    const val clawMotorId = 9
    const val elevatorMotorOne = 6
    const val elevatorMotorTwo = 10
    const val elevatorAngleMotor = 5

    const val elevatorEncoderA = 5
    const val elevatorEncoderB = 6

    const val elevatorUpperExtension = 2
    const val elevatorLowerExtension = 1

    const val elevatorLowerAngle = 3
}
