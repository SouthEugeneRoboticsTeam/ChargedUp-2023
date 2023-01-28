package org.sert2521.chargedup2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase

//The Dominion of Milo.

object Elevator : SubsystemBase() {
    private val extendMotor = CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val angleMotor = CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless)

    private val trueAngleEncoder = Encoder(1,0)

    private val extendEncoder = extendMotor.encoder

    private val angleEncoder = angleMotor.encoder

    fun extend(speed: Double) {
        extendMotor.set(speed)
    }
    fun retract(speed2 : Double){
        angleMotor.set(speed2)
    }

    fun angleMeasure(): Double {
        return angleEncoder.position
    }
    fun extensionMeasure(): Double {
        return extendEncoder.position
    }
    fun trueMeasure(): Double {
        return trueAngleEncoder.distance
    }

}
