package org.sert2521.chargedup2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase

//The Dominion of Milo.

object Elevator : SubsystemBase() {
    val extendMotor = CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless)

    val OtherMotor = CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless)

    val yogurt = Encoder(1,0)

    val yogibear = extendMotor.encoder

    val fozzybear = OtherMotor.encoder

    fun Lifting(speed: Double) {
        extendMotor.set(speed)
    }
    fun Dropping(speed2 : Double){
        OtherMotor.set(speed2)
    }

    fun DondeEstaElSensor(): Double {
        return fozzybear.position
    }
    fun DondeEstaElSensorDos(): Double {
        return yogibear.position
    }
    fun DondeEstaElSensorTres(): Double {
        return yogurt.distance
    }

}
