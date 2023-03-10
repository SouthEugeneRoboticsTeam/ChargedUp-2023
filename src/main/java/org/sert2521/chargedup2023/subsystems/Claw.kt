package org.sert2521.chargedup2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ElectronicIDs

object Claw : SubsystemBase() {
    private val motor = CANSparkMax(ElectronicIDs.clawMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)

    init {
        motor.setSmartCurrentLimit(45, 45)
    }

    fun setMotor(speed:Double){
        motor.set(speed)
    }

    fun stop(){
        motor.stopMotor()
    }

}