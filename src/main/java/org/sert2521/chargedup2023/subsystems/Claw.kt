package org.sert2521.chargedup2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ElectronicIDs

object Claw : SubsystemBase() {
    val motor = CANSparkMax(ElectronicIDs.clawMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)


    fun setMotor(speed:Double){
        motor.set(speed)
    }

    fun stop(){
        motor.stopMotor()
    }

}