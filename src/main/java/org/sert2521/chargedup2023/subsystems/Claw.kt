package org.sert2521.chargedup2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.ElectronicIDs

object Claw : SubsystemBase() {
    private val motor = CANSparkMax(ElectronicIDs.clawMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    private var currentSpeed = 0.0
    private var prev = 0

    init {
        motor.idleMode = CANSparkMax.IdleMode.kBrake
    }

    override fun periodic() {
        if (currentSpeed == 0.0){
            if (prev != 20) {
                motor.setSmartCurrentLimit(20)
            }
            motor.set(-0.1)
        } else {

            // Put in constants
            if (RobotController.getBatteryVoltage() <= ConfigConstants.preBrownOutVoltage) {
                if (prev != 20) {
                    motor.setSmartCurrentLimit(20)
                }
            } else {
                if (prev != 45) {
                    motor.setSmartCurrentLimit(45)
                }
            }

        }
    }

    fun setMotor(speed:Double){
        motor.set(speed)
        currentSpeed = speed
    }

    fun stop(){
        motor.stopMotor()
        currentSpeed = 0.0
    }

}