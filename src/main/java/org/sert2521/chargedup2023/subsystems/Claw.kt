package org.sert2521.chargedup2023.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.ElectronicIDs
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants

// Put stuff in constants
object Claw : SubsystemBase() {
    private val motor = CANSparkMax(ElectronicIDs.clawMotorId, CANSparkMaxLowLevel.MotorType.kBrushless)
    private val encoder = motor.encoder

    private var currentSpeed = 0.0
    private var prev = 0

    private val fullFilter = Debouncer(TunedConstants.clawDebounce)
    var clawFull = false
        private set

    init {
        motor.idleMode = CANSparkMax.IdleMode.kBrake

        encoder.velocityConversionFactor = PhysicalConstants.clawVelocityConversion
    }

    override fun periodic() {
        if (currentSpeed == 0.0){
            if (prev != 20) {
                motor.setSmartCurrentLimit(20)
            }

            motor.set(-0.1)
            currentSpeed = -0.1
        } else {
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

        clawFull = if (-currentSpeed >= TunedConstants.clawIntakeTryPower) {
            fullFilter.calculate(encoder.velocity <= TunedConstants.clawIntakeStoppedSpeed)
        } else if (currentSpeed >= TunedConstants.clawOuttakeTryPower) {
            fullFilter.calculate(false)
        } else {
            fullFilter.calculate(clawFull)
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