package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.CommandBase
import org.sert2521.chargedup2023.ConfigConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Elevator

class InitElevator : CommandBase() {
    // Fix trapezoidal control to be in line with SetElevator
    private val anglePID = ProfiledPIDController(TunedConstants.elevatorAngleP, TunedConstants.elevatorAngleI, TunedConstants.elevatorAngleD, TrapezoidProfile.Constraints(TunedConstants.elevatorAngleDownMaxV, TunedConstants.elevatorAngleDownMaxAByAngle))

    init {
        addRequirements(Elevator)
    }

    override fun initialize() {
        val angle = Elevator.angleMeasure()
        anglePID.reset(angle)
    }

    override fun execute() {
        if (!Elevator.angleInited) {
            Elevator.setAngle(anglePID.calculate(Elevator.angleMeasure(), Elevator.angleMeasure()))
        } else {
            Elevator.setAngle(anglePID.calculate(Elevator.angleMeasure(), ConfigConstants.resetAngle))
        }
    }

    override fun isFinished(): Boolean {
        return Elevator.extensionInited && Elevator.angleInited
    }

    override fun end(interrupted: Boolean) {
        Elevator.stop()
    }
}