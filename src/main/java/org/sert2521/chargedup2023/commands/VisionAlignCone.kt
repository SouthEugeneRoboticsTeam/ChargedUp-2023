package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.Output
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.lang.Math.PI
import kotlin.math.abs

class VisionAlignCone : JoystickCommand() {
    private val positionPID = PIDController(TunedConstants.swerveAlignDistanceP, TunedConstants.swerveAlignDistanceI, TunedConstants.swerveAlignDistanceD)
    private val anglePID = PIDController(TunedConstants.swerveAlignAngleP, TunedConstants.swerveAlignAngleI, TunedConstants.swerveAlignAngleD)

    init {
        addRequirements(Drivetrain)

        anglePID.enableContinuousInput(0.0, 2 * PI)

        positionPID.setTolerance(TunedConstants.visionPositionTolerance)
        anglePID.setTolerance(TunedConstants.visionAngleTolerance)
    }

    override fun initialize() {
        super.initialize()

        Drivetrain.setVisionStandardDeviations()
        positionPID.reset()
        anglePID.reset()
    }

    override fun execute() {
        val pose = Drivetrain.getVisionPose()

        val xTarget = PhysicalConstants.conePoints.maxBy { abs(it - pose.x) }
        val angleTarget = PhysicalConstants.colorToConeAngle[Input.getColor()]
        if (angleTarget != null) {
            // Moving the x on the stick will affect the rate of change of the y
            Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(positionPID.calculate(pose.x, xTarget), readJoystick().y, anglePID.calculate(pose.rotation.radians, angleTarget), pose.rotation))
        } else {
            positionPID.reset()
            anglePID.reset()
        }

        Output.visionHappy = positionPID.atSetpoint() && anglePID.atSetpoint()
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.setVisionAlignDeviations()
        Output.visionHappy = false
        Drivetrain.stop()
    }
}