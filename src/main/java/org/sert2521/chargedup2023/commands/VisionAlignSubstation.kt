package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.Output
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.lang.Math.PI

class VisionAlignSubstation : JoystickCommand() {
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

        val color = Input.getColor()
        val yTarget = PhysicalConstants.colorToSubstation[color]
        val farAngleAtDistance = PhysicalConstants.colorToSubstationFarAngleAtDistance[color]
        val closeAngleAtDistance = PhysicalConstants.colorToSubstationCloseAngleAtDistance[color]
        if (yTarget != null && farAngleAtDistance != null && closeAngleAtDistance != null) {
            val t = clamp((pose.x - closeAngleAtDistance.second) / (farAngleAtDistance.second - closeAngleAtDistance.second), 0.0, 1.0)
            val angleTarget = t * (farAngleAtDistance.first - closeAngleAtDistance.first) + closeAngleAtDistance.first

            // Moving the x on the stick will affect the rate of change of the y
            Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(readJoystick().x, positionPID.calculate(pose.x, yTarget), anglePID.calculate(pose.rotation.radians, angleTarget), pose.rotation))
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