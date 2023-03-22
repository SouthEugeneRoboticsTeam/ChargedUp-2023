package org.sert2521.chargedup2023.commands

import edu.wpi.first.math.MathUtil.clamp
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import org.sert2521.chargedup2023.Input
import org.sert2521.chargedup2023.Output
import org.sert2521.chargedup2023.PhysicalConstants
import org.sert2521.chargedup2023.TunedConstants
import org.sert2521.chargedup2023.subsystems.Drivetrain
import java.lang.Math.PI
import kotlin.math.abs

class VisionAlignSubstation : JoystickCommand() {
    private val positionPID = PIDController(TunedConstants.swerveAlignDistanceP, TunedConstants.swerveAlignDistanceI, TunedConstants.swerveAlignDistanceD)
    private val anglePID = PIDController(TunedConstants.swerveSubstationAlignAngleP, TunedConstants.swerveSubstationAlignAngleI, TunedConstants.swerveSubstationAlignAngleD)

    init {
        addRequirements(Drivetrain)

        anglePID.enableContinuousInput(0.0, 2 * PI)

        positionPID.setTolerance(TunedConstants.visionSubstationPositionTolerance)
        anglePID.setTolerance(TunedConstants.visionSubstationAngleTolerance)
    }

    override fun initialize() {
        super.initialize()

        Drivetrain.setVisionStandardDeviations()
        positionPID.reset()
        anglePID.reset()
        LedIdle().schedule()
    }

    override fun execute() {
        val pose = Drivetrain.getVisionPose()

        // Maybe move to constants
        val angleOffset = when (Input.getColor()) {
            DriverStation.Alliance.Blue -> 0.0
            DriverStation.Alliance.Red -> PI
            DriverStation.Alliance.Invalid -> 0.0
        }

        // Moving the x on the stick will affect the rate of change of the y
        if (!positionPID.atSetpoint() || !anglePID.atSetpoint()) {
            val t = clamp((abs(pose.x - PhysicalConstants.substationX) - PhysicalConstants.substationCloseAngleAtDistance.second) / (PhysicalConstants.substationFarAngleAtDistance.second - PhysicalConstants.substationCloseAngleAtDistance.second), 0.0, 1.0)
            val angleTarget = t * (PhysicalConstants.substationFarAngleAtDistance.first - PhysicalConstants.substationCloseAngleAtDistance.first) + PhysicalConstants.substationCloseAngleAtDistance.first

            Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(positionPID.calculate(pose.x, PhysicalConstants.substationX), readJoystick().y, anglePID.calculate(pose.rotation.radians, angleTarget + angleOffset), pose.rotation))
            Output.visionHappy = false
        } else {
            // To update stuff
            val t = clamp((abs(pose.x - PhysicalConstants.substationX) - PhysicalConstants.substationCloseAngleAtDistance.second) / (PhysicalConstants.substationFarAngleAtDistance.second - PhysicalConstants.substationCloseAngleAtDistance.second), 0.0, 1.0)
            val angleTarget = t * (PhysicalConstants.substationFarAngleAtDistance.first - PhysicalConstants.substationCloseAngleAtDistance.first) + PhysicalConstants.substationCloseAngleAtDistance.first

            positionPID.calculate(pose.y, PhysicalConstants.substationX)
            anglePID.calculate(pose.rotation.radians, angleTarget + angleOffset)

            Drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, readJoystick().y, 0.0, pose.rotation))
            Output.visionHappy = true
        }
        if (Output.visionHappy){
            LedSolid(60, 255,255).schedule()
        }else{
            LedSolid(10, 255,255).schedule()
        }
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.setVisionAlignDeviations()
        Output.visionHappy = false
        Drivetrain.stop()
        LedIdle().schedule()
    }
}