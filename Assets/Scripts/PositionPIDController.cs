/*
 * PositionPIDController.cs
 * Bart Trzynadlowski, 2024
 * 
 * Position PID controller. Outputs a directional throttle signal based on whether the vehicle is
 * ahead or behind of the target, measured relative to the vehicle's forward heading, which is
 * assumed to be constant.
 * 
 * In this sort of controller, Ki does not help and should be 0. E.g., Kp = 1, Ki = 0, Kd = 1. The
 * derivative term (change in error over time) becomes negative as the target is approached,
 * counteracting the signal and slowing the system down ahead of time without overshoot.
 * 
 * This implementation features a distance threshold outside of which the integral term is zeroed
 * out (although this still does not help with overshoot).
 */

using UnityEngine;

public class PositionPIDController : MonoBehaviour
{
    [SerializeField]
    private Transform _target;

    [SerializeField]
    private float _maximumAcceleration = 5;

    [SerializeField]
    private float _maximumSpeed = 3;

    [SerializeField]
    private float _Kp = 1.0f;

    [SerializeField]
    private float _Ki = 0.0f;

    [SerializeField]
    private float _Kd = 0.0f;

    [SerializeField]
    private float _integralActiveDistance = 5;

    // PID
    private float _prevError = 0;
    private float _integralError = 0;
    private bool _noPrevError = true;

    // Car state
    private float _velocity = 0;

    private void FixedUpdate()
    {
        float signal = UpdatePID();
        float acceleration = ThrottleToAcceleration(throttle: signal);
        _velocity = Mathf.Clamp(_velocity + acceleration * Time.fixedDeltaTime, min: -_maximumSpeed, max: _maximumSpeed);
        transform.position = transform.position + transform.forward * _velocity * Time.fixedDeltaTime;
    }

    private float UpdatePID()
    {
        float dt = Time.fixedDeltaTime;
        if (dt <= 0)
        {
            return 0;
        }

        float error = SignedPositionError();
        float direction = Mathf.Sign(error);
        error = Mathf.Abs(error);

        bool integralActive = error <= _integralActiveDistance;

        if (_noPrevError)
        {
            _prevError = error;
        }

        _integralError = integralActive ? (_integralError + dt * error) : 0;
        float derivativeError = (error - _prevError) / dt;
        float signal = _Kp * error + _Ki * _integralError + _Kd * derivativeError;

        Debug.Log($"signal={signal * direction}, error={error}, prevError={_prevError}, integralError={_integralError}, derivativeError={derivativeError}");

        _prevError = error;
        _noPrevError = false;

        return signal * direction;
    }

    private float SignedPositionError()
    {
        Vector3 position = Vector3.Dot(_target.position - transform.position, transform.forward) * transform.forward + transform.position;
        Vector3 delta = position - transform.position;
        float distance = delta.XZProject().magnitude;
        float sign = Mathf.Sign(Vector3.Dot(delta, transform.forward));
        return sign * distance;
    }

    private float ThrottleToAcceleration(float throttle)
    {
        return Mathf.Clamp(throttle, min: -1, max: 1) * _maximumAcceleration;
    }
}
