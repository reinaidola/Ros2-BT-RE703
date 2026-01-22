#ifndef PID_H
#define PID_H

#include <math.h>

class PID
{
private:
    float min_val_, max_val_;
    float KP, KI, KD;

    float kp, kpT,
        ki, kiT, kd, kdT;
    float angular_vel_Prev;

    struct e
    {
        float proportional;
        float integral;
        float derivative;
        float u;
        float previous;
    } err;

    struct p
    {
        float KP, KI, KD;

        float kp, kpT,
            ki, kiT, kd, kdT;
    } params;
    float lowpass_filt = 0;
    float lowpass_prev = 0;
    float radian = 0;
    float eProportional;
    float deg2target = 0;

    float encPrev;

    float angular_vel_Filt = 0;
    float angular_vel = 0;

    float PPR = 0;

    float error_integral;
    float error_previous;

    float eIntegral = 0;
    float prevError = 0;

public:
    PID(float MIN_VAL, float MAX_VAL, float kp_, float ki_, float kd_) : min_val_(MIN_VAL),
                                                                         max_val_(MAX_VAL),
                                                                         KP(kp_),
                                                                         KI(ki_),
                                                                         KD(kd_)
    {
    }
    void parameter(float kp_, float ki_, float kd_)
    {

        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

    void parameterT(float kp_, float ki_, float kd_)
    {
        kpT = kp_;
        kiT = ki_;
        kdT = kd_;
    }
    void ppr_total(float total_ppr)
    {
        PPR = total_ppr;
    }

    float control_angle(float target, float enc, float pwm, float deltaT)
    {
        deg2target = target / 360 * PPR;

        err.proportional = deg2target - enc;

        err.integral += err.proportional * deltaT;

        err.derivative = (err.proportional - err.previous);
        err.previous = err.proportional;

        err.u = KP * err.proportional + KI * err.integral + KD * err.derivative;

        return fmax(-1 * pwm, fmin(err.u, pwm));
    }
    float control_angle_speed(float target_angle, float target_speed, float enc, float deltaT)
    {
        deg2target = target_angle / PPR;

        err.proportional = deg2target - enc;

        err.integral += err.proportional * deltaT;

        err.derivative = (err.proportional - err.previous);
        err.previous = err.proportional;

        err.u = KP * err.proportional + KI * err.integral + KD * err.derivative;
        return control_speed(target_speed, enc, deltaT);
    }
    float control_base_rotation(float error, float deltaT)
    {
        err.proportional = error;

        if (err.proportional > 3.14)
        {
            err.proportional -= 6.28;
        }
        else if (err.proportional < -3.14)
        {
            err.proportional += 6.28;
        }
        err.integral += err.proportional * deltaT;

        err.derivative = (err.proportional - err.previous) / deltaT;
        err.previous = err.proportional;

        float u = KP * err.proportional + KI * err.integral + KD * err.derivative;

        return fmax(min_val_, fmin(u, max_val_));
    }
    float control_base_distance(float error, float deltaT)
    {

        err.proportional = error;

        err.integral += err.proportional * deltaT;

        err.derivative = (err.proportional - err.previous) / deltaT;
        err.previous = err.proportional;

        float u = KP * err.proportional + KI * err.integral + KD * err.derivative;

        return fmax(min_val_, fmin(u, max_val_));
    }
    float control_speed(float target, float enc, float deltaT)
    {
        radian = (enc - encPrev) / deltaT;
        encPrev = enc;
        angular_vel = radian / PPR;

        angular_vel_Filt = 0.854 * angular_vel_Filt + 0.0728 * angular_vel + 0.0728 * angular_vel_Prev;
        angular_vel_Prev = angular_vel;

        err.proportional = target - angular_vel_Filt;

        err.integral += err.proportional * deltaT;
        fmax(-125, fmin(err.integral, 125));
        err.derivative = (err.proportional - err.previous) / deltaT;
        err.previous = err.proportional;

        err.u = KP * err.proportional + KI * err.integral + KD * err.derivative;
        return fmax(min_val_, fmin(err.u, max_val_));
    }
    float control_default(float target, float curr, float deltaT)
    {
        float error = target - curr;

        error_integral += error * deltaT;

        float error_derivative = (error - error_previous) / deltaT;
        error_previous = error;

        float u = KP * error + KI * error_integral + KD * error_derivative;
        return fmax(min_val_, fmin(u, max_val_));
    }

    float get_filt_vel()
    {
        return angular_vel_Filt;
    }
};

#endif