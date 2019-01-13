#include "PID.h"

/*
* TODO: Complete the PID class.
*/

#include <numeric>
#include <iostream>
#include <cmath>

PID::PID(): m_coeffs(3, 0), m_dp(3, 0.1) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    m_coeffs[0] = Kp;
    m_coeffs[1] = Ki;
    m_coeffs[2] = Kd;

    m_p_error = 0;
    m_i_error = 0;
    m_d_error = 0;
}

void PID::UpdateError(double cte)
{
    m_d_error = (cte - m_p_error);
    m_p_error = cte;
    m_i_error += cte;

    m_total_err += pow(cte, 2);

    if(m_enable_twiddle && m_iteration % m_tweedle_steps == 0)
    {
        double err = m_total_err / m_tweedle_steps;
        if(err < m_best_err)
        {   
            std::cout << "New best error: " << err << std::endl;
            m_dp[m_last_param_index] *= 1.1;
            m_last_param_index = (m_last_param_index + 1) % m_dp.size();
            m_increased = false;
            m_decreased = false;
            m_best_err = err;
        }

        if(!m_increased && !m_decreased)
        {
            m_coeffs[m_last_param_index] += m_dp[m_last_param_index];
            m_increased = true;
        }
        else if(m_increased && !m_decreased)
        {
            m_coeffs[m_last_param_index] -= 2 * m_dp[m_last_param_index];
            m_decreased = true;
        }
        else
        {
            m_coeffs[m_last_param_index] += m_dp[m_last_param_index];
            m_dp[m_last_param_index] *= 0.9;
            m_last_param_index = (m_last_param_index + 1) % m_dp.size();
            m_increased = false;
            m_decreased = false;
        }

        std::cout << "New coefficients: " << m_coeffs[0] << " " <<
            m_coeffs[1] << " " << m_coeffs[2] << std::endl;
        m_total_err = 0;
                
    }
    m_iteration++;
}

double PID::TotalError()
{
    return m_coeffs[0] * m_p_error + m_coeffs[1] * m_i_error + m_coeffs[2] * m_d_error;
}
