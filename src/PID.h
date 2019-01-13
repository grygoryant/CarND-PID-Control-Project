#ifndef PID_H
#define PID_H

#include <chrono>
#include <vector>

class PID
{
public:
	/*
  * Errors
  */
	double m_p_error;
	double m_i_error;
	double m_d_error;

	/*
  * Coefficients
  */
	std::vector<double> m_coeffs;

	bool m_enable_twiddle{ false };
	std::vector<double> m_dp;
	double m_best_err{ std::numeric_limits<double>::max() };
	size_t m_last_param_index{ 0 };
	size_t m_iteration{ 0 };
	double m_total_err{ 0 };
	bool m_increased{ false };
	bool m_decreased{ false };
	size_t m_tweedle_steps{ 50 };

	/*
  * Constructor
  */
	PID();

	/*
  * Destructor.
  */
	virtual ~PID();

	/*
  * Initialize PID.
  */
	void Init(double Kp, double Ki, double Kd);

	/*
  * Update the PID error variables given cross track error.
  */
	void UpdateError(double cte);

	/*
  * Calculate the total PID error.
  */
	double TotalError();
};

#endif /* PID_H */
