#include <iostream>
#include <vector>
#include <limits>

#include "PID.h"

using namespace std;

/* Init
 * Initializes the parameter set and the class for operation.
 */
void PID::Init(const std::vector<double> &p)
{
  m_state = 0;
  m_p = p;
  m_best_err = numeric_limits<double>::max();
  m_current_err = 0.0;

  m_p_error = 0.0;
  m_d_error = 0.0;
  m_i_error = 0.0;
  m_prev_cte_error = 0.0;
  m_prev_cte_initialized = false;

}


void PID::SetTrainingParams(bool is_training,
                            const std::vector<double> &dp,
                            unsigned int steps_per_run,
                            unsigned int steps_to_skip)
{
  m_dp = dp;
  m_is_training = is_training;
  m_steps_per_run = steps_per_run;
  m_steps_to_skip = steps_to_skip;
}


double PID::TotalError()
{
  return - m_p[0] * m_p_error +
         - m_p[1] * m_i_error +
         - m_p[2] * m_d_error;
}


void PID::Update(double cte, std::function<void()> reset_sim, bool terminate_run)
{
  if (m_is_training)
  {
    m_reset_simulator = reset_sim;
    Train(cte, m_state, terminate_run);
  }
  else
    UpdateError(cte);
}


void PID::UpdateError(double cte)
{
  if (m_is_training && (m_iter >= m_steps_to_skip))
    m_current_err += cte * cte;

  if (!m_prev_cte_initialized)
  {
    m_prev_cte_error = cte;
    m_prev_cte_initialized = true;
  }

  m_p_error = cte;
  m_d_error = cte - m_prev_cte_error;
  m_i_error += cte;

  m_prev_cte_error = cte;
}


/* Twiddle
 * Performs optimization of a parameter set.
 *
 * Here's the python version
 *
    def twiddle(tol=0.2):
        p = [0, 0, 0]
        dp = [1, 1, 1]
        x_trajectory, y_trajectory, best_err = run(make_robot(), p)
        while sum(dp) > tol:
            for i in range(len(p)):
                p[i] += dp[i]
                _, _, err = run(make_robot(), p)
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] -= 2*dp[i]
                    _, _, err = run(make_robot(), p)
                    if err < best_err:
                        best_err = err
                        dp[i] *= 1.1
                    else:
                        p[i] += dp[i]
                        dp[i] *= 0.9
        return p, best_err
 *
 * Implement this as a state machine in C++
 * Note that the state is changed only upon
 * receipt of a telemetry event.
 *
    state 0: initialization
        p = [0, 0, 0]
        dp = [1, 1, 1]
        iter = 0
        run state 1 action
    state 1: run initial run
        update errors
        iter += 1
        if end of run
            best_err = x
            run state 2 action
        else
            next_state = 1
    state 2:
        if sum(dp) < tol
            mark end of twiddle run
            next_state = end
        else
            i = 0
            run state 3 action
    state 3:
        if i >= len(p):
            run state 2 action
        else
            p[i] += dp[i]
            iter = 0
            reset simulator
            next_state = 4
    state 4:
        update errors
        iter += 1
        if end of run
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
                i ++
                run state 3 action
            else:
                p[i] -= 2*dp[i]
                iter = 0
                next_state = 5
                reset simulator
        else
            next_state = 4
    state 5:
        update errors
        iter += 1
        if end of run:
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
                i ++
                run state 3 action
            else
                p[i] += dp[i]
                dp[i] *= 0.9
                i++
                run state 3 action
        else
            next_state = 5
 */


void PID::ResetTrainingState()
{
  m_iter = 0;
  m_current_err = 0.0;
  m_prev_cte_initialized = false;

  m_p_error = 0.0;
  m_i_error = 0.0;
  m_d_error = 0.0;
  m_prev_cte_error = 0.0;
}


void PID::ResetSimulator()
{
  ResetTrainingState();
  if (m_reset_simulator)
    m_reset_simulator();
}

void PID::Train(double cte, unsigned int state, bool terminate_run)
{
  switch(state)
  {
    case 0:
      ResetTrainingState();
      Train(cte, 1, false);
      break;
    case 1:
      UpdateError(cte);
      m_iter++;
      if (terminate_run && (m_iter > m_steps_to_skip))
      {
        // This is a shortcut, if the car is stalled, assume the same cte for rest of the run
        m_current_err += early_termination_penalty * (cte * cte * (m_steps_per_run - m_iter));
        m_iter = m_steps_per_run;
      }
      if (m_iter >= m_steps_per_run)
      {
        cout << "New best err! : " << AverageError(m_current_err) << " : "
          << " Kp: " << m_p[0]
          << " Ki: " << m_p[1]
          << " Kd: " << m_p[2] << endl;
        m_best_err = m_current_err;
        Train(cte, 2, false);
      }
      else
        m_state = 1;
      break;
    case 2:
      if (Sum(m_dp) < twiddle_tolerance)
      {
        // We're done!  Transition to end state
        Train(cte, 99, false);
      }
      else
      {
        m_i = 0;
        Train(cte, 3, false);
      }
      break;
    case 3:
      if (m_i >= m_p.size())
        Train(cte, 2, false);
      else
      {
        m_p[m_i] += m_dp[m_i];
        m_state = 4;
        cout << "Trying : "
          << " Kp: " << m_p[0]
          << " Ki: " << m_p[1]
          << " Kd: " << m_p[2]
          << " sum(dp): " << Sum(m_dp) << endl;
        ResetSimulator();
      }
      break;
    case 4:
      UpdateError(cte);
      m_iter++;
      if (terminate_run && (m_iter > m_steps_to_skip))
      {
        // This is a shortcut, if the car is stalled, assume the same cte for rest of the run
        m_current_err += early_termination_penalty * (cte * cte * (m_steps_per_run - m_iter));
        m_iter = m_steps_per_run;
      }
      if (m_iter >= m_steps_per_run)
      {
        if (m_current_err < m_best_err)
        {
          cout << "New best err! : " << AverageError(m_current_err) << " : "
            << " Kp: " << m_p[0]
            << " Ki: " << m_p[1]
            << " Kd: " << m_p[2]
            << " sum(dp): " << Sum(m_dp)
            << endl;
          m_best_err = m_current_err;
          m_dp[m_i] *= twiddle_increase;
          m_i++;
          Train(cte, 3, false);
        }
        else
        {
          m_p[m_i] -= 2*m_dp[m_i];
          m_state = 5;
          cout << "Trying : "
            << " Kp: " << m_p[0]
            << " Ki: " << m_p[1]
            << " Kd: " << m_p[2]
            << " sum(dp): " << Sum(m_dp) << endl;
          ResetSimulator();
        }
      }
      else
        m_state = 4;
      break;
    case 5:
      UpdateError(cte);
      m_iter++;
      if (terminate_run && (m_iter > m_steps_to_skip))
      {
        // This is a shortcut, if the car is stalled, assume the same cte for rest of the run
        m_current_err += early_termination_penalty * (cte * cte * (m_steps_per_run - m_iter));
        m_iter = m_steps_per_run;
      }
      if (m_iter >= m_steps_per_run)
      {
        if (m_current_err < m_best_err)
        {
          cout << "New lowest err : " << AverageError(m_current_err) << " : "
            << " Kp: " << m_p[0]
            << " Ki: " << m_p[1]
            << " Kd: " << m_p[2]
            << " sum(dp): " << Sum(m_dp) << endl;
          m_best_err = m_current_err;
          m_dp[m_i] *= twiddle_increase;
          m_i++;
          Train(cte, 3, false);
        }
        else
        {
          m_p[m_i] += m_dp[m_i];
          m_dp[m_i] *= twiddle_decrease;
          m_i++;
          Train(cte, 3, false);
        }
      }
      else
        m_state = 5;
      break;
    case 99:
      m_is_training = false;

      // TODO: how do we exit out cleanly?
      throw "exit!";
      break;
  }
}
