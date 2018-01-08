#ifndef PID_H
#define PID_H

#include <functional>
#include <ctime>
#include <cassert>

// This is an error factor that is multiplied to the
// error when the simulator is stopped early.
constexpr double early_termination_penalty = 9.0;

// This is the tolerance used to determine when to
// stop twiddle.
constexpr double twiddle_tolerance = 0.01;

// These parameters control how much we increase/decrease
// the model params.
constexpr double twiddle_increase = 1.2;
constexpr double twiddle_decrease = 0.8;

class PID
{
protected:
  std::vector<double>  m_p;
  std::vector<double>  m_dp;
  unsigned int    m_steps_per_run;
  unsigned int    m_steps_to_skip;

  // Implements the PID controller
  double          m_p_error;
  double          m_d_error;
  double          m_i_error;
  double          m_prev_cte_error;
  bool            m_prev_cte_initialized;

  // Keeps track of place in state machine
  unsigned int    m_i;            // which parameter we're optimizing, i.e the i in p[i]
  double          m_best_err;     // best (lowest) error found so far
  double          m_current_err;  // current error values
  unsigned int    m_iter;         // iteration #
  unsigned int    m_state;        // current state of state machine
  bool            m_is_training;  // set to true if twiddle training

  std::function<void()> m_reset_simulator;

public:
  /*
   * constructor
   */
  PID()
    : m_is_training(false)
  {
  }

  /* Init
   * Initializes the parameter set and the class for operation.
   */
  void Init(const std::vector<double> &p);

  /* SetTrainingParams
   * Initializes the parameters for training using twiddle.
   *
   * @param is_training   Set to true to enable training with twiddle.
   * @param dp            The initial dp for twiddle.  This is the amount
   *                      by which the parameter values are adjusted.
   * @param steps_per_run How many steps (or calls to Update()) are used per run.
   * @param steps_to_skip The number of steps to skip over before calculating
   *                      the error.
   */
  void SetTrainingParams(bool is_training,
                        const std::vector<double> &dp,
                        unsigned int steps_per_run,
                        unsigned int steps_to_skip);

  /* TotalError
   * Total PID error
   */
  double TotalError();

  /* Update
   * Call this when a new CTE has been received and is ready to be processed.
   *
   * @param cte       The cross-track-error
   * @param reset_sim A function that will be called when the simulator is reset.
   *                  Only used during training.
   * @param terminate_run Set this to true to prematurely terminate a training run.
   */
  void Update(double cte, std::function<void()> reset_sim = nullptr, bool terminate_run = false);

protected:
  /* UpdateError
   * Call this to update the system with the cross track error.
   */
  void UpdateError(double cte);

  /* Sum
   * Utility function to sum all elements of a vector<>
   */
  double Sum(const std::vector<double> &v) const
  {
    double vsum = 0.0;
    for (const auto& x : v)
      vsum += x;
    return vsum;
  }


  /* AverageError
   * Average the error value by the number of iterations counted.
   */
  double AverageError(double error) const
  {
    assert(m_iter > m_steps_to_skip);
    return error / (m_iter - m_steps_to_skip);
  }

  /* Train
   * This is called when a new CTE has been received.  This immplements
   * the twiddle algorithm as a state machine.
   * @param cte The cross track error
   * @param state The state to be run
   * @param terminate_run Set this to true to terminate the current run
   *                      and go to the next run (only if iter > steps_to_skip)
   */
  void Train(double cte, unsigned int state, bool terminate_run);

  void ResetSimulator();
  void ResetTrainingState();
};

#endif /* PID_H */
