/*
 * kalmanfilter.c
 *
 *  Created on: 14 Eki 2022
 *      Author: fkaanuslu
 *
 *special thanks to Simond Levy
 * https://simondlevy.academic.wlu.edu/kalman-tutorial/
 *
 *
 *
 *
 */
#include "kalmanfilter.h"
#include <math.h>

// The walkthrough
	  /*
	   *
	   * Well, here is the set of equations for our linear Kalman Filter, using a model with no state transition or control signal,
	   * some process noise, a single sensor, and a single state value [From Simond Levy's blog]
	   *
	   * 					Model :
	   *
	   * x_k = x_kpre + w_k
	   *
	   * w_k -> (process noise)
	   *
	   * z_k = c * x_k + v_k
	   *
	   * v_k -> (current noise measurement -+ 200 gibi)
	   *
	   * We used r to represent the covariance of the measurement noise v_k
	   *
	   * 					Predict :
	   *
	   * x_estimate_k = x_estimate_kpre // en son x_estimate_k görücem ekranda
	   *
	   * p_k = p_kpre + q
	   *
	   * q to represent the covariance of the process noise w_k
	   *
	   * An initial value p0 for the prediction error. It can’t be 0, otherwise p_k would stay 0 forever by multiplication.
	   * So we arbitrarily set it to 1.
	   *
	   * 					Update :
	   *
	   * gain <-- p_k * c / ( c * p_k * c + r)
	   *
	   * x_estimate_k <-- x_estimate_k + gain(z_k - c * x_estimate_k)
	   *
	   * p_k <-- (1 - gain * c) * p_k
	   *
	   * */


  float _err_measure; //w_k değerim
  float _err_estimate; // prediction error
  float _q;//w_k değerinin covariancesi
  float _current_estimate = 0; // başlagıç current estimate'im
  float _last_estimate = 0; // last estimate'im
  float _kalman_gain = 0; // kalman gain'im

void SimpleKalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure=mea_e; //
  _err_estimate=est_e;
  _q = q;
}

float updateEstimate(float mea)
{
   _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
   // g_k = p_k / p_k + r

   _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
   // x_hat_k = x_hat_k_last + g_k * (z_k - x_hat_k_last)


   _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate - _current_estimate)*_q;
   // p_k = (1 - g_k) * p_k + |x_hat_k_last - x_hat_k| * q

   _last_estimate = _current_estimate;
  // x_hat_k_prev = x_hat_k

   return _current_estimate; // x_hat_k which the filtered one.
}

void setMeasurementError(float mea_e)
{
  _err_measure = mea_e;
}

void setEstimateError(float est_e)
{
  _err_estimate = est_e;
}

void setProcessNoise(float q)
{
  _q = q;
}

float getKalmanGain() {
  return _kalman_gain;
}

float getEstimateError() {
  return _err_estimate;
}




