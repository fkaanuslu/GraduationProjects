/*
 * kalmanfilter.h
 *
 *  Created on: 18 Eki 2022
 *      Author: fkaanuslu
 */

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

  void SimpleKalmanFilter(float mea_e, float est_e, float q);
  float updateEstimate(float mea); // estimate'i updateler yani x_hat'i
  void setMeasurementError(float mea_e); // ölçüm sırasındaki hatayı ver v_k
  void setEstimateError(float est_e); // prediction errorum
  void setProcessNoise(float q); // w_k değerimin covaryansı
  float getKalmanGain(); // kalman gain elde et
  float getEstimateError(); // prediction errorumu elde et

  float _err_measure; //w_k değerim
  float _err_estimate; // prediction error
  float _q;//w_k değerinin covariancesi
  float _current_estimate = 0; // başlagıç current estimate'im
  float _last_estimate = 0; // last estimate'im
  float _kalman_gain = 0; // kalman gain'im

#endif /* KALMANFILTER_H_ */
