#ifndef _KalmanFilter_H_
#define _KalmanFilter_H_
int KalmanFilter_ax( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
int KalmanFilter_ay( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
int KalmanFilter_az( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
//int KalmanFilter_gyrox( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
//int KalmanFilter_gyroy( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);
int KalmanFilter_gyroz( int ResrcData,int ProcessNiose_Q,int MeasureNoise_R);	

#endif  