/*
 * NewKalman.cpp
 *
 *  Created on: Jan 19, 2016
 *      Author: bisman
 */

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <ctype.h>
#include <struct.h>


KalmanPoint *NewKalman(PointSeq *Point_New, int ID, int frame_count, int contourArea) {
	const float A[] = { 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1 };
	const float H[] = { 1, 0, 0, 0, 0, 1, 0, 0 };

	KalmanPoint *Kalmanfilter;
	Kalmanfilter = (KalmanPoint *) malloc(sizeof(KalmanPoint));
	CvKalman *Kalman = cvCreateKalman(4, 2, 0);

	float measure[2] =
			{ (float) Point_New->Point.x, (float) Point_New->Point.y };
	CvMat measurement = cvMat(2, 1, CV_32FC1, measure);

	// initialize the Kalman filter
	memcpy(Kalman->transition_matrix->data.fl, A, sizeof(A));
	memcpy(Kalman->measurement_matrix->data.fl, H, sizeof(H));
	cvSetIdentity(Kalman->error_cov_post, cvRealScalar(10));
	cvSetIdentity(Kalman->process_noise_cov, cvRealScalar(1e-5));
	cvSetIdentity(Kalman->measurement_noise_cov, cvRealScalar(1e-1));

	// for the first time, use the measured point as the state_post
	cvZero(Kalman->state_post);
	cvmSet(Kalman->state_post, 0, 0, Point_New->Point.x);
	cvmSet(Kalman->state_post, 1, 0, Point_New->Point.y);

	// use Kalman  filter to predict the position of the centroid
	const CvMat *prediction = cvKalmanPredict(Kalman, 0);
	// use the measurement to correct the position
	const CvMat *correction = cvKalmanCorrect(Kalman, &measurement);

	/*
	 // test code
	 printf("measurement %f,%f\n",cvmGet(&measurement,0,0),cvmGet(&measurement, 1, 0));
	 printf("prediction %f,%f\n",cvmGet(prediction, 0, 0),cvmGet(prediction, 1, 0));
	 printf("correction %f,%f\ndelta %f,%f\n",cvmGet(correction, 0, 0),cvmGet(correction, 1, 0),
	 cvmGet(correction, 2, 0), cvmGet(correction, 3, 0));
	 */
	Kalmanfilter->Point_now = Point_New->Point;
	Kalmanfilter->Point_pre = Point_New->Point;
	Kalmanfilter->firstPoint = Point_New->Point;
	Kalmanfilter->firstFrame = frame_count; //tambahan 25 januari 2016 speed measurement
	Kalmanfilter->lastFrame = frame_count; //tambahan 25 januari 2016 speed measurement
	Kalmanfilter->Kalman = Kalman;
	Kalmanfilter->ID = Point_New->ID = ID;
	Kalmanfilter->contourArea=contourArea;
	/*
	//nilai 1 = mobil nilai 2= truk nilai 3 = undefined
	if (Kalmanfilter->contourArea >= 4800
			&& Kalmanfilter->contourArea <= 19900) {
		Kalmanfilter->jenis = 1;

	} else if (Kalmanfilter->contourArea > 19900) {
		Kalmanfilter->jenis = 2;

	} else {
		Kalmanfilter->jenis = 3;
	}
	*/
	//nilai 1 = motor;nilai 2 = mobil;nilai 3 = truk sedang;nilai 4 = truk besar;nilai 0 = undefined;
	if (Kalmanfilter->contourArea >= 4000
			&& Kalmanfilter->contourArea <= 7000) {
		Kalmanfilter->jenis = 2;
	} else if (Kalmanfilter->contourArea > 7000
			&& Kalmanfilter->contourArea <= 12000) {
		Kalmanfilter->jenis = 3;
	} else if (Kalmanfilter->contourArea > 12000) {
		Kalmanfilter->jenis = 4;
	} else {
		Kalmanfilter->jenis = 0;
	}
	//Kalmanfilter->jenis= 0;//tambahan 26 januari 2016
	Kalmanfilter->Loss = 0;

	return Kalmanfilter;
}
