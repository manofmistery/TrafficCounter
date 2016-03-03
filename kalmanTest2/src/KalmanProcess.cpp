/*
 * KalmanProcess.cpp
 *
 *  Created on: Jan 19, 2016
 *      Author: bisman
 */
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <ctype.h>
#include <struct.h>
#include <math.h>

PointSeqList KalmanProcess(KalmanPoint *Kalmanfilter, PointSeqList Points,
		IplImage *temp, IplImage *final, int *StopReflesh, int frame_count) {
	//use kalman_filter to predict the state of the centroid
//  int P_num;
	CvPoint Centroid;
	CvPoint Point_Find;
	const CvMat *Prediction, *correction;
	CvMat measurement;
	int delta =10;
	//int delta = 600;

	Point_Find.x = Point_Find.y = 0;

	const CvMat *state_ptMat_pre = Kalmanfilter->Kalman->state_post;
	Prediction = cvKalmanPredict(Kalmanfilter->Kalman, 0);

	//printf("1_prediction %f,%f\n", cvmGet(Prediction, 0, 0),
	//		cvmGet(Prediction, 1, 0));

	const CvMat *Prediction_error = Kalmanfilter->Kalman->error_cov_pre;

	double error_x_pre = cvmGet(Prediction_error, 0, 0);
	double error_y_pre = cvmGet(Prediction_error, 1, 1);

	//printf("2_prediction error %f,%f\n", error_x_pre, error_y_pre);

	Centroid = cvPoint(cvRound(cvmGet(Prediction, 0, 0)),
			cvRound(cvmGet(Prediction, 1, 0)));

	// out of the screen,release the filter
	if (Centroid.x > temp->width + delta || Centroid.x < 0 - delta
			|| Centroid.y > temp->height + delta || Centroid.y < 0 - delta) {
		cvReleaseKalman(&(Kalmanfilter->Kalman));
		Kalmanfilter->Kalman = NULL;
		return Points;
	}

	//*********** search the PointSeqList Directly ***************
	PointSeq *find = NULL, *pt = Points;
	//int direction = 320 * 240; //ori
	//int direction = 1280 * 720; //video HD
	int direction = 640 * 480;//webcam logitech
	//search for the minimal direction Point
	while (pt) {
		int x = pt->Point.x - Centroid.x;
		int y = pt->Point.y - Centroid.y;
		int t = x * x + y * y;
		if (t < direction) {
			direction = t;

			find = pt;
			Point_Find = pt->Point;
		}
		pt = pt->next;
	}

//  printf("direction %f\n",sqrt(direction));
	if (sqrt(direction) > 30) {
		Point_Find = Centroid;
		find = NULL;
		Kalmanfilter->Loss++;
	}

	// delete the Point found
	if (find) {
		if (find == Points) {
			Points = Points->next;
			delete[] find;
			find = Points;
		} else {
			if (find->next == NULL) {
				find = find->pre;
				delete[] find->next;
				find->next = NULL;
			} else {
				PointSeq *s;
				s = find->next;
				find->pre->next = find->next;
				find->next->pre = find->pre;
				delete[] find;
				find = s;
			}
		}
	} else {
		Point_Find = Centroid;
		Kalmanfilter->Loss++;
	}

	Kalmanfilter->Point_pre = Kalmanfilter->Point_now;
	Kalmanfilter->Point_now = Point_Find;
	Kalmanfilter->lastFrame = frame_count;//tambahan 25 januari 2016 speed measurement

	int delta_x = abs(Kalmanfilter->Point_now.x - Kalmanfilter->Point_pre.x);
	int delta_y = abs(Kalmanfilter->Point_now.y - Kalmanfilter->Point_now.y);
	//errrorrrr stopreflesh
	if (delta_x <= 2 && delta_y <= 2) {
		*StopReflesh = 1;
	} else
		*StopReflesh = 0;

	float measure[2] = { (float) Point_Find.x, (float) Point_Find.y };
	measurement = cvMat(2, 1, CV_32FC1, measure);
	correction = cvKalmanCorrect(Kalmanfilter->Kalman, &measurement);

	//************************************************************

	// test code
	//printf("3_measurement %f,%f\n\n", cvmGet(&measurement, 0, 0),
	//		cvmGet(&measurement, 1, 0));
	/*  printf("correction %f,%f\n",cvmGet(correction, 0, 0), cvmGet(correction, 1, 0));

	 const CvMat *correction_error = Kalmanfilter->Kalman->error_cov_post;
	 int error_x_post = (int)sqrt(correction_error->data.fl[0]);
	 int error_y_post = (int)sqrt(correction_error->data.fl[5]);
	 printf("correction error %d,%d\n",error_x_post, error_y_post);
	 */
	//draw the trajectory
	const CvMat *state_ptMat_now = Kalmanfilter->Kalman->state_post;
	const CvPoint state_pt_pre = cvPoint(cvRound(cvmGet(state_ptMat_pre, 0, 0)),
			cvRound(cvmGet(state_ptMat_pre, 1, 0)));
	const CvPoint state_pt_now = cvPoint(cvRound(cvmGet(state_ptMat_now, 0, 0)),
			cvRound(cvmGet(state_ptMat_now, 1, 0)));
	//cvLine(final, state_pt_now, state_pt_pre, cvScalar(255), 3, CV_AA, 0);
	cvLine(final, Kalmanfilter->firstPoint, state_pt_now, CV_RGB(255, 255, 0),
			1, CV_AA, 0);
	// Print the ID of the object on the screen
	char text[10];
	//sprintf(text, "ID:%d", Kalmanfilter->ID);
	//sprintf(text, "x:%d", state_pt_now.x);
	sprintf(text, "%d", Kalmanfilter->contourArea);//test 26 januari 2016
	//sprintf(text, "%d", Kalmanfilter->jenis);	//test 26 januari 2016
	CvFont font;
	cvInitFont(&font, CV_FONT_VECTOR0, 0.5f, 0.5f, 0, 2);
	cvPutText(final, text, state_pt_now, &font, CV_RGB(255, 0, 0));

	//tambahan coba memory leak

	char jns[1];
	/*
	if (Kalmanfilter->jenis == 1) {
		sprintf(jns, "mobil");
	} else if (Kalmanfilter->jenis == 2) {
		sprintf(jns, "truk");
	} else {
		sprintf(jns, "undefined");
	}*/
	//nilai 1 = motor;nilai 2 = mobil;nilai 3 = truk sedang;nilai 4 = truk besar;nilai 0 = undefined;
	if (Kalmanfilter->jenis == 2) {
		sprintf(jns, "mobil");
	} else if (Kalmanfilter->jenis == 3) {
		sprintf(jns, "truk sedang");
	} else if (Kalmanfilter->jenis == 4) {
		sprintf(jns, "truk besar");
	} else {
		sprintf(jns, "undefined");
	}

	cvPutText(final, jns, cvPoint(state_pt_now.x, state_pt_now.y + 15), &font,
			CV_RGB(255, 0, 0));

	return Points;
}
