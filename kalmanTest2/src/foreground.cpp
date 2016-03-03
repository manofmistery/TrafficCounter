/*
 * foreground.cpp
 *
 *  Created on: Jan 19, 2016
 *      Author: bisman
 */

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <ctype.h>
#include <struct.h>

//extract the centroid of the moved object of the image
PointSeqList foreground(IplImage *frame_gray_now, IplImage *frame_bkg, IplImage *final,
		int frame_count, double *averagethreshold) {
	//const int CONTOUR_MAX_AREA = 600; //orignal
	const int CONTOUR_MAX_AREA = 4900; //tambahan
	//const int CONTOUR_MIN_AREA = 100;
	const double SPEED_LIMIT = 80.00;
	CvScalar *mean, *std_dev;
	CvMemStorage *storage;
	CvSeq *cont;
	PointSeq *Centroids;
	PointSeqList Head = NULL;
	double threshold_now;

	IplImage *pyr = cvCreateImage(
			cvSize((frame_gray_now->width & -2) / 2,
					(frame_gray_now->height & -2) / 2), 8, 1);
	//cvShowImage("debug 0", pyr);
	IplImage *temp = cvCreateImage(
			cvSize(frame_gray_now->width, frame_gray_now->height), 8, 1);

	temp->origin = 1;
	//cvFlip(temp,temp, -1);//tambahan dari sahid
	mean = (CvScalar *) malloc(sizeof(CvScalar));
	std_dev = (CvScalar *) malloc(sizeof(CvScalar));

	// get the difference of the current image and the background
	cvAbsDiff(frame_gray_now, frame_bkg, frame_gray_now);

	// get the standard deviation of the difference image
	cvAvgSdv(frame_gray_now, mean, std_dev, NULL);

	//*************************************************************

	// accumulate the threshold to get the average threshold
	threshold_now = 2.3 * std_dev->val[0];

	if (frame_count < 55) { //original
		//if (frame_count < 25) {//tambahan
		// initialize the average threshold
		if (*averagethreshold < threshold_now) {
			*averagethreshold = threshold_now;
		}
	} else if (threshold_now < *averagethreshold) {
		threshold_now = *averagethreshold;
	} else {
		// Update the average threshold
		*averagethreshold = ((frame_count - 1) * (*averagethreshold)
				+ threshold_now) / frame_count;
	}
	cvThreshold(frame_gray_now, frame_gray_now, threshold_now, 255,
			CV_THRESH_BINARY);

//  printf("%f, %f, %d\n",*averagethreshold, threshold_now, frame_count);

// reduce the noise

	cvErode(frame_gray_now, frame_gray_now, NULL, 1);
	cvDilate(frame_gray_now, frame_gray_now, NULL, 3);
	cvErode(frame_gray_now, frame_gray_now, NULL, 1);

	cvSmooth(frame_gray_now, temp, CV_GAUSSIAN, 5, 3, 0);
	cvThreshold(temp, temp, 0, 255, CV_THRESH_BINARY);

//  cvShowImage( "background", temp );

	cvPyrDown(temp, pyr, CV_GAUSSIAN_5x5);
	cvDilate(pyr, pyr, 0, 1);
	cvPyrUp(pyr, temp, CV_GAUSSIAN_5x5);
	cvSmooth(temp, temp, CV_GAUSSIAN, 5, 3, 0);
	cvAvgSdv(temp, mean, std_dev, NULL);
	cvThreshold(temp, temp, mean->val[0], 255, CV_THRESH_BINARY);

	cvDilate(temp, temp, NULL, 1);
	cvErode(temp, temp, NULL, 1);

	cvShowImage("temp", temp);
	//cvShowImage("pyr", pyr);
	//************************************************************

	storage = cvCreateMemStorage(0);
	cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint),
			storage);

	// find the external contours of the object
	cvFindContours(temp, storage, &cont, sizeof(CvContour), CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));

	cvZero(frame_gray_now);

	for (; cont; cont = cont->h_next) {
		// Number point must be more than or equal to 20
		if (cont->total < 20) //the number of sequence elements
			continue;

		CvRect r = ((CvContour*) cont)->rect;
		if (r.height * r.width > CONTOUR_MAX_AREA) // remove the smaller areas
				{
			// Draw current contour.
			//cvDrawContours(frame_gray_now, cont, CV_RGB(255,0,0),CV_RGB(255,255,255),
			//                    0, CV_FILLED, 8, cvPoint(0,0));
			// get the centroid of the object
			CvMoments moments;
			cvMoments(cont, &moments, 0);
			CvPoint Center = cvPoint(cvRound(moments.m10 / moments.m00),
					cvRound(moments.m01 / moments.m00));
			//if virtual line
			/*if (Center.x > 20 && Center.x < 570 && Center.y > 450
					&& Center.y < 600
					or Center.x > 690 && Center.x < 1280 && Center.y > 450
							&& Center.y < 600)*/
			if (Center.x > 0 && Center.x <= 640 && Center.y > 210
					&& Center.y <= 320)
			{
				cvRectangleR(final, r, CV_RGB(255,255,0), 2);
				cvCircle(final, Center, 2, CV_RGB(255,255,0), -1, 4, 0);
				// make a list of the centroidste
				Centroids = (PointSeq *) malloc(sizeof(PointSeq));
				Centroids->Point = Center;
				Centroids->ID = 0;
				//klasifikasi kendaraan
				Centroids->contourArea = r.height * r.width;
				Centroids->next = Head;
				Head = Centroids;
				if (Head->next) {
					Head->next->pre = Head;
				}
			}
		}
	}

	cvReleaseMemStorage(&storage);
	cvReleaseImage(&pyr);
	cvReleaseImage(&temp);
	delete[] mean;
	//free(mean);
	delete[] std_dev;
	//free(std_dev);
	return Head;
}
