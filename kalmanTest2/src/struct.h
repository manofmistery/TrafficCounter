/*
 * struct.h
 *
 *  Created on: Jan 19, 2016
 *      Author: bisman
 */

#ifndef STRUCT_H_
#define STRUCT_H_

#include <stdio.h>
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <ctype.h>

typedef struct PointSeq {
	CvPoint Point;
	int ID;
	int contourArea;
	int jenis;
	PointSeq *pre;
	PointSeq *next;
} PointSeq, *PointSeqList;

typedef struct KalmanPoint {
	int Loss;
	CvPoint firstPoint;
	CvPoint Point_now;
	CvPoint Point_pre;
	CvKalman *Kalman;
	int ID;
	int contourArea;
	int jenis;
	int firstFrame; //update 25 januari 2015
	int lastFrame; //update 25 januari 2015
	KalmanPoint *pre;
	KalmanPoint *next;
} KalmanPoint, *filterList;

//extern int counterMob;
//extern int counterTruk;
//extern int counterMob2;
//extern int counterTruk2;

#endif /* STRUCT_H_ */
