/*
 * kalmanTest1.cpp
 *
 *  Created on: Feb 3, 2016
 *      Author: bisman
 */

#include <sys/resource.h> // memory management.
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <struct.h>
#include <unistd.h>
#include <ctype.h>
#include "dbsql.h"
//#include <windows.h>

using namespace std;

static int frame_count = 0;
static int ID = 1;
static int StopReflesh = 0;
static double averagethreshold = 0;

int kalman_num = 0;

IplImage *frame, *frame_now, *frame_gray_now, *frame_gray_temp, *frame_bkg,
		*frame_gray_pass = NULL; //Preparing frame pointer

IplImage *background(IplImage *frame_gray_now, IplImage *frame_bkg,
		IplImage *frame_gray_pass, int frame_count, int StopReflesh);

PointSeqList foreground(IplImage *frame_gray_now, IplImage *frame_bkg,
		IplImage *final, int frame_count, double *averagethreshold); //update frame_count for speed measurement

KalmanPoint *NewKalman(PointSeq *Point_New, int ID, int frame_count,
		int contourArea);

PointSeqList KalmanProcess(KalmanPoint *Kalmanfilter, PointSeqList Points, //update frame_count for speed measurement
		IplImage *temp, IplImage *final, int *StopReflesh, int frame_count);

double GetTickCount(void) {
	struct timespec now;
	if (clock_gettime(CLOCK_MONOTONIC, &now))
		return 0;
	return now.tv_sec * 1000.0 + now.tv_nsec / 1000000.0;
}

int main() {
	filterList List = NULL;
	int key;
	int time = GetTickCount();
	//rata2 kecepatan kendaraan dan jumlah mobil
	int counterMot = 0;
	int counterMob = 0;
	int counterTruksed = 0;
	int counterTrukbes = 0;
	double sumspeed1 = 0.0;
	double avgspeed1 = 0.0;
	int n1 = 0;
	int i = 0;

//Allocates and initializes cvCapture structure
// for reading a video stream from the camera.
//Index of camera is -1 since only one camera
// connected to the computer or it does not
// matter what camera to use.
	CvCapture *input_camera=cvCreateFileCapture("rtsp://10.10.62.74:5004/stream");
	//CvCapture *input_camera = cvCaptureFromCAM(-1);
	//CvCapture *input_camera = cvCaptureFromAVI(
	//		"D:/PTSPT/2016/Tracking&Counting/rawimage/demo3.avi");
	//CvCapture *input_camera = cvCaptureFromAVI(
	//		"/home/ptspt/tracking_counting/rawimage/video2.avi");
//Grabs and returns a frame from camera
	frame = cvQueryFrame(input_camera);

//Creates window for displaying the frames
//Flag is reset (0) --> change window size
// manually
	cvNamedWindow("Capturing Image ...", 0);

//Change to the appropriate size. In GTK, the
// inappropriate size will return a segmentation
// fault. I don't know why ...
//Gets the appropriate size using cvGetCaptureProperty
// with CV_CAP_PROP_FRAME_HEIGHT and CV_CAP_PROP_FRAME_WIDTH
// as property_id
	cvResizeWindow("Capturing Image ...",
			(int) cvGetCaptureProperty(input_camera, CV_CAP_PROP_FRAME_HEIGHT),
			(int) cvGetCaptureProperty(input_camera, CV_CAP_PROP_FRAME_WIDTH));

	CvSize size = cvSize(
			(int) cvGetCaptureProperty(input_camera, CV_CAP_PROP_FRAME_HEIGHT),
			(int) cvGetCaptureProperty(input_camera, CV_CAP_PROP_FRAME_WIDTH));

	int fps = cvGetCaptureProperty(input_camera, CV_CAP_PROP_FPS);

	CvVideoWriter *writer = cvCreateVideoWriter(
			"/home/ptspt/tracking_counting/mjpg/out.avi",
			CV_FOURCC('I', 'Y', 'U', 'V'), fps, size, -1);

	struct rusage usage;
	while (frame != NULL) {

		//int fps = frame_count * 1000 / (GetTickCount() - time); //tambahan tanggal 25 januari 2016
		frame_count++;
		i++;
//Shows a frame
		//cvShowImage("Capturing Image ...", frame);

//Grabs and returns the next frame
		//frame = cvQueryFrame(input_camera);
		frame = cvQueryFrame(input_camera);
		frame_now = cvCreateImage(cvSize(frame->width, frame->height),
		IPL_DEPTH_8U, frame->nChannels);
		cvCopy(frame, frame_now, 0);
		frame_now->origin = frame->origin;
		//cvShowImage("Capturing Image 1...", frame_now);
		frame_gray_now = cvCreateImage(cvSize(frame->width, frame->height),
		IPL_DEPTH_8U, 1);
		frame_gray_now->origin = frame->origin;
		cvCvtColor(frame, frame_gray_now, CV_BGR2GRAY);
		frame_gray_now->origin = frame->origin;
		//cvShowImage("Capturing Image ...1", frame_gray_now);
		frame_gray_temp = cvCreateImage(cvSize(frame->width, frame->height),
		IPL_DEPTH_8U, 1);
		frame_gray_temp->origin = frame->origin;
		cvCopy(frame_gray_now, frame_gray_temp, 0);
		frame_gray_temp->origin = frame_gray_now->origin;

		frame_bkg = background(frame_gray_now, frame_bkg, frame_gray_pass,
				frame_count, StopReflesh);
		frame_bkg->origin = 1;

		PointSeqList Centroids = foreground(frame_gray_now, frame_bkg,
				frame_now, frame_count, &averagethreshold);
		//cout << frame_count << endl;

		//*****************Kalman Tracking**********************
		// initialize the Kalman Filter List
		if (!List) {
			PointSeq *s, *q = Centroids;
			while (q) {

				// create Kalman filter according to the centroid
				KalmanPoint *Kalman_Now;
				Kalman_Now = NewKalman(q, ID, frame_count, q->contourArea);
				// add the new filter to the list

				kalman_num++;
				ID++;
				cout << ID << endl;
				Kalman_Now->next = List;
				List = Kalman_Now;
				if (List->next) {
					List->next->pre = List;
				}

				//delete the point, and find the next one
				if (q = Centroids) {
					Centroids = Centroids->next;
				}
				s = q->next;
				delete[] q;
				q = s;
			}
		}

		else {
			// use the existed Kalman Filter to track the object
			KalmanPoint *k = List;
			while (k) {
				//tracking
				PointSeq *q = Centroids;

				Centroids = KalmanProcess(k, q, frame_gray_now, frame_now,
						&StopReflesh, frame_count);

				// if the Filter is outside the range of the screen,
				// or the object has disappeared for a long time,
				// delete it from the List
				//if (k->Kalman == NULL || k->Loss > 3) { //k-Loss > 3 original
				if (k->Kalman == NULL || k->Loss > 3) { //k-Loss > 3 original
					kalman_num--;
					//tambahan untuk speed
					// speed calculation based on distance (in pixel) travelled
					// time taken calculated from how many frame taken
					// 25frame ~ 1 second = 1000ms so 1 frame ~ 40ms
					// assume 1pixe = 10cm on real world (not calibrated)
					double dx = abs(k->Point_now.x - k->firstPoint.x) * 20;
					double dy = abs(k->Point_now.y - k->firstPoint.y) * 20;
					double displacement = sqrt(dx * dx + dy * dy);
					double timeTaken = (k->lastFrame - k->firstFrame)
							* (1000 / fps);
					// *36 is conversion cm/s to km/h
					double speed = (displacement / timeTaken) * 36;
					//cout<<"id "<<k->ID<<" kecepatan "<<speed<<endl;
					if (speed > 0) {
						if (k->Point_now.x <= 640) {
							n1++;
							sumspeed1 += speed;
							//nilai 1 = motor;nilai 2 = mobil;nilai 3 = truk sedang;nilai 4 = truk besar;nilai 0 = undefined;
							if (k->jenis == 2) {
								counterMob++;
								//cvLine(frame_now, cvPoint(260, 455),
								//		cvPoint(570, 455),
								//		CV_RGB(255, 255, 255), 2, CV_AA, 0);
							} else if (k->jenis == 3) {
								counterTruksed++;
								//cvLine(frame_now, cvPoint(260, 455),
								//		cvPoint(570, 455),
								//		CV_RGB(255, 255, 255), 2, CV_AA, 0);
							} else if (k->jenis == 4) {
								counterTrukbes++;
								//cvLine(frame_now, cvPoint(260, 455),
								//		cvPoint(570, 455),
								//		CV_RGB(255, 255, 255), 2, CV_AA, 0);
							} else {
								cout << "undefined" << endl;
							}
							avgspeed1 = sumspeed1 / n1;
							//cout<<speed<<" "<<n1<<" "<<avgspeed1<<endl;
						}
					}
					cvReleaseKalman(&k->Kalman);
					if (k == List) {
						List = List->next;
						delete[] k;
						//free(k);					//avoid memory leak
						k = List;
					} else {
						if (k->next == NULL) {
							k = k->pre;
							delete[] k->next;
							//free(k->next);					//avoid memory leak
							k->next = NULL;
						} else {
							KalmanPoint *s;
							s = k->next;
							k->pre->next = k->next;
							k->next->pre = k->pre;
							delete[] k;
							//free(k);					//avoid memory leak
							k = s;
						}
					}
				} else {
					k = k->next;
				}
			}

			// initialize the filter for the new object, and add it to the list
			if (Centroids) {
				PointSeq *s, *p = Centroids;
				while (p) {
					if (p->ID == 0) {
						// create Kalman filter according to the centroid
						KalmanPoint *Kalman_Now;
						Kalman_Now = NewKalman(p, ID, frame_count,
								p->contourArea);

						// add the new filter to the list
						kalman_num++;
						ID++;
						Kalman_Now->next = List;
						List = Kalman_Now;
						if (List->next) {
							List->next->pre = List;
						}
					}

					//delete the point, and find the next one
					if (p = Centroids) {
						Centroids = Centroids->next;
					}
					s = p->next;
					delete[] p;
					//free(p);					//avoid memory leak
					p = s;
				}
			}
		}
		//          printf("%d\n",kalman_num);  // display the number of the filters

		//******************************************************

		//-------------------process   END-------------------

		char text[10];

		char avgspeed[20];
		char countMot[10];
		char countMob[10];
		char countTruksed[10];
		char countTrukbes[10];

		sprintf(text, "%d:%dfps", frame_count, fps);
		CvFont font;
		cvInitFont(&font, CV_FONT_VECTOR0, 0.7f, 0.7f, 0, 2);
		cvPutText(frame_now, text, cvPoint(420, 380), &font,
				CV_RGB(255, 255, 255));

		////jalur 1////
		///avgspeed
		sprintf(avgspeed, "avg speed %f ", avgspeed1);
		cvPutText(frame_now, avgspeed, cvPoint(380, 400), &font,
				CV_RGB(255, 255, 255));
		///counting vehicle
		sprintf(countMob, "mobil %d ", counterMob);
		cvPutText(frame_now, countMob, cvPoint(380, 420), &font,
				CV_RGB(255, 255, 255));

		sprintf(countTruksed, "truk sedang %d ", counterTruksed);
		cvPutText(frame_now, countTruksed, cvPoint(380, 440), &font,
				CV_RGB(255, 255, 255));

		sprintf(countTrukbes, "truk besar / bis %d ", counterTrukbes);
		cvPutText(frame_now, countTrukbes, cvPoint(380, 460), &font,
				CV_RGB(255, 255, 255));

		//batas garis untuk webcamera
		cvLine(frame_now, cvPoint(0, 420), cvPoint(640, 420), cvScalar(255), 2,
		CV_AA, 0);	//line bawah
		cvLine(frame_now, cvPoint(0, 210), cvPoint(640, 210), cvScalar(255), 2,
		CV_AA, 0);	//line atas

		cvReleaseImage(&frame_gray_pass);
		if (!frame_gray_pass)
			frame_gray_pass = cvCreateImage(cvSize(frame->width, frame->height),
			IPL_DEPTH_8U, 1);
		frame_gray_pass->origin = 1;
		cvCopy(frame_gray_temp, frame_gray_pass, NULL);

		//show result
		cvShowImage("Capturing Image 2...", frame_gray_now);
		cvShowImage("Capturing Image 1...", frame_bkg);
		cvShowImage("Capturing Image ...", frame_now);

		//generate video or image
		//cvWriteFrame(writer, frame_now);
		char path[255];
		strcpy(path, "/home/ptspt/counterstream/out.jpg");
		cvSaveImage(path, frame_now);

		if (i == fps * 60 * 1) {
			dbinsert(counterMob, counterTruksed, counterTrukbes, avgspeed1);
			counterMob = 0;
			counterTruksed = 0;
			counterTrukbes = 0;
			i = 0;
			sumspeed1 = 0.0;
			avgspeed1 = 0.0;
			n1 = 0;
			cout
					<< "kirim database///////////////////////////////////////////////"
					<< endl;
		}

		cvReleaseImage(&frame_now);
		cvReleaseImage(&frame_gray_now);
		cvReleaseImage(&frame_gray_temp);

		//Sleep(50);
		//Checks if ESC is pressed and gives a delay
		// so that the frame can be displayed properly
		//getrusage(RUSAGE_SELF, &usage);
		//cout << "DEBUG leakTest_bug_report " << frame_count << " "
		//		<< usage.ru_maxrss / 1024.0 << endl;

		key = cvWaitKey(10);
		if (key == 27)
			break;

	}

//Release cvCapture structure
	cvReleaseCapture(&input_camera);
//release cvVideoWriter
	cvReleaseVideoWriter(&writer);
//Destroy the window
	cvDestroyWindow("Capturing Image ...");

	return 0;
}

