
#ifndef __PROPAGATION_H_INCLUDED__
#define __PROPAGATION_H_INCLUDED__

#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
//#include <complex.h>
#include <stdio.h>


#define lambda2 0.6328
#define _USE_MATH_DEFINES
#include <math.h>

using namespace cv;
using namespace std;


int PropPC(Mat& real, Mat& img, int cx, int cy, int dx, int dy, double z); 
void mostrarMagPha_PC(Mat& real, Mat& imag);
int fourierMethod_PC(Mat& real, Mat& img, bool inver);
int multiplyI(Mat mat1[], Mat mat2[], Mat E[]);
int fourierMethod(Mat mat[], bool inver);
int shiftFTS(Mat& magI);
int shiftFT(Mat& magI);

/*
static void help()
{
	printf("\nEste programa calculará la propagación de una campo con información inicial representada como archivo de imagen en blanco y negro \n"
		"The dft of an image is taken and it's power spectrum is displayed.\n"
		"Usage:\n"
		"./dft [image_name -- default ../data/lena.jpg]\n");
}
*/
#endif

