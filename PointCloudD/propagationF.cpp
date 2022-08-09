#include "propagationF.h" 
//#include "FocusTool.hpp" 
#define _USE_MATH_DEFINES

using namespace cv;
using namespace std;

int fourierMethod(Mat mat[],bool inver) {
	Mat complexMat(mat[0].rows, mat[0].cols, CV_64F);
	cv::merge(mat, 2, complexMat);
	shiftFT(complexMat);
	if (!inver)
	    cv::dft(complexMat, complexMat);
	else
		cv::dft(complexMat, complexMat,DFT_INVERSE);
	shiftFT(complexMat);
	cv::split(complexMat, mat);
	return(1);
}

int multiplyI(Mat mat1[], Mat mat2[],Mat E[]) {
	double a, b, c, d;
	for (int j = 0; j < mat1[0].rows; j++)
		for (int i = 0; i < mat1[1].cols; i++)
		{
			a = mat1[0].at<double>(j, i);
			b = mat1[1].at<double>(j, i);
			c = mat2[0].at<double>(j, i);
			d = mat2[1].at<double>(j, i);
			E[0].at<double>(j, i) = (a*c - b * d);
			E[1].at<double>(j, i) = (a*d + b * c);
		}
	return(1);
}
/*
void ajustSizeFT(Mat &real, Mat &imag, int padvalue)
{//SOLO SI LA IMAGEN ES PEQUEÑA
	/*img TIENE LA INFORMACIÓN DE LA IMAGEN EN FORMA MATRICIAL
	COMO UNA IMAGEN EN TONOS DE GRIS 

	int N = getOptimalDFTSize(real.rows);
	int M = getOptimalDFTSize(real.cols);

	M *= 2;
	N *= 2;

	//int padx = M - img.cols;
	//int pady = N - img.rows;

	/* M y N TIENEN LA INFORMACIÓN DEL TAMAÑO DE LA IMAGEN YA OPTIMIZADA PARA DFT 

	int top;
	top = N - real.rows;
	int left;
	left = M - real.cols;
	Mat padded(N, M, CV_64F);
	copyMakeBorder(
		real,
		padded,
		top / 2.0, top / 2.0,
		left / 2.0, left / 2.0,
		BORDER_CONSTANT,
		Scalar::all(padvalue)
	);

	Mat imaginaria(N, M, CV_64F);
	copyMakeBorder(
		imag,
		imaginaria,
		top / 2.0, top / 2.0,
		left / 2.0, left / 2.0,
		BORDER_CONSTANT,
		Scalar::all(padvalue)
	);
	padded.copyTo(real);
	imaginaria.copyTo(imag);
	/* PADDED ES COPIA DE IMG, CON EL TAMAÑO OPTIMIZADO PARA DFT 
	return;
}
*/
/*
void mostrarShift(string title, Mat *planes) {
	/*
	MOSTRAR SI LA TRANSFORMADA FUE CORRECTA. NO SE NECESITA PARA EL CODIGO FINAL
	

	//Mat temp;
	//planes.copyTo(temp);

	Mat temp[2];
	temp[0] = planes[0].clone();
	temp[1] = planes[1].clone();

	Mat mag = temp[0].clone();
	magnitude(temp[0], temp[1], mag);
	//mag = temp[0];

	//CORTAR ESPECTRO SI TIENE UN NUMERO PAR DE LINEAS
	mag = mag(Rect(0, 0, mag.cols& -2, mag.rows & -2));

	int cx = mag.cols / 2;
	int cy = mag.rows / 2;
	Mat tmp;
	Mat q0(mag, Rect(0, 0, cx, cy));
	Mat q1(mag, Rect(cx, 0, cx, cy));
	Mat q2(mag, Rect(0, cy, cx, cy));
	Mat q3(mag, Rect(cx, cy, cx, cy));

	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);
	q1.copyTo(tmp);
	q2.copyTo(q1);
	tmp.copyTo(q2);

	normalize(mag, mag, 0, 1, NORM_MINMAX);
	namedWindow(title, WINDOW_NORMAL);
	imshow(title, mag);
	//waitKey(0);

}
*/
void mostrarMagPha(Mat &real, Mat &imag)
{
	Mat mag = real.clone();
	Mat phasmat = real.clone();

	mag.setTo(0);
	phasmat.setTo(0);

	magnitude(real, imag, mag);

	phase(real, imag, phasmat, false);

	//phasmat = phasmat - M_PI;
	phasmat = (phasmat / (2 * M_PI)) * 255.0;
	Mat phaseview = phasmat.clone();
	phasmat.convertTo(phaseview, CV_8U);
	namedWindow("phase imag", WINDOW_NORMAL);
	resizeWindow("phase imag", 500, 500);
	imshow("phase imag", phaseview);

	waitKey(10);

	double mmin, mmax;
	cv::minMaxLoc(mag, &mmin, &mmax);
	//mmax = sqrt(mmax);
	mag = (mag / mmax)* 255.0;
	Mat magview = mag.clone();
	mag.convertTo(magview, CV_8U);
	namedWindow("magnitude", WINDOW_NORMAL);
	imshow("magnitude", magview);
	resizeWindow("magnitude", 500, 500);
	waitKey(0);
	/*magview.setTo(0.0);
	imshow("magnitude", magview);
	resizeWindow("magnitude", 500, 500);
	waitKey(10);*/

}

void mostrarMagPha_PC(Mat& real, Mat& imag)
{
	Mat mag = real.clone();
	Mat phasmat = real.clone();

	mag.setTo(0);
	//randu(mag, Scalar(0.), Scalar(1.));
	phasmat.setTo(0);
	//randu(phasmat, Scalar(0.), Scalar(1.));

	magnitude(real, imag, mag);

	phase(real, imag, phasmat, false);

	//phasmat = phasmat - M_PI;
	phasmat = (phasmat / (2 * M_PI)) * 255.0;
	Mat phaseview = phasmat.clone();
	phasmat.convertTo(phaseview, CV_8U);
	namedWindow("phase imag", WINDOW_NORMAL);
	resizeWindow("phase imag", 500, 500);
	imshow("phase imag", phaseview);

	waitKey(10);

	double mmin, mmax;
	cv::minMaxLoc(mag, &mmin, &mmax);
	//mmax = sqrt(mmax);
	//mag = (mag / mmax) * 255.0;
	mag = (255.0 / (mmax - mmin)) * (mag - mmin);
	Mat magview = mag.clone();
	mag.convertTo(magview, CV_8U);
	namedWindow("magnitude", WINDOW_NORMAL);
	imshow("magnitude", magview);
	resizeWindow("magnitude", 500, 500);

	waitKey(0);
	/*magview.setTo(0.0);
	imshow("magnitude", magview);
	resizeWindow("magnitude", 500, 500);
	waitKey(10);*/
}

int PropPC(Mat& real, Mat& img, int cx, int cy, int dx, int dy, double z) {
	double lamb = 0.635;
	double k = (2 * M_PI / lamb);
	double r;

	for (int j = 0; j < real.rows; j++) {
		for (int i = 0; i < real.cols; i++) {
			double x = (i - cx) * dx;
			double y = (cy - j) * dy; 
			r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
			double argum = (k * r);
			//real.at<double>(j, i) = (z / (lamb * pow(r, 2))) * (sin(argum));
			real.at<double>(j, i) = (z / (lamb * pow(r, 2)))* (sin(argum));
			img.at<double>(j, i) = (z / (lamb * pow(r, 2))) * (-1.0 * cos(argum));
		}
	}
	return (1);
}

int fourierMethod_PC(Mat& real, Mat& img, bool inver) {
	Mat Fouri[] = {real, img};  
	Mat complexMat(Fouri[0].rows, Fouri[0].cols, CV_64F);
	cv::merge(Fouri, 2, complexMat);
	shiftFT(complexMat);
	if (!inver)
		cv::dft(complexMat, complexMat);
	else
		cv::dft(complexMat, complexMat, DFT_INVERSE);
	shiftFT(complexMat);
	cv::split(complexMat, Fouri);
	return(1);
}

int shiftFTS(Mat &magI)
{
	// rearrange the quadrants of Fourier image  so that the origin is at the image center
	int cx = magI.cols / 2;
	int cy = magI.rows / 2;

	Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
	Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
	Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

	Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);

	q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
	q2.copyTo(q1);
	tmp.copyTo(q2);
	return 1;
}

int shiftFT(Mat &magI)
{
	Mat result(magI.rows, magI.cols, CV_64FC2);

	//vector<Mat> complexComp(2);

	Mat complexComp[] = { Mat(magI.rows, magI.cols, CV_64F), Mat(magI.rows, magI.cols, CV_64F) };
	split(magI, complexComp);
	// rearrange the quadrants of Fourier image  so that the origin is at the image center
	int cx = complexComp[0].cols / 2;
	int cy = complexComp[0].rows / 2;

	Mat q0(complexComp[0], Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat q1(complexComp[0], Rect(cx, 0, cx, cy));  // Top-Right
	Mat q2(complexComp[0], Rect(0, cy, cx, cy));  // Bottom-Left
	Mat q3(complexComp[0], Rect(cx, cy, cx, cy)); // Bottom-Right

	Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);

	q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
	q2.copyTo(q1);
	tmp.copyTo(q2);

	Mat q01(complexComp[1], Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
	Mat q11(complexComp[1], Rect(cx, 0, cx, cy));  // Top-Right
	Mat q21(complexComp[1], Rect(0, cy, cx, cy));  // Bottom-Left
	Mat q31(complexComp[1], Rect(cx, cy, cx, cy)); // Bottom-Right

	Mat tmp1;                           // swap quadrants (Top-Left with Bottom-Right)
	q01.copyTo(tmp1);
	q31.copyTo(q01);
	tmp1.copyTo(q31);

	q11.copyTo(tmp1);                    // swap quadrant (Top-Right with Bottom-Left)
	q21.copyTo(q11);
	tmp1.copyTo(q21);

	//merge(complexComp, result);
	merge(complexComp, 2, result);
	result.copyTo(magI);
	return 1;
}