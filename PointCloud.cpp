#ifdef _MSC_VER					
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "propagationF.h"

using namespace cv;
using namespace std;

const int T = 1024;

int main()
{
	//Lectura de datos
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\oscar\\OneDrive\\Documentos\\Delfín2022\\PointCloudD\\PointCloudD\\Banana.pcd", *cloud) == -1) //* load the file
	 {
		  PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		  return (-1);
	 }
	//Imprime los datos del archivo 

	//Matriz de puntos
	int TamNube = cloud->size();
	int tam = cloud->width * cloud->height;
	Mat points(tam, 3, CV_32F);
	int i = 0;
	double xp, yp, zp;

	for (const auto& point : *cloud){
		if (!(isnan(point.x) || isnan(point.y) || isnan(point.z))) {
			//std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
			//Le pasa los datos al objeto Mat
			points.at<float>(i, 0) = point.x;
			points.at<float>(i, 1) = point.y;
			points.at<float>(i, 2) = point.z;
			i++;
		}
	}	

	double mmin, mmax;
	minMaxLoc(points, &mmin, &mmax);
	points = (points / mmax) * 1.5;
	//Matriz objeto original
	Mat HoloReal(T, T, CV_64F, Scalar(0));
	Mat HoloImag(T, T, CV_64F, Scalar(0));

	//Operaciones para calcular el centro del objeto
	int centroX = HoloReal.cols / 2;
	int centroY = HoloReal.rows / 2;
	double lamb = 0.635;
	int dy, dx;
	dy = 4; // Resolución 
	dx = dy;
	double rh = 0;
	double z0 = 50000;	//Distancia al holograma

	//Implementaíón de la formúla básica de Point Cloud
	for (int j = 0; j < TamNube; j++) {
		for (int yh = 0; yh< HoloReal.rows; yh++){
			for (int xh = 0; xh < HoloReal.cols; xh++){
				xp = points.at<float>(j, 0) * 1000;
				yp = points.at<float>(j, 1) * 1000;
				zp = points.at<float>(j, 2) * 1000;
				double x0 = (xh - centroX) * dx;
				double y0 = (centroY - yh) * dy;
				rh = sqrt(pow(xp-x0,2) + pow(yp-y0,2) + pow(z0-zp,2)); 
				double phasephi = ((2 * M_PI) / lamb) * rh;
				HoloReal.at<double>(yh, xh) += cos(phasephi) * (255/rh);
				HoloImag.at<double>(yh, xh) += sin(phasephi) * (255/rh);
			}
		}
	} 
	
	//Comienza propagacion
	fourierMethod_PC(HoloReal, HoloImag, false);
	mostrarMagPha_PC(HoloReal, HoloImag);
	Mat Fo[] = { HoloReal, HoloImag };
	
	Mat PropReal(T, T, CV_64F, Scalar(0));
	Mat PropImag(T, T, CV_64F, Scalar(0));
	PropPC(PropReal, PropImag, centroX, centroY, dx, dy, z0);
	PropImag = PropImag * -1;	//Retropropagación
	mostrarMagPha_PC(PropReal, PropImag);

	fourierMethod_PC(PropReal, PropImag, false);
	mostrarMagPha_PC(PropReal, PropImag);
	Mat Fc[] = { PropReal, PropImag };
	
	Mat Fh[] = { Mat(T, T, CV_64F), Mat(T, T, CV_64F) };
	multiplyI(Fo, Fc, Fh); 
	fourierMethod(Fh, true);
	mostrarMagPha_PC(Fh[0], Fh[1]);

	return (0);
}
