#ifdef _MSC_VER					
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include "propagationF.h" 
#define A 255; //Intensidad  
using namespace cv;
using namespace std;

//-------Escalas reales para desplegar en el modulador--------//
//const int T = 4096;
//int HI = 1024;
//float scale = 750;

//--------Escalas ideales---------//
const int T = 1024;
 int HI = 780;
float scale = 50;

//Compara dos puntos de la lista OrdenZ y devuelve el menor
bool compareTwoPoints(Point3f a, Point3f b)
{
	return (a.z < b.z);
}

struct PlateComplex{
	Mat Real, Imag;
};

int main()
{
	//Lectura de datos
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\oscar\\OneDrive\\Documentos\\Delfín2022\\PointCloudD\\PointCloudD\\Banana.pcd", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	//Matriz de puntos
	int TamNube = cloud->size();//Tamaño de la nube 
	int tam = cloud->width * cloud->height;
	Mat points(tam, 3, CV_32F);

	int i = 0;
	double xp, yp, zp;

	Mat Ztotal(tam, 1, CV_32F);
	vector<Point3f> OrdenZ;
	for (const auto& point : *cloud) {
		if (!(isnan(point.x) || isnan(point.y) || isnan(point.z))) {
			//std::cout << "    " << point.x << " " << point.y << " " << point.z << std::endl;
			//Le pasa los datos al objeto Mat
			points.at<float>(i, 0) = point.x;
			points.at<float>(i, 1) = point.y;
			points.at<float>(i, 2) = point.z;
			Ztotal.at<float>(i, 0) = point.z;
			OrdenZ.push_back(Point3f(point.x, point.y, point.z));
			i++;
		}
	}
	sort(OrdenZ.begin(), OrdenZ.end(), compareTwoPoints);//Ordena los puntos
	double zmin, zmax;
	zmin = OrdenZ[0].z;//Z minimo
	zmax = OrdenZ[tam-1].z; //Z maximo
	//double NofP = abs(zmin - zmax);

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
	dy = 4;// Resolución 
	dx = dy;
	double rh = 0;
	double z0 = 50000; //Distancia al holograma puede ser de 50000(5cm) o 35000(3.5cm)
	
	double DeltaX = 500; //Distancia entre cada centro del vector
	double DisC = 250; //Mitad de un centro
	double ZminR = zmin * 1000;	
	double ZmaxR = zmax * 1000;
	double Cmin = (ZminR - DisC);	
	double Cmax = (ZmaxR + DisC);

	vector<double> CentroPlates;
	//Calcula la cantidad de platos 
	while (Cmin){
		if (Cmin > Cmax){
			CentroPlates.push_back(Cmin);
			break;
		}
		else{
			CentroPlates.push_back(Cmin);//Vector de Centros
			Cmin = Cmin + DeltaX;
		}
	}
	//Para todos los puntos que se encuentren en cada Plate, le corresponde una matriz de 100 * 100

	int TamPlate = 300;
	int TotPlates = CentroPlates.size();
	vector<PlateComplex*> Table;
	for (int x = 0; x < TotPlates ; x++) {
		PlateComplex* Plate1 = new PlateComplex();
		Plate1->Real = Mat(TamPlate, TamPlate, CV_64F);
		Plate1->Imag = Mat(TamPlate, TamPlate, CV_64F);
		Table.push_back(Plate1);
	}
	
	int centro2x, centro2y;

	centro2x = TamPlate / 2;
	centro2y = TamPlate / 2;

	//Fase aleatoria//
	/*Mat PhaseR = Mat(T, T, CV_64F);
	randu(PhaseR, Scalar(0.), Scalar(1.));
	Mat phi = PhaseR * 2 * M_PI;*/

	for (int yh = 0; yh < TamPlate; yh++) {
		for (int xh = 0; xh < TamPlate; xh++) {
			double x0 = (xh - centro2x) * dx;	
			double y0 = (centro2y - yh) * dy;
			
			//double phix = cos(phi.at<double>(yh, xh));
			//double phiy = sin(phi.at<double>(yh, xh));

			for (int i = 0; i < CentroPlates.size(); i++) { //Cambiar for al numero de rebanadas 
				double zi = z0 + CentroPlates[i];
				rh = sqrt(pow(x0, 2) + pow(y0, 2) + pow(zi, 2));
				double phasephi = ((2 * M_PI) / lamb) * rh;
				Table[i]->Real.at<double>(yh, xh) = cos(phasephi) * (255 / rh);
				Table[i]->Imag.at<double>(yh, xh) = sin(phasephi) * (255 / rh); 
			}
			//mostrarMagPha_PC(Table[i]->Real, Table[i]->Imag);
		}
	}
	//copiar table[indexP] real/imaginaria al real/imaginario del holograma
	//Usando las coordenadas OrdenZ[i](x,y)
	int indexP = 0;	//Indice del plato
	for (int i = 0; i < OrdenZ.size(); i++) {
		double PointZ = OrdenZ[i].z * scale/10;
		double xp = OrdenZ[i].x * scale;
		double yp = OrdenZ[i].y * scale;
		double xi = (xp / dx) + centroX;
		double yi = centroY - (yp / dy) ;
		if (PointZ < CentroPlates[indexP+1]) {	//Si el punto Z es mayor o igual al centro del plato actual entonces se guardara en el plato actual
			if ((xi - (TamPlate/2)) > 0 && (xi + (TamPlate / 2)) < HoloReal.rows) {
				if ((yi - (TamPlate / 2)) > 0 && (yi + (TamPlate / 2)) < HoloReal.cols) {
					Mat temp = HoloReal.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2));
					temp += Table[indexP]->Real;
					Mat temp2 = HoloImag.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2));
					temp2 += Table[indexP]->Imag;
					//Table[indexP]->Real.copyTo(HoloReal.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2)));
					//Table[indexP]->Imag.copyTo(HoloImag.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2)));
					temp.copyTo(HoloReal.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2)));
					temp2.copyTo(HoloImag.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2)));
				}
			}
		}
		else{
			indexP++;//Avanza el indice del plato
			if ((xi - (TamPlate / 2)) > 0 && (xi + (TamPlate / 2)) < HoloReal.rows) {
				if ((yi - (TamPlate / 2)) > 0 && (yi + (TamPlate / 2)) < HoloReal.cols) {
					Mat temp = HoloReal.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2));
					temp += Table[indexP]->Real;
					Mat temp2 = HoloImag.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2));
					temp2 += Table[indexP]->Imag;
					//Table[indexP]->Real.copyTo(HoloReal.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2)));
					//Table[indexP]->Imag.copyTo(HoloImag.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2)));
					temp.copyTo(HoloReal.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2)));
					temp2.copyTo(HoloImag.colRange(xi - (TamPlate / 2), xi + (TamPlate / 2)).rowRange(yi - (TamPlate / 2), yi + (TamPlate / 2)));
				}
			}
		}
		//mostrarMagPha_PC(HoloReal,HoloImag);
	}
	double V1 = ((T / 2) - (HI / 2));
	double V2 = ((T / 2) + (HI / 2));
	int campo;
	cout << "1. Holograma de magnitud. \n" "2. Holograma de fase. \n3. Holograma de campo completo. \n";cin >> campo;
	if(campo == 1) {
		//---------------Holograma de magnitud---------------// 
		Mat MagHI, MagMap;
		MagHI = HoloReal.clone();
		MagHI.setTo(0);
		//randu(MagHI, Scalar(0.), Scalar(1.));
		magnitude(HoloReal, HoloImag, MagHI);
		double Mmin, Mmax;
		minMaxLoc(abs(MagHI), &Mmin, &Mmax);
		MagMap = MagHI * 255.0 / Mmax;
		Mat MagMap2 = MagMap.clone();
		MagMap.convertTo(MagMap2, CV_8U);
		Mat HoloMag;
		MagMap2.rowRange(V1, V2).colRange(V1, V2).copyTo(HoloMag);
		imwrite("Holograma de Magnitud_6.png", HoloMag);

		
		//Reconstruccion 
		Mat RecReal(T, T, CV_64F);	
		RecReal.setTo(125);
		Mat RecImag(T, T, CV_64F);
		RecImag.setTo(0);
	
		//Copiar HoloMag en e centrol de RecReal
		double CentroXh = HoloMag.cols / 2;
		double CentroYh = HoloMag.rows / 2;
		
		HoloMag.copyTo(RecReal.rowRange(centroX - CentroXh, centroX + CentroXh).colRange(centroY - CentroYh, centroY + CentroYh));
		HoloMag = RecReal; 
		HoloMag.convertTo(RecReal, CV_64F);
	
		//Reconstruccion
		fourierMethod_PC(RecReal, RecImag, false);
		//mostrarMagPha_PC(RecReal, RecImag);
		Mat Fo[] = {RecReal, RecImag};

		Mat PropReal(T , T, CV_64F, Scalar(0));
		Mat PropImag(T, T, CV_64F, Scalar(0));
		PropPC(PropReal, PropImag, centroX, centroY, dx, dy, z0);

		//Retropropagación
		PropImag = PropImag * -1;
		//mostrarMagPha_PC(PropReal, PropImag);

		fourierMethod_PC(PropReal, PropImag, false);
		//mostrarMagPha_PC(PropReal, PropImag);
		Mat Fc[] = { PropReal, PropImag };

		Mat Fh[] = { Mat(T, T, CV_64F), Mat(T, T, CV_64F) };
		multiplyI(Fo, Fc, Fh);
		fourierMethod(Fh, true);
		mostrarMagPha_PC(Fh[0], Fh[1]);
		

	}else if (campo == 2) {
		//---------------Holograma de fase---------------// 
		Mat PhaseH;
		PhaseH = HoloReal.clone();
		PhaseH.setTo(0);
		//randu(PhaseH, Scalar(0.), Scalar(1.));
		
		phase(HoloReal, HoloImag, PhaseH);
		PhaseH = (PhaseH / (2 * M_PI)) * 255.0; 
		Mat PhaseH2 = PhaseH.clone();
		PhaseH.convertTo(PhaseH2, CV_8U);
		Mat HoloPhas(HI, HI, CV_8U);
		PhaseH2.rowRange(V1, V2).colRange(V1, V2).copyTo(HoloPhas);
		imwrite("Holograma de Fase_1.png", HoloPhas);
		//----------------------------------------------------------//
		//----------------------------------------------------------//
		
		//Mapeo 2pi
		//PhaseHMP.rowRange(122, 902).colRange(122, 902).copyTo(HoloPhasMP);
		Mat PhaseHMP(T, T, CV_8U);
		PhaseHMP.setTo(0);
		Mat PhaseH2MP(T, T, CV_64F);
		Mat HoloPhasMP(T, T, CV_64F);
		HoloPhas.copyTo(PhaseHMP.rowRange(V1, V2).colRange(V1, V2));
		PhaseHMP.convertTo(HoloPhasMP, CV_64F);
		HoloPhasMP = (HoloPhasMP / 255.0) * (2 * M_PI);

		Mat RecReal(T, T, CV_64F);	
		RecReal.setTo(125);
		Mat RecImag(T, T, CV_64F);
		RecImag.setTo(125);
		double CentroXh = HoloPhasMP.cols / 2;
		double CentroYh = HoloPhasMP.rows / 2;

		for (int j = 0; j < HoloPhasMP.cols; j++)
		{
			for (int i = 0; i < HoloPhasMP.rows; i++)
			{
				//Formula de euler
				double phix = cos(HoloPhasMP.at<double>(j, i));
				double phiy = sin(HoloPhasMP.at<double>(j, i));
				RecReal.at<double>(j, i) = 255.0 * phix;  // A*cos(M_PI*phaseR);
				RecImag.at<double>(j, i) = 255.0 * phiy;  // A*sin(M_PI*phaseR);
			}
		}
		
		//Reconstruccion//
		fourierMethod_PC(RecReal, RecImag, false);
		//mostrarMagPha_PC(RecReal, RecImag);
		Mat Fo[] = {RecReal, RecImag};

		Mat PropReal(T , T, CV_64F, Scalar(0));
		Mat PropImag(T, T, CV_64F, Scalar(0));
		PropPC(PropReal, PropImag, centroX, centroY, dx, dy, z0);

		//Retropropagación
		PropImag = PropImag * -1;
		//mostrarMagPha_PC(PropReal, PropImag);

		fourierMethod_PC(PropReal, PropImag, false);
		//mostrarMagPha_PC(PropReal, PropImag);
		Mat Fc[] = { PropReal, PropImag };

		Mat Fh[] = { Mat(T, T, CV_64F), Mat(T, T, CV_64F) };
		multiplyI(Fo, Fc, Fh);
		fourierMethod(Fh, true);
		mostrarMagPha_PC(Fh[0], Fh[1]);
	}else{
		//---------------Holograma de de campo completo---------------// 
		
		//Mat Fo[] = { HoloReal, HoloImag };
		fourierMethod_PC(HoloReal, HoloImag, false);
		//mostrarMagPha_PC(HoloReal, HoloImag);
		Mat Fo[] = { HoloReal, HoloImag };

		Mat PropReal(T , T, CV_64F, Scalar(0));
		Mat PropImag(T, T, CV_64F, Scalar(0));
		PropPC(PropReal, PropImag, centroX, centroY, dx, dy, z0);

		//Retropropagación
		PropImag = PropImag * -1;
		//mostrarMagPha_PC(PropReal, PropImag);

		fourierMethod_PC(PropReal, PropImag, false);
		//mostrarMagPha_PC(PropReal, PropImag);
		Mat Fc[] = { PropReal, PropImag };

		Mat Fh[] = { Mat(T, T, CV_64F), Mat(T, T, CV_64F) };
		multiplyI(Fo, Fc, Fh);
		fourierMethod(Fh, true); 
		mostrarMagPha_PC(Fh[0], Fh[1]);
	}
	return (0);
}