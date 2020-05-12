//-------------------------------------------------------------------------
// miscellaneous.h 
//-------------------------------------------------------------------------
// Description: miscellaneous functions and macros.
//-------------------------------------------------------------------------
// Author: Carlos Roberto del Blanco Adán, 
//         Grupo de Tratamiento de Imágenes, GTI SSR, Madrid
//		   cda@gti.ssr.upm.es
// Date: 02/12/2007
// Version 1.0
//-------------------------------------------------------------------------

#pragma once
//OpenCV
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"


//***********************************Macros***********************************************
// It is recommended to put into brackets all the arguments to avoid problems in the case that
// one argument will be a mathematical expresion. 

// Access to the image pixels. (i, j) = (row, col).
#define pixelImgU81C_M(img_U81C, i, j)  (((unsigned char*)((img_U81C)->imageData+(i)*(img_U81C)->widthStep))[(j)])
#define pixelImgS161C_M(img_S161C, i, j)  (((short*)((img_S161C)->imageData+(i)*(img_S161C)->widthStep))[(j)])
#define pixelImg32F1C_M(img_32F1C, i, j)  (((float*)((img_32F1C)->imageData+(i)*(img_32F1C)->widthStep))[(j)])

// Access to the matrix elements. (i, j) = (row, col).
#define elemMat32F1C_M(mat_32F1C, i, j)  (((float*)((mat_32F1C)->data.ptr+(i)*(mat_32F1C)->step))[(j)])

// Access to the tridimensional matrix elements. (i, j) = (row, col).
#define elemMat3D32F_M(mat_3D32F, i, j, l)  (((float*)((mat_3D32F)->data.ptr+(i)*((mat_3D32F)->dim[0].size*4)+(j)*((mat_3D32F)->dim[1].size*4))[(l)])

// Check that the coordinates (row,col) belong to the image.
#define checkMargins(image, row, col) (row) >= 0 && (row) < (image)->height && (col) >= 0 && (col) < (image)->width 

// Check that the squared region defined by its center (row,col) and its half width (hwidth) belongs to the image.
#define checkSquareReg(image, row, col, hwidth) ((row)-hwidth) >= 0 && ((row)+hwidth) < (image)->height && ((col)-hwidth) >= 0 && ((col)+hwidth) < (image)->width
//****************************************************************************************

// Converts an image of type XX1C to U81C and scales the intensity values to show the maximum dynamic range of the image. 
void scaleXX1CtoU81C(IplImage* imgXX1C, IplImage* imgU81C);

// Draws an array of points as white points in a black and white image.
void drawPtosBW(IplImage* imgU81C, CvPoint2D32f* ptos, int noPtos);


// Print the values of a float vector.
void printVectorFloat(float* vector, int length, int newline);

// Draw singular points on an image.
void drawPtosIm(IplImage* imgU83C, CvPoint2D32f* singPtos, int noSingPtos);

// Draw correspondences as a sparse motion vector field.
void drawCorr(IplImage* imgU83C, CvPoint2D32f** correspondences, int noCorr);

//Print correspondence vector values.
void printCorr(CvPoint2D32f** correspondences, int noCorr);	
