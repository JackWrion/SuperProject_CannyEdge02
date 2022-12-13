#ifndef CANNY_H_INCLUDED
#define CANNY_H_INCLUDED


//#include <stdio.h>
//#include <stdlib.h>
#include "math.h"
#include "string.h"
//#include <time.h>
//#include "road2.h"

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#define CANNY_LIB_VERBOSE 0
#define BOOSTBLURFACTOR 90.0
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define NOEDGE 255
#define POSSIBLE_EDGE 128
#define EDGE 0



void gaussian_smooth(unsigned char *image, int rows, int cols, float sigma,
    short int **smoothedim);
void make_gaussian_kernel(float sigma, float **kernel, int *windowsize);
void derrivative_x_y(short int *smoothedim, int rows, int cols,
    short int **delta_x, short int **delta_y);
void magnitude_x_y(short int *delta_x, short int *delta_y, int rows, int cols,
    short int **magnitude);
void apply_hysteresis(short int *mag, unsigned char *nms, int rows, int cols,
    float tlow, float thigh, unsigned char *edge);
//void radian_direction(short int *delta_x, short int *delta_y, int rows,
    //int cols, float **dir_radians, int xdirtag, int ydirtag);
//double angle_radians(double x, double y);



/*******************************************************************************
* PROCEDURE: follow_edges
* PURPOSE: This procedure edges is a recursive routine that traces edgs along
* all paths whose magnitude values remain above some specifyable lower
* threshhold.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void follow_edges(unsigned char *edgemapptr, short *edgemagptr, short lowval,
    int cols)
{
    short *tempmagptr;
    unsigned char *tempmapptr;
    int i;
    //float thethresh;
    int x[8] = { 1, 1, 0, -1, -1, -1, 0, 1 },
        y[8] = { 0, 1, 1, 1, 0, -1, -1, -1 };

    for (i = 0; i<8; i++){
        tempmapptr = edgemapptr - y[i] * cols + x[i];
        tempmagptr = edgemagptr - y[i] * cols + x[i];

        if ((*tempmapptr == POSSIBLE_EDGE) && (*tempmagptr > lowval)){
            *tempmapptr = (unsigned char)EDGE;
            follow_edges(tempmapptr, tempmagptr, lowval, cols);
        }
    }
}

/*******************************************************************************
* PROCEDURE: apply_hysteresis
* PURPOSE: This routine finds edges that are above some high threshhold or
* are connected to a high pixel by a path of pixels greater than a low
* threshold.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void apply_hysteresis(short int *mag, unsigned char *nms, int rows, int cols,
    float tlow, float thigh, unsigned char *edge)
{
    int r, c, pos, numedges, highcount, lowthreshold, highthreshold, hist[32768];
    //int rr,cc,i,lowcount;
    short int maximum_mag;
    //short int sumpix;

    /****************************************************************************
    * Initialize the edge map to possible edges everywhere the non-maximal
    * suppression suggested there could be an edge except for the border. At
    * the border we say there can not be an edge because it makes the
    * follow_edges algorithm more efficient to not worry about tracking an
    * edge off the side of the image.
    ****************************************************************************/
    for (r = 0, pos = 0; r<rows; r++){
        for (c = 0; c<cols; c++, pos++){
            if (nms[pos] == POSSIBLE_EDGE) edge[pos] = POSSIBLE_EDGE;
            else edge[pos] = NOEDGE;
        }
    }

    for (r = 0, pos = 0; r<rows; r++, pos += cols){
        edge[pos] = NOEDGE;
        edge[pos + cols - 1] = NOEDGE;
    }
    pos = (rows - 1) * cols;
    for (c = 0; c<cols; c++, pos++){
        edge[c] = NOEDGE;
        edge[pos] = NOEDGE;
    }

    /****************************************************************************
    * Compute the histogram of the magnitude image. Then use the histogram to
    * compute hysteresis thresholds.
    ****************************************************************************/
    for (r = 0; r<32768; r++) hist[r] = 0;

//    struct timespec start;
    //clock_gettime(CLOCK_MONOTONIC, &start);

    for (r = 0, pos = 0; r<rows; r++){
        for (c = 0; c<cols; c++, pos++){
            if (edge[pos] == POSSIBLE_EDGE) hist[mag[pos]]++;
        }
    }

    //printf("Hist loop in function %s runtime: %.3f\n", __FUNCTION__, get_runtime(start));

    /****************************************************************************
    * Compute the number of pixels that passed the nonmaximal suppression.
    ****************************************************************************/
    for (r = 1, numedges = 0; r<32768; r++){
        if (hist[r] != 0) maximum_mag = r;
        numedges += hist[r];
    }

    highcount = (int)(numedges * thigh + 0.5);

    /****************************************************************************
    * Compute the high threshold value as the (100 * thigh) percentage point
    * in the magnitude of the gradient histogram of all the pixels that passes
    * non-maximal suppression. Then calculate the low threshold as a fraction
    * of the computed high threshold value. John Canny said in his paper
    * "A Computational Approach to Edge Detection" that "The ratio of the
    * high to low threshold in the implementation is in the range two or three
    * to one." That means that in terms of this implementation, we should
    * choose tlow ~= 0.5 or 0.33333.
    ****************************************************************************/
    r = 1;
    numedges = hist[1];
    while ((r<(maximum_mag - 1)) && (numedges < highcount)){
        r++;
        numedges += hist[r];
    }
    highthreshold = r;
    lowthreshold = (int)(highthreshold * tlow + 0.5);

    /****************************************************************************
    * This loop lor pixels above the highthreshold to locate edges and
    * then calls follow_edges to continue the edge.
    ****************************************************************************/
    for (r = 0, pos = 0; r<rows; r++){
        for (c = 0; c<cols; c++, pos++){
            if ((edge[pos] == POSSIBLE_EDGE) && (mag[pos] >= highthreshold)){
                edge[pos] = EDGE;
                follow_edges((edge + pos), (mag + pos), lowthreshold, cols);
            }
        }
    }

    /****************************************************************************
    * Set all the remaining possible edges to non-edges.
    ****************************************************************************/
    for (r = 0, pos = 0; r<rows; r++){
        for (c = 0; c<cols; c++, pos++) if (edge[pos] != EDGE) edge[pos] = NOEDGE;
    }
}

/*******************************************************************************
* PROCEDURE: non_max_supp
* PURPOSE: This routine applies non-maximal suppression to the magnitude of
* the gradient image.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void non_max_supp(short *mag, short *gradx, short *grady, int nrows, int ncols,
    unsigned char *result)
{
    int rowcount, colcount, count;
    short *magrowptr, *magptr;
    short *gxrowptr, *gxptr;
    short *gyrowptr, *gyptr, z1, z2;
    short m00, gx, gy;
    float mag1, mag2, xperp, yperp;
    unsigned char *resultrowptr, *resultptr;


    /****************************************************************************
    * Zero the edges of the result image.
    ****************************************************************************/
    for (count = 0, resultrowptr = result, resultptr = result + ncols*(nrows - 1);
        count<ncols; resultptr++, resultrowptr++, count++){
        *resultrowptr = *resultptr = (unsigned char)0;
    }

    for (count = 0, resultptr = result, resultrowptr = result + ncols - 1;
        count<nrows; count++, resultptr += ncols, resultrowptr += ncols){
        *resultptr = *resultrowptr = (unsigned char)0;
    }
/*
  struct timespec start;
  clock_gettime(CLOCK_MONOTONIC, &start);
*/
    /****************************************************************************
    * Suppress non-maximum points.
    ****************************************************************************/
    for (rowcount = 1, magrowptr = mag + ncols + 1, gxrowptr = gradx + ncols + 1,
        gyrowptr = grady + ncols + 1, resultrowptr = result + ncols + 1;
        rowcount<nrows - 2;
    rowcount++, magrowptr += ncols, gyrowptr += ncols, gxrowptr += ncols,
        resultrowptr += ncols){
        for (colcount = 1, magptr = magrowptr, gxptr = gxrowptr, gyptr = gyrowptr,
            resultptr = resultrowptr; colcount<ncols - 2;
            colcount++, magptr++, gxptr++, gyptr++, resultptr++){
            m00 = *magptr;
            if (m00 == 0){
                *resultptr = (unsigned char)NOEDGE;
            }
            else{
                xperp = -(gx = *gxptr) / ((float)m00);
                yperp = (gy = *gyptr) / ((float)m00);
            }

            if (gx >= 0){
                if (gy >= 0){
                    if (gx >= gy)
                    {
                        /* 111 */
                        /* Left point */
                        z1 = *(magptr - 1);
                        z2 = *(magptr - ncols - 1);

                        mag1 = (m00 - z1)*xperp + (z2 - z1)*yperp;

                        /* Right point */
                        z1 = *(magptr + 1);
                        z2 = *(magptr + ncols + 1);

                        mag2 = (m00 - z1)*xperp + (z2 - z1)*yperp;
                    }
                    else
                    {
                        /* 110 */
                        /* Left point */
                        z1 = *(magptr - ncols);
                        z2 = *(magptr - ncols - 1);

                        mag1 = (z1 - z2)*xperp + (z1 - m00)*yperp;

                        /* Right point */
                        z1 = *(magptr + ncols);
                        z2 = *(magptr + ncols + 1);

                        mag2 = (z1 - z2)*xperp + (z1 - m00)*yperp;
                    }
                }
                else
                {
                    if (gx >= -gy)
                    {
                        /* 101 */
                        /* Left point */
                        z1 = *(magptr - 1);
                        z2 = *(magptr + ncols - 1);

                        mag1 = (m00 - z1)*xperp + (z1 - z2)*yperp;

                        /* Right point */
                        z1 = *(magptr + 1);
                        z2 = *(magptr - ncols + 1);

                        mag2 = (m00 - z1)*xperp + (z1 - z2)*yperp;
                    }
                    else
                    {
                        /* 100 */
                        /* Left point */
                        z1 = *(magptr + ncols);
                        z2 = *(magptr + ncols - 1);

                        mag1 = (z1 - z2)*xperp + (m00 - z1)*yperp;

                        /* Right point */
                        z1 = *(magptr - ncols);
                        z2 = *(magptr - ncols + 1);

                        mag2 = (z1 - z2)*xperp + (m00 - z1)*yperp;
                    }
                }
            }
            else
            {
                if ((gy = *gyptr) >= 0)
                {
                    if (-gx >= gy)
                    {
                        /* 011 */
                        /* Left point */
                        z1 = *(magptr + 1);
                        z2 = *(magptr - ncols + 1);

                        mag1 = (z1 - m00)*xperp + (z2 - z1)*yperp;

                        /* Right point */
                        z1 = *(magptr - 1);
                        z2 = *(magptr + ncols - 1);

                        mag2 = (z1 - m00)*xperp + (z2 - z1)*yperp;
                    }
                    else
                    {
                        /* 010 */
                        /* Left point */
                        z1 = *(magptr - ncols);
                        z2 = *(magptr - ncols + 1);

                        mag1 = (z2 - z1)*xperp + (z1 - m00)*yperp;

                        /* Right point */
                        z1 = *(magptr + ncols);
                        z2 = *(magptr + ncols - 1);

                        mag2 = (z2 - z1)*xperp + (z1 - m00)*yperp;
                    }
                }
                else
                {
                    if (-gx > -gy)
                    {
                        /* 001 */
                        /* Left point */
                        z1 = *(magptr + 1);
                        z2 = *(magptr + ncols + 1);

                        mag1 = (z1 - m00)*xperp + (z1 - z2)*yperp;

                        /* Right point */
                        z1 = *(magptr - 1);
                        z2 = *(magptr - ncols - 1);

                        mag2 = (z1 - m00)*xperp + (z1 - z2)*yperp;
                    }
                    else
                    {
                        /* 000 */
                        /* Left point */
                        z1 = *(magptr + ncols);
                        z2 = *(magptr + ncols + 1);

                        mag1 = (z2 - z1)*xperp + (m00 - z1)*yperp;

                        /* Right point */
                        z1 = *(magptr - ncols);
                        z2 = *(magptr - ncols - 1);

                        mag2 = (z2 - z1)*xperp + (m00 - z1)*yperp;
                    }
                }
            }

            /* Now determine if the current point is a maximum point */

            if ((mag1 > 0.0) || (mag2 > 0.0))
            {
                *resultptr = (unsigned char)NOEDGE;
            }
            else
            {
                if (mag2 == 0.0)
                    *resultptr = (unsigned char)NOEDGE;
                else
                    *resultptr = (unsigned char)POSSIBLE_EDGE;
            }
        }
    }

  //  printf("Function %s runtime: %.3f\n", __FUNCTION__, get_runtime(start));

}

/*******************************************************************************
* PROCEDURE: canny
* PURPOSE: To perform canny edge detection.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void canny(unsigned char *image, int rows, int cols, float sigma,
    float tlow, float thigh, unsigned char **edge, char *fname)
{
    //FILE *fpdir = NULL;          /* File to write the gradient image to.     */
    unsigned char *nms;        /* Points that are local maximal magnitude. */
    short int *smoothedim,     /* The image after gaussian smoothing.      */
        *delta_x,        /* The first devivative image, x-direction. */
        *delta_y,        /* The first derivative image, y-direction. */
        *magnitude;      /* The magnitude of the gadient image.      */
    //int r, c, pos;
    //float *dir_radians = NULL;   /* Gradient direction image.                */

    /****************************************************************************
    * Perform gaussian smoothing on the image using the input standard
    * deviation.
    ****************************************************************************/

    gaussian_smooth(image, rows, cols, sigma, &smoothedim);

    /****************************************************************************
    * Compute the first derivative in the x and y directions.
    ****************************************************************************/
    derrivative_x_y(smoothedim, rows, cols, &delta_x, &delta_y);

    free(smoothedim);

    /****************************************************************************
    * Compute the magnitude of the gradient.
    ****************************************************************************/
    magnitude_x_y(delta_x, delta_y, rows, cols, &magnitude);

    /****************************************************************************
    * Perform non-maximal suppression.
    ****************************************************************************/
    if ((nms = (unsigned char *)calloc(rows*cols, sizeof(unsigned char))) == NULL){
        fprintf(stderr, "Error allocating the nms image.\n");
        exit(1);
    }

    non_max_supp(magnitude, delta_x, delta_y, rows, cols, nms);

    free(delta_x);
    free(delta_y);
    /****************************************************************************
    * Use hysteresis to mark the edge pixels.
    ****************************************************************************/
    if (CANNY_LIB_VERBOSE) printf("Doing hysteresis thresholding.\n");

    if ((*edge = (unsigned char *)calloc(rows*cols, sizeof(unsigned char))) == NULL){
        fprintf(stderr, "Error allocating the edge image.\n");
        exit(1);
    }

    apply_hysteresis(magnitude, nms, rows, cols, tlow, thigh, *edge);

    /****************************************************************************
    * Free all of the memory that we allocated except for the edge image that
    * is still being used to store out result.
    ****************************************************************************/

    free(magnitude);
    free(nms);
}

/*******************************************************************************
* Procedure: radian_direction
* Purpose: To compute a direction of the gradient image from component dx and
* dy images. Because not all derriviatives are computed in the same way, this
* code allows for dx or dy to have been calculated in different ways.
*
* FOR X:  xdirtag = -1  for  [-1 0  1]
*         xdirtag =  1  for  [ 1 0 -1]
*
* FOR Y:  ydirtag = -1  for  [-1 0  1]'
*         ydirtag =  1  for  [ 1 0 -1]'
*
* The resulting angle is in radians measured counterclockwise from the
* xdirection. The angle points "up the gradient".
*******************************************************************************/

/*
void radian_direction(short int *delta_x, short int *delta_y, int rows,
    int cols, float **dir_radians, int xdirtag, int ydirtag)
{
    int r, c, pos;
    float *dirim = NULL;
    double dx, dy;


    if ((dirim = (float *)calloc(rows*cols, sizeof(float))) == NULL){
        fprintf(stderr, "Error allocating the gradient direction image.\n");
        exit(1);
    }
    *dir_radians = dirim;

    for (r = 0, pos = 0; r<rows; r++){
        for (c = 0; c<cols; c++, pos++){
            dx = (double)delta_x[pos];
            dy = (double)delta_y[pos];

            if (xdirtag == 1) dx = -dx;
            if (ydirtag == -1) dy = -dy;

            dirim[pos] = (float)angle_radians(dx, dy);
        }
    }
}

*/

/*******************************************************************************
* FUNCTION: angle_radians
* PURPOSE: This procedure computes the angle of a vector with components x and
* y. It returns this angle in radians with the answer being in the range
* 0 <= angle <2*PI.
*******************************************************************************/
/*
double angle_radians(double x, double y)
{
    double xu, yu, ang;

    xu = fabs(x);
    yu = fabs(y);

    if ((xu == 0) && (yu == 0)) return(0);

    ang = atan(yu / xu);

    if (x >= 0){
        if (y >= 0) return(ang);
        else return(2 * M_PI - ang);
    }
    else{
        if (y >= 0) return(M_PI - ang);
        else return(M_PI + ang);
    }
}

*/
/*******************************************************************************
* PROCEDURE: magnitude_x_y
* PURPOSE: Compute the magnitude of the gradient. This is the square root of
* the sum of the squared derivative values.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void magnitude_x_y(short int *delta_x, short int *delta_y, int rows, int cols,
    short int **magnitude)
{
    int r, c, pos, sq1, sq2;

    /****************************************************************************
    * Allocate an image to store the magnitude of the gradient.
    ****************************************************************************/
    if ((*magnitude = (short *)calloc(rows*cols, sizeof(short))) == NULL){
        fprintf(stderr, "Error allocating the magnitude image.\n");
        exit(1);
    }

    //struct timespec start;
    //clock_gettime(CLOCK_MONOTONIC, &start);

    for (r = 0, pos = 0; r<rows; r++){
        for (c = 0; c<cols; c++, pos++){
            sq1 = (int)delta_x[pos] * (int)delta_x[pos];
            sq2 = (int)delta_y[pos] * (int)delta_y[pos];
            (*magnitude)[pos] = (short)(0.5 + sqrt((float)sq1 + (float)sq2));
        }
    }

    //printf("Loop in function %s runtime: %.3f\n", __FUNCTION__, get_runtime(start));

}

/*******************************************************************************
* PROCEDURE: derrivative_x_y
* PURPOSE: Compute the first derivative of the image in both the x any y
* directions. The differential filters that are used are:
*
*                                          -1
*         dx =  -1 0 +1     and       dy =  0
*                                          +1
*
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void derrivative_x_y(short int *smoothedim, int rows, int cols,
    short int **delta_x, short int **delta_y)
{
    int r, c, pos;

    /****************************************************************************
    * Allocate images to store the derivatives.
    ****************************************************************************/
    if (((*delta_x) = (short *)calloc(rows*cols, sizeof(short))) == NULL){
        fprintf(stderr, "Error allocating the delta_x image.\n");
        exit(1);
    }
    if (((*delta_y) = (short *)calloc(rows*cols, sizeof(short))) == NULL){
        fprintf(stderr, "Error allocating the delta_x image.\n");
        exit(1);
    }

    /****************************************************************************
    * Compute the x-derivative. Adjust the derivative at the borders to avoid
    * losing pixels.
    ****************************************************************************/
    if (CANNY_LIB_VERBOSE) printf("   Computing the X-direction derivative.\n");

    //struct timespec start;
    //clock_gettime(CLOCK_MONOTONIC, &start);

    for (r = 0; r<rows; r++){
        pos = r * cols;
        (*delta_x)[pos] = smoothedim[pos + 1] - smoothedim[pos];
        pos++;
        for (c = 1; c<(cols - 1); c++, pos++){
            (*delta_x)[pos] = smoothedim[pos + 1] - smoothedim[pos - 1];
        }
        (*delta_x)[pos] = smoothedim[pos] - smoothedim[pos - 1];
    }

    /****************************************************************************
    * Compute the y-derivative. Adjust the derivative at the borders to avoid
    * losing pixels.
    ****************************************************************************/
    if (CANNY_LIB_VERBOSE) printf("   Computing the Y-direction derivative.\n");
    for (c = 0; c < cols; c++) {
        pos = c;
        (*delta_y)[pos] = smoothedim[pos + cols] - smoothedim[pos];
        pos = rows * (rows - 1) + c;
        (*delta_y)[pos] = smoothedim[pos] - smoothedim[pos - cols];
    }
    for (r = 1; r < (rows - 1); r++) {
        for (c = 0; c < cols; c++) {
            pos = r * cols + c;
            (*delta_y)[pos] = smoothedim[pos + cols] - smoothedim[pos - cols];
        }
    }

    //printf("Loop in function %s runtime: %.3f\n", __FUNCTION__, get_runtime(start));

}

/*******************************************************************************
* PROCEDURE: gaussian_smooth
* PURPOSE: Blur an image with a gaussian filter.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void gaussian_smooth(unsigned char *image, int rows, int cols, float sigma,
    short int **smoothedim)
{
    int r, c, rr, cc,     /* Counter variables. */
        windowsize,        /* Dimension of the gaussian kernel. */
        center;            /* Half of the windowsize. */
    float *tempim,        /* Buffer for separable filter gaussian smoothing. */
        *kernel = NULL;        /* A one dimensional gaussian kernel. */
    //float  dot,            /* Dot product summing variable. */
        //sum;            /* Sum of the kernel weights variable. */

    /****************************************************************************
    * Create a 1-dimensional gaussian smoothing kernel.
    ****************************************************************************/

    float *ktmp = kernel;

    make_gaussian_kernel(sigma, &ktmp, &windowsize);
    kernel = ktmp;
    center = windowsize / 2;

    /****************************************************************************
    * Allocate a temporary buffer image and the smoothed image.
    ****************************************************************************/
    if ((tempim = (float *)calloc(rows*cols, sizeof(float))) == NULL){
        fprintf(stderr, "Error allocating the buffer image.\n");
        exit(1);
    }
    if (((*smoothedim) = (short int *)calloc(rows*cols,
        sizeof(short int))) == NULL){
        fprintf(stderr, "Error allocating the smoothed image.\n");
        exit(1);
    }

    /****************************************************************************
    * Blur in the x - direction.
    ****************************************************************************/

    float* dot_arr = calloc(cols, sizeof(float));
    float* sum_arr = calloc(cols, sizeof(float));

    for (r = 0; r < rows; r++) {
        memset(dot_arr, 0, cols * sizeof(float));
        memset(sum_arr, 0, cols * sizeof(float));
        for (cc = (-center); cc <= center; cc++) {
            for (c = MAX(0, -cc); c < MIN(cols, cols - cc); c++) {
                dot_arr[c] +=
                    (float)image[r * cols + (c + cc)] * kernel[center + cc];
                sum_arr[c] += kernel[center + cc];
            }
        }

        for (c = 0; c < cols; c++) {
            tempim[r * cols + c] = dot_arr[c] / sum_arr[c];
        }
    }

    /****************************************************************************
    * Blur in the y - direction.
    ****************************************************************************/

    for (r = 0; r < rows; r++) {
        memset(dot_arr, 0, cols * sizeof(float));
        memset(sum_arr, 0, cols * sizeof(float));
        for (rr = (-center); rr <= center; rr++) {
            if (((r + rr) >= 0) && ((r + rr) < rows)) {
                for (c = 0; c < cols; c++) {
                    dot_arr[c] +=
                        tempim[(r + rr) * cols + c] * kernel[center + rr];
                    sum_arr[c] += kernel[center + rr];
                }
            }
        }
        for (c = 0; c < cols; c++) {
            (*smoothedim)[r * cols + c] =
                (short int)(dot_arr[c] * BOOSTBLURFACTOR / sum_arr[c] + 0.5);
        }
    }
    free(dot_arr);
    free(sum_arr);

    //printf("Loops in function %s runtime: %.3f\n", __FUNCTION__, get_runtime(start));

    free(tempim);
    free(kernel);
}

/*******************************************************************************
* PROCEDURE: make_gaussian_kernel
* PURPOSE: Create a one dimensional gaussian kernel.
* NAME: Mike Heath
* DATE: 2/15/96
*******************************************************************************/
void make_gaussian_kernel(float sigma, float **kernel, int *windowsize)
{
    int i, center;
    float x, fx, sum = 0.0;

    *windowsize = 1 + 2 * ceil(2.5 * sigma);
    center = (*windowsize) / 2;


    if ((*kernel = (float *)calloc((*windowsize), sizeof(float))) == NULL){
        fprintf(stderr, "Error callocing the gaussian kernel array.\n");
        exit(1);
    }

    for (i = 0; i<(*windowsize); i++){
        x = (float)(i - center);
        fx = pow(2.71828, -0.5*x*x / (sigma*sigma)) / (sigma * sqrt(6.2831853));
        (*kernel)[i] = fx;
        sum += fx;
    }

    for (i = 0; i<(*windowsize); i++) (*kernel)[i] /= sum;

}


#define CANNY_TEST_VERBOSE 1
#define TI_BUILD 1




#endif // CANNY_H_INCLUDED
