#if !defined(CLASSIFIER_H)
#define CLASSIFIER_H

/*
 *******************************************************************************
 *                        (C) Copyright 2020 IoT Seminar                       *
 * Created: 01/03/2020                                                         *
 *                                                                             *
 * Programmer(s):                                                              *
 * - Sonnya Dellarosa                                                          *
 * - Charles Randolph                                                          *
 *                                                                             *
 * Description:                                                                *
 *  Functions necessary for classifications                                    *
 *                                                                             *
 *******************************************************************************
*/


#include "math.h"
#include "config.h"
#include "mpu6050.h"
#include "msg.h"


/*
 *******************************************************************************
 *                             Symbolic Constants                              *
 *******************************************************************************
*/


// K value of KNN
#define K_VALUE                            11

// Sampling frequency
#define FREQUENCY                          40

// Gyroscope sensitivity value used for conversion
#define GYROSCOPE_SENSITIVITY			   131

// Accelerometer sensitivity value used for conversion
#define ACCELEROMETER_SENSITIVITY		   16384

// Value that describes the weight of the next sample in the low pass filter
#define ALPHA							   0.1

// Handy MAX macro
#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})


/*
 *******************************************************************************
 *                              Type Definitions                               *
 *******************************************************************************
*/


// Structure describing a neighbor
typedef struct {
    double distance;
    brush_zone_t brush_zone;
} neighbor_t;


// Structure describing a sample in a training dataset
typedef struct {
	double pitch;
	double roll;
    brush_zone_t brush_zone;
} trained_data_t;


/*
 *******************************************************************************
 *                            Function Declarations                            *
 *******************************************************************************
*/


/* @brief Returns the pitch angle calculated from sensor values
 * 
 * @param 
 * - data: a sample data containing the sensor values
 * 
 * @return double: the pitch angle
*/
double calculate_pitch(mpu6050_data_t *data_p);


/* @brief Returns the roll angle calculated from sensor values
 * 
 * @param 
 * - data: a sample data containing the sensor values
 * 
 * @return double: the roll angle
*/
double calculate_roll(mpu6050_data_t *data_p);


/* @brief Returns the Euclidean distance between two points in a 2D plane
 * 
 * @param 
 * - a1: x-value of point 1
 * - b1: y-value of point 1
 * - a2: x-value of point 2
 * - b2: y-value of point 2
 * 
 * @return double: the calculated distance
*/
double calculate_distance (double a1, double b1, double a2, double b2);


/* @brief Comparator function to sort by increasing order of distance
 * 
 * @param 
 * - s1: first value in comparison
 * - s2: second value in comparison
 * 
 * @return void
*/
int compare (const void *s1, const void *s2);

/* @brief Applies a filter to the given value. 
 * @param
 * - data_p: Pointer to the sample to filter
 * - last_p: Pointer to the last filtered value
 * @return void
*/
void filter (mpu6050_data_t *data_p, mpu6050_data_t *last_p);


/* @brief Same as train, but allows for real-time training
 * @param
 * - data_p: Pointer to the sample
 * - zone:   The training zone the sample belongs to
 * - n:      The number of the sample (< IMU_TRAINING_SAMPLE_BUF_SIZE)
 * @return void
*/
void train_rt (mpu6050_data_t *data_p, brush_zone_t zone, off_t n);


/* @brief Classifies a sample using the KNN method
 * @note  The set of samples cannot be empty, and is assumed to be
 *        fully set.
 * @param
 * - data_p: The data pointer which will get classified
*/
brush_zone_t classify_rt (mpu6050_data_t *data_p);


#endif