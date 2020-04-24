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
#include "msg.h"
#include "imu.h"
#include "imu_task.h"


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


/* @brief Low pass filter that filters data to remove noise 
 * @note Only use to filter training data for a higher KNN accuracy
 * 
 * @param 
 * - train_data: set of data for training
 * - train_data_filtered: set of filtered training data
 * 
 * @return void
*/
void filter_data(mpu6050_data_t **train_data, mpu6050_data_t **train_data_filtered);


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


/* @brief Returns the Eucledian distance between two points in a 2D plane
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


/* @brief Counts the number of each label in a set of neighbors
 * 
 * @param 
 * - neighbors: set of all neighbors containing the distance with each one
 * - counters: array of pointers to the variable holding the counter of each region
 * 
 * @return void
*/
void count (neighbor_t *neighbors, uint8_t **counters);

/* @brief Generates a training dataset that comprises [pitch, roll, label]
 * 
 * @param 
 * - train_data: set of data for training
 * 
 * @return void
*/
void train (mpu6050_data_t **train_data);


/* @brief Generates the distance of a point to all other points
 * 
 * @param 
 * - pitch: pitch angle of data
 * - roll: roll angle of data
 * - neighbors: pointer to an array holding all calculated distances
 * 
 * @return void
*/
void generate_neighbors (double pitch, double roll, neighbor_t *neighbors);


/* @brief Classifies a sample using the KNN method.
 *
 * @note Sets of samples cannot be empty.
 *
 * @param
 * - sample : sample to be classified
 *
 * @return Label of the new sample
*/
brush_zone_t classify (mpu6050_data_t *sample);

#endif