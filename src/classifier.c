#include "classifier.h"


/*
 *******************************************************************************
  *                             Global Variables                               *
 *******************************************************************************
*/


// Trained data
static trained_data_t g_trained_data[IMU_TRAINING_SAMPLE_BUF_SIZE  *4];


/*
 *******************************************************************************
  *                       External Function Definitions                        *
 *******************************************************************************
*/

// Function to filter data to remove noise
// Note: only use to filter training data for a higher KNN accuracy
void filter_data(mpu6050_data_t **train_data, mpu6050_data_t **train_data_filtered) {
    
    mpu6050_data_t *current_data_p;               // Current sample being iterated
    mpu6050_data_t *prev_filtered_data_p;         // Previous sample in iteration
    mpu6050_data_t  filtered_data;                // Newly filtered sample
    uint8_t         training_zone = 0;            // Current training zone
    uint8_t         t = 0;                        // Counter of all data point in the training set
    uint8_t         i;

    // Do for each training zone
    while (training_zone < 4) {
        
        for (i = 0; i < IMU_TRAINING_SAMPLE_BUF_SIZE; i++) {

            // Get current sample
            current_data_p = &train_data[training_zone][i];

            if (t > 0) {

                // Get previous sample
                if (training_zone == 0) {
                    prev_filtered_data_p = &train_data_filtered[training_zone][i-1];
                } 
                
                // Get previous sample from previous zone for zones > 0
                else {
                    prev_filtered_data_p = &train_data_filtered[training_zone-1][IMU_TRAINING_SAMPLE_BUF_SIZE-1];
                }

                // Apply filter on all sensor values

                // y_t = ALPHA * x_t + (1-ALPHA) * y[t-1];
                // y_t    : filtered value
                // x_t    : raw value
                // y[t-1] : previous filtered value

                filtered_data.ax = ALPHA * current_data_p->ax + (1-ALPHA) * prev_filtered_data_p->ax;
                filtered_data.ay = ALPHA * current_data_p->ay + (1-ALPHA) * prev_filtered_data_p->ay;
                filtered_data.az = ALPHA * current_data_p->az + (1-ALPHA) * prev_filtered_data_p->az;
                filtered_data.gx = ALPHA * current_data_p->gx + (1-ALPHA) * prev_filtered_data_p->gx;
                filtered_data.gy = ALPHA * current_data_p->gy + (1-ALPHA) * prev_filtered_data_p->gy;
                filtered_data.gz = ALPHA * current_data_p->gz + (1-ALPHA) * prev_filtered_data_p->gz;

            } else {

                filtered_data.ax = ALPHA * current_data_p->ax;
                filtered_data.ay = ALPHA * current_data_p->ay;
                filtered_data.az = ALPHA * current_data_p->az;
                filtered_data.gx = ALPHA * current_data_p->gx;
                filtered_data.gy = ALPHA * current_data_p->gy;
                filtered_data.gz = ALPHA * current_data_p->gz;
                
            }

            // Store in set of filtered samples
            train_data_filtered[training_zone][i] = filtered_data;

            t++;
        }

        training_zone++;
    }
}


// Calculate the pitch angle from the sensor values
double calculate_pitch(mpu6050_data_t *data_p) {
    double pitch;

    // Formula to obtain pitch value using purely the accelerometer data
    pitch = atan2(data_p->ax, data_p->ay);

    // Return the value
    return pitch;
}


// Calculate the roll angle from the sensor values
double calculate_roll(mpu6050_data_t *data_p) {
    double roll;

    // Formula to obtain roll value using purely the accelerometer data
    roll = atan2(data_p->ay, data_p->az);

    // Return the value
    return roll;
}


// Calculate distance between two points in a 2D plane
double calculate_distance (double a1, double b1, double a2, double b2) {
    double distance = 0;
    
    // Calculate Euclidean distance
    distance = sqrt((a1 - a2)  *(a1 - a2) + (b1 - b2)  *(b1 - b2));

    // Return the distance
    return distance;
}


// Comparator function to sort by increasing order of distance 
int compare (const void *s1, const void *s2) { 
	neighbor_t *n1 = (neighbor_t *) s1;
	neighbor_t *n2 = (neighbor_t *) s2;

    return (n1->distance - n2->distance); 
}


// Count the number of each label in a set of neighbors
void count (neighbor_t *neighbors, uint8_t **counters) {
    uint8_t *counter_LL_p;
    uint8_t *counter_LR_p;
    uint8_t *counter_TL_p;
    uint8_t *counter_TR_p;
    uint8_t i;

    // Assign each counter
    counter_LL_p = counters[0];
    counter_LR_p = counters[1];
    counter_TL_p = counters[2];
    counter_TR_p = counters[3];

    // Start each counter at zero
    *counter_LL_p = 0;
    *counter_LR_p = 0;
    *counter_TL_p = 0;
    *counter_TR_p = 0;

    // Iterate through each neighbor and count
    for (i = 0; i < K_VALUE; i++) {

        if ((neighbors[i]).brush_zone == BRUSH_ZONE_LL) {
            counter_LL_p++;
        }

        if ((neighbors[i]).brush_zone == BRUSH_ZONE_LR) {
            counter_LR_p++;
        }

        if ((neighbors[i]).brush_zone == BRUSH_ZONE_TL) {
            counter_TL_p++;
        }

        if ((neighbors[i]).brush_zone == BRUSH_ZONE_TR) {
            counter_TR_p++;
        }

    }
}


// Generate a training dataset that comprises [pitch, roll, label]
void train (mpu6050_data_t **train_data) {

    mpu6050_data_t *current_sample_p;             // Current sample being iterated
    mpu6050_data_t  train_data_filtered[4][IMU_TRAINING_SAMPLE_BUF_SIZE];     // Set of filtered train data
    trained_data_t  trained_data;                 // A single trained data
    uint8_t         training_zone = 0;            // Current training zone
    uint8_t         counter = 0;                  // Counter of all data point in the training set
    uint8_t         i;


    // Filter training data
    filter_data(train_data, (mpu6050_data_t **) train_data_filtered);

    // Do for each training zone
    while (training_zone < 4) {
        
        for (i = 0; i < IMU_TRAINING_SAMPLE_BUF_SIZE; i++) {
            
            // Get current sample
            current_sample_p = &train_data[training_zone][i];

            // Create a trained data point
            trained_data.pitch = calculate_pitch(current_sample_p);
            trained_data.roll = calculate_roll(current_sample_p);
            trained_data.brush_zone = training_zone;

            // Store in trained data set
            g_trained_data[counter] = trained_data;

            counter++;
        }

        training_zone++;
    }
}


// Generates the distance of a point to all other points
void generate_neighbors (double pitch, double roll, neighbor_t *neighbors) {

    neighbor_t neighbor;                          // Holds a neighbor
    trained_data_t * trained_data_p;              // Data point
    double distance;                              // Distance between two points
    uint8_t training_size;
    uint8_t i;

    training_size = (uint8_t) IMU_TRAINING_SAMPLE_BUF_SIZE;

    // For each point in the training dataset
    for (i = 0; i < training_size  *4; i++) {

        // Get the current trained data
        trained_data_p = &g_trained_data[i];

        // Calculate distance
        distance = calculate_distance(pitch, trained_data_p->pitch, roll, trained_data_p->roll);

        // Create a neighbor
        neighbor.distance = distance;
        neighbor.brush_zone = trained_data_p->brush_zone;

        // Store to neighbor array
        neighbors[i] = neighbor;
    }
}


// Classify a sample being passed in argument
brush_zone_t classify (mpu6050_data_t *sample) {

    brush_zone_t brush_zone;                                // Brush zone of the sample to be returned
    neighbor_t neighbors[IMU_TRAINING_SAMPLE_BUF_SIZE*4];   // Set of neighbors
    uint8_t counter_LL, counter_LR, counter_TL, counter_TR; // Variables to store the count of each label
    double pitch, roll;                                     // Pitch and roll angles of data
    uint8_t *counters[4] = {                                // Array of all the counters
        &counter_LL, &counter_LR, &counter_TL, &counter_TR
    };

    // Calculate pitch and roll angle
    pitch = calculate_pitch(sample);
    roll = calculate_roll(sample);

    // Generate an array of neighbors
    generate_neighbors (pitch, roll, neighbors);

	// Sort distance array by the smallest distance
	qsort(neighbors, IMU_TRAINING_SAMPLE_BUF_SIZE*4, sizeof(neighbor_t), compare);

	// Count each label
    count(neighbors, counters);

    // Find the most frequent
    // could also have a for loop here

    if (counter_LL >= counter_LR && counter_LL >= counter_TL && counter_LL >= counter_TR ) {
        brush_zone = BRUSH_ZONE_LL;
    }

    else if (counter_LR >= counter_LL && counter_LR >= counter_TL && counter_LR >= counter_TR ) {
        brush_zone = BRUSH_ZONE_LR;
    }

    else if (counter_TL >= counter_LL && counter_TL >= counter_LR && counter_TL >= counter_TR ) {
        brush_zone = BRUSH_ZONE_TL;
    }

    else if (counter_TR >= counter_LL && counter_TR >= counter_LR && counter_TR >= counter_TL) {
        brush_zone = BRUSH_ZONE_TR;
    }

    else {
        brush_zone = BRUSH_MODE_MAX;
    }

	return brush_zone;
}