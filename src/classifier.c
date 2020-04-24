#include "classifier.h"


/*
 *******************************************************************************
  *                             Global Variables                               *
 *******************************************************************************
*/


// Trained data
static trained_data_t g_trained_data[IMU_TRAINING_SAMPLE_BUF_SIZE * 4];


// Neighbours (for classification)
static neighbor_t g_neighbors[IMU_TRAINING_SAMPLE_BUF_SIZE * 4];


/*
 *******************************************************************************
  *                       External Function Definitions                        *
 *******************************************************************************
*/


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

    return abs(n1->distance - n2->distance); 
}


// Realtime variant of train
void train_rt (mpu6050_data_t *data_p, brush_zone_t zone, off_t n) {
    static mpu6050_data_t data_filtered_last;

    // The absolute sample index
    off_t k = zone * IMU_TRAINING_SAMPLE_BUF_SIZE + n;

    // If the first sample - zero out filtered_last fields
    if (k == 0) {
        data_filtered_last = (mpu6050_data_t){0};
    }
    
    // Compute the filtered sample :: y_t = ALPHA * x_t + (1-ALPHA) * y[t-1];
    mpu6050_data_t data_filtered = (mpu6050_data_t) {
        .ax = ALPHA * data_p->ax + (1-ALPHA) * data_filtered_last.ax,
        .ay = ALPHA * data_p->ay + (1-ALPHA) * data_filtered_last.ay,
        .az = ALPHA * data_p->az + (1-ALPHA) * data_filtered_last.az,
        .gx = ALPHA * data_p->gx + (1-ALPHA) * data_filtered_last.gx,
        .gy = ALPHA * data_p->gy + (1-ALPHA) * data_filtered_last.gy,
        .gz = ALPHA * data_p->gz + (1-ALPHA) * data_filtered_last.gz
    };

    // Update filtered_last
    data_filtered_last = data_filtered;

    // Create a new trained data point
    trained_data_t trained_data = (trained_data_t) {
        .pitch      = calculate_pitch(&data_filtered),
        .roll       = calculate_roll(&data_filtered),
        .brush_zone = zone
    };

    // Store the trained data point
    g_trained_data[k] = trained_data; 
}


brush_zone_t classify_rt (mpu6050_data_t *data_p) {
    double pitch, roll;
    uint8_t zone_count[4] = {0};

    // Compute pitch and roll. 
    pitch = calculate_pitch(data_p); roll = calculate_roll(data_p);

    // Compute neighbours
    for (int i = 0; i < (4 * IMU_TRAINING_SAMPLE_BUF_SIZE); ++i) {

        // Get the corresponding trained sample
        trained_data_t *t = g_trained_data + i;

        // Compute the euclidean distance
        g_neighbors[i] = (neighbor_t) {
            .distance = calculate_distance(pitch, t->pitch, roll, t->roll),
            .brush_zone = t->brush_zone
        };
    }

    // Sort the neighbours by smallest distance
    qsort(g_neighbors, IMU_TRAINING_SAMPLE_BUF_SIZE * 4, sizeof(neighbor_t),
        compare);

    // Count number of classes appearing in first K 
    for (int i = 0; i < K_VALUE; ++i) {
        zone_count[g_neighbors[i].brush_zone]++;
    }

    // Return majority
    if (zone_count[0] >= zone_count[1]) {
        if (zone_count[2] >= zone_count[3]) {
            return zone_count[0] >= zone_count[2] ? 0 : 2;
        } else {
            return zone_count[0] >= zone_count[3] ? 0 : 3;
        }
    } else {
        if (zone_count[2] >= zone_count[3]) {
            return zone_count[1] >= zone_count[2] ? 1 : 2;
        } else {
            return zone_count[1] >= zone_count[3] ? 1 : 3;
        }     
    }
}