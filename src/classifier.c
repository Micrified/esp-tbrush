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

    // qsort comparison rules
    // (1) A negative value if s1 < s2
    // (2) Zero if s1 == s2
    // (3) A positive value if s1 > s2

    if (n2->distance > n1->distance) {
        return -1;
    }

    if (n2->distance == n1->distance) {
        return 0;
    }

    return 1;
}


// Applies a filter to the given value. 
void filter (mpu6050_data_t *data_p, mpu6050_data_t *last_p) {

    // Update data_p with last_p
    data_p->ax = ALPHA * data_p->ax + (1-ALPHA) * last_p->ax;
    data_p->ay = ALPHA * data_p->ay + (1-ALPHA) * last_p->ay;
    data_p->az = ALPHA * data_p->az + (1-ALPHA) * last_p->az;
    data_p->gx = ALPHA * data_p->gx + (1-ALPHA) * last_p->gx;
    data_p->gy = ALPHA * data_p->gy + (1-ALPHA) * last_p->gy;
    data_p->gz = ALPHA * data_p->gz + (1-ALPHA) * last_p->gz;

    // Update last with latest
    *last_p = *data_p;
}

void display_training_data () {
    for (int i = 0; i < (IMU_TRAINING_SAMPLE_BUF_SIZE * 4); ++i) {
        printf("[%d] = {%f, %f, %d}\n", i, g_trained_data[i].pitch,
            g_trained_data[i].roll, g_trained_data[i].brush_zone);
    }
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

    // Apply the filter, and update the last
    filter(data_p, &data_filtered_last);

    // Create a new trained data point
    trained_data_t trained_data = (trained_data_t) {
        .pitch      = calculate_pitch(&data_filtered_last),
        .roll       = calculate_roll(&data_filtered_last),
        .brush_zone = zone
    };

    // Store the trained data point
    g_trained_data[k] = trained_data; 
}

brush_zone_t classify_rt (mpu6050_data_t *data_p) {
    double pitch, roll;
    uint8_t zone_count[4] = {0};

    // Compute pitch and roll
    pitch = calculate_pitch(data_p); roll = calculate_roll(data_p);

    // Compute neighbours
    for (int i = 0; i < (4 * IMU_TRAINING_SAMPLE_BUF_SIZE); ++i) {

        // Get the corresponding trained sample
        trained_data_t *t = g_trained_data + i;

        // Compute the euclidean distance
        g_neighbors[i] = (neighbor_t) {
            .distance = calculate_distance(pitch, roll, t->pitch, t->roll),
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

brush_zone_t classify_rt_2 (double pitch, double roll) {
    uint8_t zone_count[4] = {0};

    // Compute neighbours
    for (int i = 0; i < (4 * IMU_TRAINING_SAMPLE_BUF_SIZE); ++i) {

        // Get the corresponding trained sample
        trained_data_t *t = g_trained_data + i;

        // Compute the euclidean distance
        g_neighbors[i] = (neighbor_t) {
            .distance = calculate_distance(pitch, roll, t->pitch, t->roll),
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