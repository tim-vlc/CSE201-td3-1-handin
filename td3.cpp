#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
  double* new_array = new double[new_size];
  for (int i = 0; i < new_size; i++){
      if (i < length){
          new_array[i] = array[i];
      }
      else {
          new_array[i] = 0;
      }
  }
  delete[] array;
  return new_array;
}

double* shrink_array(double* array, int length, int new_size) {
    double* new_array = new double[new_size];
    for (int i = 0; i < new_size; i++){
            new_array[i] = array[i];
    }
    delete[] array;
    return new_array;
  }

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
  if (current_size == max_size){
      array = extend_array(array, max_size, max_size + 5);
      max_size += 5;
  }
  array[current_size] = element;
  current_size += 1;
  return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    current_size -= 1;
    if (max_size - current_size >= 5){
        array = shrink_array(array, max_size, max_size - 5);
        max_size -= 5;
    }
    return array;
  }

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {
  // YOU CAN MODIFY THIS FUNCTION TO RECORD THE TELEMETRY

  bool hit_target, hit_obstacle;
  double v0_x, v0_y, x, y, t;
  double PI = 3.14159265;
  double g = 9.8;

  v0_x = magnitude * cos(angle * PI / 180);
  v0_y = magnitude * sin(angle * PI / 180);

  t = 0;
  x = 0;
  y = 0;

  hit_target = false;
  hit_obstacle = false;
  while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
    double * target_coordinates = find_collision(x, y, targets, tot_targets);
    if (target_coordinates != NULL) {
      remove_target(targets, tot_targets, target_coordinates);
      hit_target = true;
    } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
      hit_obstacle = true;
    } else {
      t = t + simulation_interval;
      y = v0_y * t  - 0.5 * g * t * t;
      x = v0_x * t;
    }
    telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
    telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);
  }

  return hit_target;
}

void swap(double *xp, double *yp)
{
    double temp = *xp;
    *xp = *yp;
    *yp = temp;
}

void sort(double* global_telemetry, const int global_telemetry_current_size)
{
   int i, j;
   bool swapped;
   for (i = 0; i < global_telemetry_current_size; i = i + 3)
   {
     swapped = false;
     for (j = 0; j < (global_telemetry_current_size-i-3); j = j + 3)
     {
        if (global_telemetry[j] > global_telemetry[j+3])
        {
           swap(&global_telemetry[j], &global_telemetry[j+3]);
           swap(&global_telemetry[j+1], &global_telemetry[j+4]);
           swap(&global_telemetry[j+2], &global_telemetry[j+5]);
           swapped = true;
        }
     }
     if (swapped == false)
        break;
   }
}

void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
    for (int i = 0; i < tot_telemetries; i++){
        for (int j = 0; j < telemetries_sizes[i]; j++){
            global_telemetry = append_to_array(telemetries[i][j], global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
        }
    }
    if (global_telemetry_current_size != 0) {
        sort(global_telemetry, global_telemetry_current_size);
    }
}
