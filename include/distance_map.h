#ifndef _DISTANCE_MAP_H_
#define _DISTANCE_MAP_H_

#include <ros/ros.h>
#include <fftw3.h>
#include "../include/vector_2d.h"

namespace map
{
    class DistanceMap
    {
        public:
        /*
         * @brief the construct function
         */
        DistanceMap();

        ~DistanceMap();

        static double CalculateThreshold(uint16_t n);

        int8_t DistanceMapUpdate(uint16_t width,uint16_t height, int8_t *input_map, double *output_map);

        private:

        Vector2d *_eight_neighbors_mask;

        std::vector<Vector2d> _M1;
        std::vector<Vector2d> _M2;
        std::vector<Vector2d> _M3;
        std::vector<Vector2d> _M4;
    };
}
#endif
