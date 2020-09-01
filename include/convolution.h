#ifndef _CONVOLUTION_H_
#define _CONVOLUTION_H_

#include <fftw3.h>
#include <ros/ros.h>

namespace Common
{
    class Convolution
    {
        public:

        /*
         * @brief construct
         */
        Convolution() = default;

        /*
         * @brief construct
         */
        ~Convolution() = default;

        /*
         * @brief the fft base on the fftw3 lib
         * @param input_map: the fft source input map
         * @param input_map_width : the input map width
         * @param input_map_height: the input map height
         * @param cov_output_map_width : the width of convolution map
         * @param cov_output_map_height: the heigth of convolution map
         * @param output_cov_fft_map: output the fft map
         * @retun the status of fft 
         */
         static int8_t fft2d( const int8_t  *input_map, 
                              const uint16_t input_map_width, 
                              const uint16_t input_map_height, 
                              const uint16_t cov_map_width, 
                              const uint16_t cov_map_height, 
                              fftw_complex  *output_cov_fft_map);

        /*
         * @brief the ifft base on the fftw3 lib
         * @param input_fft_map: the ifft map input
         * @param cov_map_width: the width of the convolution map
         * @param cov_map_height: the height of the convolution map
         * @param output_map_width: the width of the output map
         * @param output_map_height: the height of the output map
         * @param output_map: the map output
         * @return the status of ifft
         */
        static int8_t ifft2d( const fftw_complex *input_fft_map, 
                              const uint16_t cov_map_width, 
                              const uint16_t cov_map_height, 
                              const uint16_t output_map_width, 
                              const uint16_t output_map_height,
                              int8_t *output_map); 

        /*
         * @brief the convolution of two picture
         * @param input_kernel_map: the point of kernel map
         * @param input_kernel_width: the width of the kernel map
         * @param input_kernel_height: the height of the kernel map
         * @param input_obstacle_map: the point of obstacle map
         * @param input_obstacle_width: the wdth of the obstacle map
         * @param input_obstacle_height: the height of the obstacle map
         * @return the status of convolution
         */
        static int8_t convolution_2d( const int8_t  *input_kernel_map,   
                                      const uint16_t input_kernel_width,   
                                      const uint16_t input_kernel_height, 
                                      const int8_t  *input_obstacle_map, 
                                      const uint16_t input_obstacle_width, 
                                      const uint16_t input_obstacle_height, 
                                      int8_t *output_sum_map);

    };
}
#endif

