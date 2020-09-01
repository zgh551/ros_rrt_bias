#include "../include/convolution.h"


int8_t Common::Convolution::fft2d( const int8_t  *input_map, 
                                   const uint16_t input_map_width, 
                                   const uint16_t input_map_height, 
                                   const uint16_t cov_map_width, 
                                   const uint16_t cov_map_height, 
                                   fftw_complex  *output_cov_fft_map)
{
    double *d_input_array;
    fftw_complex *c_output_array;
    fftw_plan fft2d_plan;

    if(input_map == nullptr)
    {
        return -1;
    }

    if((input_map_width  > cov_map_width ) ||
       (input_map_height > cov_map_height))
    {
        return -1;
    }
    // create the fft input and output space
    d_input_array  = (double*)fftw_malloc(sizeof(double) * cov_map_height * cov_map_width);
    c_output_array = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * cov_map_height * (cov_map_width / 2 + 1));

    // create the fft plan
    fft2d_plan = fftw_plan_dft_r2c_2d(cov_map_height, cov_map_width, d_input_array, c_output_array, FFTW_MEASURE);

    // the input array init
    for(uint16_t i = 0; i < cov_map_height * cov_map_width; i++)
    {
        d_input_array[i] = 0.0;
    }

    // input map update
    for(uint16_t j = 0; j < input_map_height; j++)
    {
        for(uint16_t i=0; i < input_map_width; i++)
        {
            d_input_array[i + j * cov_map_width] = input_map[i + j * input_map_width];
        }
    }

    // calculate the fft result
    fftw_execute(fft2d_plan);

    // output the fft result
    for(uint16_t i = 0; i < cov_map_height * (cov_map_width / 2 + 1); i++)
    {
        output_cov_fft_map[i][0] = c_output_array[i][0];
        output_cov_fft_map[i][1] = c_output_array[i][1];
    }

    // free the memory space
    fftw_destroy_plan(fft2d_plan);
    fftw_free(d_input_array);
    fftw_free(c_output_array);

    return 0;
}

int8_t Common::Convolution::ifft2d( const fftw_complex *input_fft_map, 
                                    const uint16_t cov_map_width, 
                                    const uint16_t cov_map_height, 
                                    const uint16_t output_map_width, 
                                    const uint16_t output_map_height,
                                    int8_t *output_map)
{
    fftw_complex *c_inverse_input_array;
    double *d_inverse_output_array;
    fftw_plan ifft2d_plan;
    uint16_t cov_map_size;
    uint16_t kernel_origin_x, kernel_origin_y;

    if(input_fft_map == nullptr)
    {
        return -1;
    }

    if((cov_map_width < output_map_width) ||
       (cov_map_height < output_map_height))
    {
        return -1;
    }

    cov_map_size = cov_map_width * cov_map_height;
    kernel_origin_x = (cov_map_width - output_map_width + 1) / 2;
    kernel_origin_y = (cov_map_height - output_map_height + 1) / 2;

    // create the input and output space
    c_inverse_input_array  = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  cov_map_height * (cov_map_width / 2 + 1));
    d_inverse_output_array = (double*)fftw_malloc(sizeof(double) * cov_map_size);

    // create the ifft plan
    ifft2d_plan = fftw_plan_dft_c2r_2d(cov_map_height, cov_map_width, c_inverse_input_array, d_inverse_output_array, FFTW_MEASURE);

    // the input array init
    for(uint16_t i=0; i < cov_map_height * (cov_map_width / 2 + 1); i++)
    {
        c_inverse_input_array[i][0] = input_fft_map[i][0];
        c_inverse_input_array[i][1] = input_fft_map[i][1];
    }

    fftw_execute(ifft2d_plan);

    /*
    for(uint16_t i=0; i < _fft_map_size; i++)
    {
        output_map[i] = _d_inverse_output_array[i] / _map_size > 1.0e-6 ? 100 : 0;
    }
    */
    for (uint16_t j = 0; j < output_map_height; j++)
    {
        for (uint16_t i = 0; i < output_map_width; i++)
        {
            output_map[i + j * output_map_width] = d_inverse_output_array[kernel_origin_x +  i + (j + kernel_origin_y) * cov_map_width] / cov_map_size > 1.0e-6 ? 100 : 0;
        }
    }

    fftw_destroy_plan(ifft2d_plan);
    fftw_free(c_inverse_input_array);
    fftw_free(d_inverse_output_array);

    return 0; 
}


int8_t Common::Convolution::convolution_2d( const int8_t *input_kernel_map,   const uint16_t input_kernel_width,   const uint16_t input_kernel_height, 
                                            const int8_t *input_obstacle_map, const uint16_t input_obstacle_width, const uint16_t input_obstacle_height, 
                                            int8_t *output_sum_map)
{
    uint16_t cov_map_width, cov_map_height;
    fftw_complex *c_disk_fft_array, *c_obstacle_fft_array, *c_cov_ifft_array;
    
    // calculate the width and height of map
    cov_map_width  = input_kernel_width  + input_obstacle_width  - 1;
    cov_map_height = input_kernel_height + input_obstacle_height - 1;

    // malloc the memory space for fft 
    c_disk_fft_array     = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  cov_map_height * (cov_map_width / 2 + 1));
    c_obstacle_fft_array = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  cov_map_height * (cov_map_width / 2 + 1));
    c_cov_ifft_array     = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) *  cov_map_height * (cov_map_width / 2 + 1));

    // the fft of disk
    fft2d(  input_kernel_map, 
            input_kernel_width, 
            input_kernel_height,
            cov_map_width, 
            cov_map_height, 
            c_disk_fft_array);

    // the fft of obstacle map
    fft2d(  input_obstacle_map, 
            input_obstacle_width, 
            input_obstacle_height,
            cov_map_width, 
            cov_map_height, 
            c_obstacle_fft_array);

    // Frequence domain complex multiplication
    for (uint16_t i=0; i < cov_map_height * (cov_map_width / 2 + 1); i++)
    {
        c_cov_ifft_array[i][0] =  c_disk_fft_array[i][0] * c_obstacle_fft_array[i][0] - c_disk_fft_array[i][1] * c_obstacle_fft_array[i][1]; 
        c_cov_ifft_array[i][1] =  c_disk_fft_array[i][0] * c_obstacle_fft_array[i][1] + c_disk_fft_array[i][1] * c_obstacle_fft_array[i][0]; 
    }

    // the ifft for convolution result
    ifft2d(c_cov_ifft_array, cov_map_width, cov_map_height, input_obstacle_width, input_obstacle_height, output_sum_map);

    fftw_free(c_disk_fft_array);
    fftw_free(c_obstacle_fft_array);
    fftw_free(c_cov_ifft_array);
    return 0;
}

