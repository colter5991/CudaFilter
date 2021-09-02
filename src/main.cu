#include <stdio.h>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb_image.h"
#include "stb_image_write.h"

#define NUM_CHANNELS 3
#define WINDOW_SIZE 3

__device__ int index(int x, int y, int width, int height) {
    return (y * NUM_CHANNELS * width) + (x * NUM_CHANNELS);
}

__device__ int square(int a) {return a * a;}

struct Quadrant {
    int x_start;
    int x_end;
    int y_start;
    int y_end;
};;

__global__ void oilpaint(const uint8_t* input, uint8_t* output, int width, int height) {
    // Iterate through image
    for (int y = blockIdx.y * blockDim.y + threadIdx.y; y < height; y += blockDim.y * gridDim.y) { 
    for (int x = blockIdx.x * blockDim.x + threadIdx.x; x < width;  x += blockDim.x * gridDim.x) {

        // Define 4 overlapping quadrants around the center pixel
        Quadrant quadrants[4] = {
            {max(x - WINDOW_SIZE, 0), x,                               max(y - WINDOW_SIZE, 0), y},
            {max(x - WINDOW_SIZE, 0), x,                               y,                       min(height - 1, y + WINDOW_SIZE)},
            {x,                       min(width - 1, x + WINDOW_SIZE), max(y - WINDOW_SIZE, 0), y},
            {x,                       min(width - 1, x + WINDOW_SIZE), y,                       min(height - 1, y + WINDOW_SIZE)},
        };

        // Calculate mean variance and intensity for each quadrant of the image
        int min_variance = 0x7FFFFFFF;
        uint8_t min_red = 0;
        uint8_t min_green = 0;
        uint8_t min_blue = 0;

        // Iterate through the 4 quadrants
        for (int i = 0; i < 4; i++) {
            Quadrant* quadrant = &quadrants[i];
            int red_sum = 0;
            int green_sum = 0;
            int blue_sum = 0;

            // First, get the mean brightness
            int brightness = 0;
            for (int y_quad = quadrant->y_start; y_quad <= quadrant->y_end; y_quad++) {
            for (int x_quad = quadrant->x_start; x_quad <= quadrant->x_end; x_quad++) {
                int red = input[index(x_quad, y_quad, width, height)];
                int green = input[index(x_quad, y_quad, width, height) + 1];
                int blue = input[index(x_quad, y_quad, width, height) + 2];
                red_sum += red;
                green_sum += green;
                blue_sum += blue;

                brightness += max(max(red, green), blue);
            }}
            int mean_brightness = brightness / square(WINDOW_SIZE + 1);


            // Next get the variance
            int variance = 0;
            for (int y_quad = quadrant->y_start; y_quad <= quadrant->y_end; y_quad++) {
            for (int x_quad = quadrant->x_start; x_quad <= quadrant->x_end; x_quad++) {
                int red = input[index(x_quad, y_quad, width, height)];
                int green = input[index(x_quad, y_quad, width, height) + 1];
                int blue = input[index(x_quad, y_quad, width, height) + 2];
                variance += square(max(max(red, green), blue) - mean_brightness);
            }}

            // Update the color if this variance is lower
            if (variance < min_variance) {
                variance = min_variance;
                min_red = (red_sum / square(WINDOW_SIZE + 1));
                min_green = (green_sum / square(WINDOW_SIZE + 1));
                min_blue = (blue_sum / square(WINDOW_SIZE + 1));
            }
        }

        // Write output
        output[index(x, y, width, height) + 0] = min_red;
        output[index(x, y, width, height) + 1] = min_green;
        output[index(x, y, width, height) + 2] = min_blue;
    }}
}

int main(int argc, char** argv) {
    if (argc != 3) {
        printf("usage: %s <input picture> <output picture>\n", argv[0]);
        return 1;
    }

    // Load input image
    int width;
    int height;
    int channels;
    const uint8_t* input_image = (const uint8_t*)stbi_load(argv[1], &width, &height, &channels, NUM_CHANNELS);
    if (input_image == NULL) {
        printf("Could not load image \"%s\"\n", argv[1]);
    }

    // Allocate input and output buffers
    uint8_t* d_input_image;
    cudaError_t error;
    error = cudaMalloc(&d_input_image, width * height * NUM_CHANNELS);
    if (error != cudaSuccess) {
        printf("Failed to allocate gpu buffer: %s\n", cudaGetErrorString(error));
        return 1;
    }
    uint8_t* d_output_image;
    error = cudaMalloc(&d_output_image, width * height * NUM_CHANNELS);
    if (error != cudaSuccess) {
        printf("Failed to allocate gpu buffer: %s\n", cudaGetErrorString(error));
        return 1;
    }

    // Copy input buffer to gpu
    error = cudaMemcpy(d_input_image, input_image, width * height * NUM_CHANNELS, cudaMemcpyHostToDevice);
    if (error != cudaSuccess) {
        printf("Failed to copy memory from host to device: %s\n", cudaGetErrorString(error));
        return 1;
    }

    // Call gpu kernel
    dim3 grid(32,32,1);
    dim3 block(8,8,1);
    oilpaint<<<grid, block>>>(d_input_image, d_output_image, width, height);


    // Copy output memory to local buffer
    uint8_t* output_image = (uint8_t*)malloc(width * height * NUM_CHANNELS);
    error = cudaMemcpy(output_image, d_output_image, width * height * NUM_CHANNELS, cudaMemcpyDeviceToHost);
    if (error != cudaSuccess) {
        printf("Failed to copy memory from device to host: %s\n", cudaGetErrorString(error));
        return 1;
    }

    // Write output to image file
    int stbi_error = stbi_write_bmp(argv[2], width, height, NUM_CHANNELS, output_image);
    if (stbi_error == 0) {
        printf("Failed to write to output image \"%s\"\n", argv[2]);
        return 1;
    }

    return 0;

}

