/**
 * @file cuda_accelerator.cu
 * @brief CUDA加速关键算子核函数实现
 * @author OpenVisionFlow Team
 * @version 0.1.0
 * 
 * 本文件包含真实的CUDA核函数实现。
 * 当CUDA可用时，会覆盖.cpp中的CPU fallback实现。
 */

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>
#include <cmath>

// 错误检查宏
#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            printf("CUDA Error at %s:%d - %s\n", __FILE__, __LINE__, \
                   cudaGetErrorString(err)); \
            return false; \
        } \
    } while(0)

// ========== 1. 高斯模糊 CUDA核函数 ==========

// 高斯模糊水平方向核函数
__global__ void gaussian_blur_horizontal_kernel(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    const float* kernel, int ksize) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    int half = ksize / 2;
    float sum = 0.0f;
    
    for (int k = 0; k < ksize; ++k) {
        int sx = x + k - half;
        // 钳制到边界
        sx = max(0, min(sx, width - 1));
        sum += src[y * width + sx] * kernel[k];
    }
    
    dst[y * width + x] = (uint8_t)(sum > 255 ? 255 : (sum < 0 ? 0 : sum));
}

// 高斯模糊垂直方向核函数
__global__ void gaussian_blur_vertical_kernel(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    const float* kernel, int ksize) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    int half = ksize / 2;
    float sum = 0.0f;
    
    for (int k = 0; k < ksize; ++k) {
        int sy = y + k - half;
        // 钳制到边界
        sy = max(0, min(sy, height - 1));
        sum += src[sy * width + x] * kernel[k];
    }
    
    dst[y * width + x] = (uint8_t)(sum > 255 ? 255 : (sum < 0 ? 0 : sum));
}

// ========== 2. Sobel边缘检测 CUDA核函数 ==========

__global__ void sobel_filter_kernel(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    bool compute_dx, bool compute_dy, int threshold) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x + 1;  // 避开边界
    int y = blockIdx.y * blockDim.y + threadIdx.y + 1;
    
    if (x >= width - 1 || y >= height - 1) return;
    
    int gx = 0, gy = 0;
    
    if (compute_dx) {
        // Sobel X方向梯度
        gx = -src[(y - 1) * width + (x - 1)] + src[(y - 1) * width + (x + 1)]
             - 2 * src[y * width + (x - 1)] + 2 * src[y * width + (x + 1)]
             - src[(y + 1) * width + (x - 1)] + src[(y + 1) * width + (x + 1)];
    }
    
    if (compute_dy) {
        // Sobel Y方向梯度
        gy = -src[(y - 1) * width + (x - 1)] - 2 * src[(y - 1) * width + x] - src[(y - 1) * width + (x + 1)]
             + src[(y + 1) * width + (x - 1)] + 2 * src[(y + 1) * width + x] + src[(y + 1) * width + (x + 1)];
    }
    
    // 计算梯度幅值
    int magnitude = (int)sqrtf((float)(gx * gx + gy * gy));
    
    // 阈值化
    if (threshold > 0) {
        dst[y * width + x] = (magnitude > threshold) ? 255 : 0;
    } else {
        dst[y * width + x] = (uint8_t)(magnitude > 255 ? 255 : magnitude);
    }
}

// ========== 3. 阈值分割 CUDA核函数 ==========

__global__ void threshold_binary_kernel(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    uint8_t thresh, uint8_t max_val) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    uint8_t val = src[y * width + x];
    dst[y * width + x] = (val > thresh) ? max_val : 0;
}

__global__ void threshold_binary_inv_kernel(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    uint8_t thresh, uint8_t max_val) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    uint8_t val = src[y * width + x];
    dst[y * width + x] = (val > thresh) ? 0 : max_val;
}

// ========== 4. 形态学处理 CUDA核函数 ==========

__global__ void erode_kernel(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    const uint8_t* kernel, int ksize) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    int half = ksize / 2;
    uint8_t min_val = 255;
    
    for (int ky = 0; ky < ksize; ++ky) {
        for (int kx = 0; kx < ksize; ++kx) {
            if (kernel[ky * ksize + kx] > 0) {
                int sy = y + ky - half;
                int sx = x + kx - half;
                
                if (sy >= 0 && sy < height && sx >= 0 && sx < width) {
                    min_val = min(min_val, src[sy * width + sx]);
                }
            }
        }
    }
    
    dst[y * width + x] = min_val;
}

__global__ void dilate_kernel(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    const uint8_t* kernel, int ksize) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    int half = ksize / 2;
    uint8_t max_val = 0;
    
    for (int ky = 0; ky < ksize; ++ky) {
        for (int kx = 0; kx < ksize; ++kx) {
            if (kernel[ky * ksize + kx] > 0) {
                int sy = y + ky - half;
                int sx = x + kx - half;
                
                if (sy >= 0 && sy < height && sx >= 0 && sx < width) {
                    max_val = max(max_val, src[sy * width + sx]);
                }
            }
        }
    }
    
    dst[y * width + x] = max_val;
}

// ========== 5. 直方图计算 CUDA核函数 ==========

// 使用原子操作计算直方图
__global__ void histogram_kernel(
    const uint8_t* src, int* hist,
    int width, int height) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    uint8_t val = src[y * width + x];
    atomicAdd(&hist[val], 1);
}

// ========== 6. 模板匹配 CUDA核函数 ==========

__global__ void template_match_sad_kernel(
    const uint8_t* src, float* scores,
    int src_w, int src_h,
    int tmpl_w, int tmpl_h) {
    
    // 这个核函数需要src包含源图像和模板图像的合并数据
    // 简化实现：这里假设模板数据已经在常量内存或全局内存中
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    int search_w = src_w - tmpl_w + 1;
    int search_h = src_h - tmpl_h + 1;
    
    if (x >= search_w || y >= search_h) return;
    
    // 注意：完整实现需要将模板数据传入
    // 这里仅作为框架示例
    scores[y * search_w + x] = 0.0f;
}

// ========== 7. Blob分析 CUDA核函数 ==========

// 使用原子操作的连通区域标记（简化版）
__global__ void blob_label_kernel(
    const uint8_t* src, int* labels,
    int width, int height) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= width || y >= height) return;
    
    if (src[y * width + x] > 128) {
        labels[y * width + x] = 1;  // 前景标记
    } else {
        labels[y * width + x] = 0;  // 背景
    }
}

// ========== 8. 图像缩放 CUDA核函数 ==========

__global__ void resize_bilinear_kernel(
    const uint8_t* src, uint8_t* dst,
    int src_w, int src_h,
    int dst_w, int dst_h) {
    
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (x >= dst_w || y >= dst_h) return;
    
    float scale_x = (float)src_w / dst_w;
    float scale_y = (float)src_h / dst_h;
    
    float src_x = x * scale_x;
    float src_y = y * scale_y;
    
    int x0 = (int)src_x;
    int y0 = (int)src_y;
    int x1 = min(x0 + 1, src_w - 1);
    int y1 = min(y0 + 1, src_h - 1);
    
    float fx = src_x - x0;
    float fy = src_y - y0;
    
    float v00 = src[y0 * src_w + x0];
    float v01 = src[y0 * src_w + x1];
    float v10 = src[y1 * src_w + x0];
    float v11 = src[y1 * src_w + x1];
    
    float value = v00 * (1 - fx) * (1 - fy) +
                  v01 * fx * (1 - fy) +
                  v10 * (1 - fx) * fy +
                  v11 * fx * fy;
    
    dst[y * dst_w + x] = (uint8_t)(value > 255 ? 255 : (value < 0 ? 0 : value));
}

// ========== CUDA辅助函数 ==========

// 计算高斯核
void compute_gaussian_kernel_cuda(float* kernel, float sigma, int ksize) {
    int half = ksize / 2;
    float sum = 0.0f;
    
    for (int i = 0; i < ksize; ++i) {
        float x = i - half;
        kernel[i] = expf(-(x * x) / (2 * sigma * sigma));
        sum += kernel[i];
    }
    
    // 归一化
    for (int i = 0; i < ksize; ++i) {
        kernel[i] /= sum;
    }
}

// 创建结构元素
void create_morph_kernel_cuda(uint8_t* kernel, int ksize, int shape) {
    int cx = ksize / 2;
    int cy = ksize / 2;
    
    // 0: Rect, 1: Ellipse, 2: Cross
    if (shape == 0) {
        // Rect
        for (int i = 0; i < ksize * ksize; ++i) {
            kernel[i] = 255;
        }
    } else if (shape == 1) {
        // Ellipse
        for (int y = 0; y < ksize; ++y) {
            for (int x = 0; x < ksize; ++x) {
                float dx = x - cx;
                float dy = y - cy;
                float rx = ksize / 2.0f;
                float ry = ksize / 2.0f;
                if ((dx * dx) / (rx * rx) + (dy * dy) / (ry * ry) <= 1.0f) {
                    kernel[y * ksize + x] = 255;
                } else {
                    kernel[y * ksize + x] = 0;
                }
            }
        }
    } else if (shape == 2) {
        // Cross
        for (int y = 0; y < ksize; ++y) {
            for (int x = 0; x < ksize; ++x) {
                if (x == cx || y == cy) {
                    kernel[y * ksize + x] = 255;
                } else {
                    kernel[y * ksize + x] = 0;
                }
            }
        }
    }
}

// ========== CUDA检测与初始化 ==========

extern "C" {

// 检测CUDA是否可用
bool cuda_is_available() {
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    
    if (err != cudaSuccess || device_count == 0) {
        return false;
    }
    
    // 检查第一个设备是否可用
    cudaDeviceProp prop;
    err = cudaGetDeviceProperties(&prop, 0);
    
    if (err != cudaSuccess) {
        return false;
    }
    
    return true;
}

// 获取GPU信息
bool cuda_get_gpu_info(char* name, size_t* total_memory, char* compute_capability) {
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    
    if (err != cudaSuccess || device_count == 0) {
        return false;
    }
    
    cudaDeviceProp prop;
    err = cudaGetDeviceProperties(&prop, 0);
    
    if (err != cudaSuccess) {
        return false;
    }
    
    strcpy(name, prop.name);
    *total_memory = prop.totalGlobalMem / (1024 * 1024);  // MB
    
    sprintf(compute_capability, "%d.%d", prop.major, prop.minor);
    
    return true;
}

// ========== 高斯模糊GPU实现 ==========

int cuda_gaussian_blur(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    float sigma, int ksize,
    float* gpu_time_ms) {
    
    // 确定核大小
    if (ksize <= 0) {
        ksize = (int)(ceil(6 * sigma)) | 1;  // 确保奇数
    }
    
    // 分配GPU内存
    uint8_t* d_src = nullptr;
    uint8_t* d_temp = nullptr;
    uint8_t* d_dst = nullptr;
    float* d_kernel = nullptr;
    
    size_t img_size = width * height;
    
    CUDA_CHECK(cudaMalloc(&d_src, img_size));
    CUDA_CHECK(cudaMalloc(&d_temp, img_size));
    CUDA_CHECK(cudaMalloc(&d_dst, img_size));
    CUDA_CHECK(cudaMalloc(&d_kernel, ksize));
    
    // 计算高斯核
    float* h_kernel = new float[ksize];
    compute_gaussian_kernel_cuda(h_kernel, sigma, ksize);
    
    // 复制数据到GPU
    CUDA_CHECK(cudaMemcpy(d_src, src, img_size, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_kernel, h_kernel, ksize * sizeof(float), cudaMemcpyHostToDevice));
    
    // 设置核函数参数
    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
    
    // 创建CUDA事件用于计时
    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));
    
    CUDA_CHECK(cudaEventRecord(start));
    
    // 执行水平方向滤波
    gaussian_blur_horizontal_kernel<<<grid, block>>>(d_src, d_temp, width, height, d_kernel, ksize);
    
    // 执行垂直方向滤波
    gaussian_blur_vertical_kernel<<<grid, block>>>(d_temp, d_dst, width, height, d_kernel, ksize);
    
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    
    // 计算执行时间
    float elapsed_ms = 0;
    CUDA_CHECK(cudaEventElapsedTime(&elapsed_ms, start, stop));
    *gpu_time_ms = elapsed_ms;
    
    // 复制结果回主机
    CUDA_CHECK(cudaMemcpy(dst, d_dst, img_size, cudaMemcpyDeviceToHost));
    
    // 清理
    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));
    CUDA_CHECK(cudaFree(d_src));
    CUDA_CHECK(cudaFree(d_temp));
    CUDA_CHECK(cudaFree(d_dst));
    CUDA_CHECK(cudaFree(d_kernel));
    delete[] h_kernel;
    
    return 0;  // Success
}

// ========== Sobel边缘检测GPU实现 ==========

int cuda_sobel_filter(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    bool compute_dx, bool compute_dy, int threshold,
    float* gpu_time_ms) {
    
    size_t img_size = width * height;
    
    uint8_t* d_src = nullptr;
    uint8_t* d_dst = nullptr;
    
    CUDA_CHECK(cudaMalloc(&d_src, img_size));
    CUDA_CHECK(cudaMalloc(&d_dst, img_size));
    
    CUDA_CHECK(cudaMemcpy(d_src, src, img_size, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemset(d_dst, 0, img_size));  // 边界初始化为0
    
    dim3 block(16, 16);
    dim3 grid((width - 2 + block.x - 1) / block.x, (height - 2 + block.y - 1) / block.y);
    
    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));
    
    CUDA_CHECK(cudaEventRecord(start));
    
    sobel_filter_kernel<<<grid, block>>>(d_src, d_dst, width, height, compute_dx, compute_dy, threshold);
    
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    
    float elapsed_ms = 0;
    CUDA_CHECK(cudaEventElapsedTime(&elapsed_ms, start, stop));
    *gpu_time_ms = elapsed_ms;
    
    CUDA_CHECK(cudaMemcpy(dst, d_dst, img_size, cudaMemcpyDeviceToHost));
    
    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));
    CUDA_CHECK(cudaFree(d_src));
    CUDA_CHECK(cudaFree(d_dst));
    
    return 0;
}

// ========== 阈值分割GPU实现 ==========

int cuda_threshold_binary(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    uint8_t thresh, uint8_t max_val,
    float* gpu_time_ms) {
    
    size_t img_size = width * height;
    
    uint8_t* d_src = nullptr;
    uint8_t* d_dst = nullptr;
    
    CUDA_CHECK(cudaMalloc(&d_src, img_size));
    CUDA_CHECK(cudaMalloc(&d_dst, img_size));
    
    CUDA_CHECK(cudaMemcpy(d_src, src, img_size, cudaMemcpyHostToDevice));
    
    dim3 block(32, 32);
    dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
    
    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));
    
    CUDA_CHECK(cudaEventRecord(start));
    
    threshold_binary_kernel<<<grid, block>>>(d_src, d_dst, width, height, thresh, max_val);
    
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    
    float elapsed_ms = 0;
    CUDA_CHECK(cudaEventElapsedTime(&elapsed_ms, start, stop));
    *gpu_time_ms = elapsed_ms;
    
    CUDA_CHECK(cudaMemcpy(dst, d_dst, img_size, cudaMemcpyDeviceToHost));
    
    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));
    CUDA_CHECK(cudaFree(d_src));
    CUDA_CHECK(cudaFree(d_dst));
    
    return 0;
}

// ========== 形态学处理GPU实现 ==========

int cuda_erode(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    const uint8_t* kernel, int ksize,
    float* gpu_time_ms) {
    
    size_t img_size = width * height;
    size_t kernel_size = ksize * ksize;
    
    uint8_t* d_src = nullptr;
    uint8_t* d_dst = nullptr;
    uint8_t* d_kernel = nullptr;
    
    CUDA_CHECK(cudaMalloc(&d_src, img_size));
    CUDA_CHECK(cudaMalloc(&d_dst, img_size));
    CUDA_CHECK(cudaMalloc(&d_kernel, kernel_size));
    
    CUDA_CHECK(cudaMemcpy(d_src, src, img_size, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_kernel, kernel, kernel_size, cudaMemcpyHostToDevice));
    
    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
    
    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));
    
    CUDA_CHECK(cudaEventRecord(start));
    
    erode_kernel<<<grid, block>>>(d_src, d_dst, width, height, d_kernel, ksize);
    
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    
    float elapsed_ms = 0;
    CUDA_CHECK(cudaEventElapsedTime(&elapsed_ms, start, stop));
    *gpu_time_ms = elapsed_ms;
    
    CUDA_CHECK(cudaMemcpy(dst, d_dst, img_size, cudaMemcpyDeviceToHost));
    
    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));
    CUDA_CHECK(cudaFree(d_src));
    CUDA_CHECK(cudaFree(d_dst));
    CUDA_CHECK(cudaFree(d_kernel));
    
    return 0;
}

int cuda_dilate(
    const uint8_t* src, uint8_t* dst,
    int width, int height,
    const uint8_t* kernel, int ksize,
    float* gpu_time_ms) {
    
    size_t img_size = width * height;
    size_t kernel_size = ksize * ksize;
    
    uint8_t* d_src = nullptr;
    uint8_t* d_dst = nullptr;
    uint8_t* d_kernel = nullptr;
    
    CUDA_CHECK(cudaMalloc(&d_src, img_size));
    CUDA_CHECK(cudaMalloc(&d_dst, img_size));
    CUDA_CHECK(cudaMalloc(&d_kernel, kernel_size));
    
    CUDA_CHECK(cudaMemcpy(d_src, src, img_size, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(d_kernel, kernel, kernel_size, cudaMemcpyHostToDevice));
    
    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
    
    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));
    
    CUDA_CHECK(cudaEventRecord(start));
    
    dilate_kernel<<<grid, block>>>(d_src, d_dst, width, height, d_kernel, ksize);
    
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    
    float elapsed_ms = 0;
    CUDA_CHECK(cudaEventElapsedTime(&elapsed_ms, start, stop));
    *gpu_time_ms = elapsed_ms;
    
    CUDA_CHECK(cudaMemcpy(dst, d_dst, img_size, cudaMemcpyDeviceToHost));
    
    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));
    CUDA_CHECK(cudaFree(d_src));
    CUDA_CHECK(cudaFree(d_dst));
    CUDA_CHECK(cudaFree(d_kernel));
    
    return 0;
}

// ========== 直方图计算GPU实现 ==========

int cuda_histogram(
    const uint8_t* src, int* hist,
    int width, int height,
    float* gpu_time_ms) {
    
    size_t img_size = width * height;
    size_t hist_size = 256 * sizeof(int);
    
    uint8_t* d_src = nullptr;
    int* d_hist = nullptr;
    
    CUDA_CHECK(cudaMalloc(&d_src, img_size));
    CUDA_CHECK(cudaMalloc(&d_hist, hist_size));
    
    CUDA_CHECK(cudaMemcpy(d_src, src, img_size, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemset(d_hist, 0, hist_size));
    
    dim3 block(32, 32);
    dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
    
    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));
    
    CUDA_CHECK(cudaEventRecord(start));
    
    histogram_kernel<<<grid, block>>>(d_src, d_hist, width, height);
    
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    
    float elapsed_ms = 0;
    CUDA_CHECK(cudaEventElapsedTime(&elapsed_ms, start, stop));
    *gpu_time_ms = elapsed_ms;
    
    CUDA_CHECK(cudaMemcpy(hist, d_hist, hist_size, cudaMemcpyDeviceToHost));
    
    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));
    CUDA_CHECK(cudaFree(d_src));
    CUDA_CHECK(cudaFree(d_hist));
    
    return 0;
}

// ========== 图像缩放GPU实现 ==========

int cuda_resize_bilinear(
    const uint8_t* src, uint8_t* dst,
    int src_w, int src_h,
    int dst_w, int dst_h,
    float* gpu_time_ms) {
    
    size_t src_size = src_w * src_h;
    size_t dst_size = dst_w * dst_h;
    
    uint8_t* d_src = nullptr;
    uint8_t* d_dst = nullptr;
    
    CUDA_CHECK(cudaMalloc(&d_src, src_size));
    CUDA_CHECK(cudaMalloc(&d_dst, dst_size));
    
    CUDA_CHECK(cudaMemcpy(d_src, src, src_size, cudaMemcpyHostToDevice));
    
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x, (dst_h + block.y - 1) / block.y);
    
    cudaEvent_t start, stop;
    CUDA_CHECK(cudaEventCreate(&start));
    CUDA_CHECK(cudaEventCreate(&stop));
    
    CUDA_CHECK(cudaEventRecord(start));
    
    resize_bilinear_kernel<<<grid, block>>>(d_src, d_dst, src_w, src_h, dst_w, dst_h);
    
    CUDA_CHECK(cudaEventRecord(stop));
    CUDA_CHECK(cudaEventSynchronize(stop));
    
    float elapsed_ms = 0;
    CUDA_CHECK(cudaEventElapsedTime(&elapsed_ms, start, stop));
    *gpu_time_ms = elapsed_ms;
    
    CUDA_CHECK(cudaMemcpy(dst, d_dst, dst_size, cudaMemcpyDeviceToHost));
    
    CUDA_CHECK(cudaEventDestroy(start));
    CUDA_CHECK(cudaEventDestroy(stop));
    CUDA_CHECK(cudaFree(d_src));
    CUDA_CHECK(cudaFree(d_dst));
    
    return 0;
}

}  // extern "C"