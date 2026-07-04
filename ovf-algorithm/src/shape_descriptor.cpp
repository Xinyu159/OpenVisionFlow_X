/**
 * @file shape_descriptor.cpp
 * @brief 形状描述符和形状特征算子实现
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <complex>
#include <algorithm>
#include <numeric>

#include "ovf/algorithm/shape_descriptor.h"
#include "ovf/core/logger.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ovf {
namespace algorithm {

// ==================== 形状描述符工具函数实现 ====================

namespace shape_utils {

/**
 * @brief 从图像提取轮廓（边界跟踪算法）
 */
std::vector<ContourPoint> extract_contour(const ImageData& binary_img) {
    std::vector<ContourPoint> contour;
    
    if (binary_img.empty() || binary_img.channels != 1) {
        return contour;
    }
    
    uint32_t width = binary_img.width;
    uint32_t height = binary_img.height;
    
    // 查找起始点（第一个前景像素）
    int start_x = -1, start_y = -1;
    for (uint32_t y = 0; y < height && start_x < 0; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            if (binary_img.data[y * width + x] > 0) {
                start_x = static_cast<int>(x);
                start_y = static_cast<int>(y);
                break;
            }
        }
    }
    
    if (start_x < 0) {
        return contour;  // 无前景像素
    }
    
    // 8邻域方向：右、右下、下、左下、左、左上、上、右上
    const int dx[] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dy[] = {0, 1, 1, 1, 0, -1, -1, -1};
    
    // 边界跟踪（Moore邻域算法）
    int x = start_x, y = start_y;
    int dir = 0;  // 初始方向：右
    
    contour.emplace_back(static_cast<float>(x), static_cast<float>(y));
    
    int prev_x = x - 1;  // 起始时的前一个点（左边）
    int prev_y = y;
    
    bool first_step = true;
    
    do {
        // 找到下一个边界点
        int start_dir = (dir + 5) % 8;  // 从左后方开始搜索
        
        bool found = false;
        for (int i = 0; i < 8; ++i) {
            int search_dir = (start_dir + i) % 8;
            int nx = x + dx[search_dir];
            int ny = y + dy[search_dir];
            
            if (nx >= 0 && nx < static_cast<int>(width) &&
                ny >= 0 && ny < static_cast<int>(height) &&
                binary_img.data[ny * width + nx] > 0) {
                x = nx;
                y = ny;
                dir = search_dir;
                found = true;
                break;
            }
        }
        
        if (!found) {
            break;  // 无法继续跟踪
        }
        
        if (!first_step && x == start_x && y == start_y) {
            break;  // 回到起点，完成
        }
        
        contour.emplace_back(static_cast<float>(x), static_cast<float>(y));
        first_step = false;
        
    } while (contour.size() < width * height);  // 防止无限循环
    
    return contour;
}

/**
 * @brief 计算轮廓面积（使用Green公式）
 */
double contour_area(const std::vector<ContourPoint>& contour) {
    if (contour.size() < 3) {
        return 0.0;
    }
    
    double area = 0.0;
    size_t n = contour.size();
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += contour[i].x * contour[j].y;
        area -= contour[j].x * contour[i].y;
    }
    
    return std::abs(area) / 2.0;
}

/**
 * @brief 计算轮廓周长
 */
double contour_perimeter(const std::vector<ContourPoint>& contour) {
    if (contour.size() < 2) {
        return 0.0;
    }
    
    double perimeter = 0.0;
    size_t n = contour.size();
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        perimeter += contour[i].distance_to(contour[j]);
    }
    
    return perimeter;
}

/**
 * @brief 计算轮廓质心
 */
void contour_centroid(const std::vector<ContourPoint>& contour, double& cx, double& cy) {
    if (contour.empty()) {
        cx = 0.0;
        cy = 0.0;
        return;
    }
    
    double area = contour_area(contour);
    
    if (area < 1e-10) {
        // 使用简单平均
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto& p : contour) {
            sum_x += p.x;
            sum_y += p.y;
        }
        cx = sum_x / contour.size();
        cy = sum_y / contour.size();
        return;
    }
    
    // 使用面积加权计算质心
    double sum_x = 0.0, sum_y = 0.0;
    size_t n = contour.size();
    
    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        double factor = contour[i].x * contour[j].y - contour[j].x * contour[i].y;
        sum_x += (contour[i].x + contour[j].x) * factor;
        sum_y += (contour[i].y + contour[j].y) * factor;
    }
    
    cx = sum_x / (6.0 * area);
    cy = sum_y / (6.0 * area);
}

/**
 * @brief 计算几何矩 m_pq
 */
double geometric_moment(const std::vector<ContourPoint>& contour, int p, int q, double cx, double cy) {
    if (contour.empty()) {
        return 0.0;
    }
    
    // 对于轮廓，使用离散积分近似
    double moment = 0.0;
    
    for (const auto& pt : contour) {
        double dx = pt.x - cx;
        double dy = pt.y - cy;
        moment += std::pow(dx, p) * std::pow(dy, q);
    }
    
    // 考虑面积权重（使用Green公式的积分形式）
    double area = contour_area(contour);
    if (area > 1e-10 && p + q <= 3) {
        // 低阶矩使用更精确的计算
        size_t n = contour.size();
        double sum = 0.0;
        
        for (size_t i = 0; i < n; ++i) {
            size_t j = (i + 1) % n;
            double x_i = contour[i].x - cx;
            double y_i = contour[i].y - cy;
            double x_j = contour[j].x - cx;
            double y_j = contour[j].y - cy;
            
            double factor = x_i * y_j - x_j * y_i;
            
            // 使用积分公式
            if (p == 0 && q == 0) {
                sum += factor;  // 面积
            } else if (p == 1 && q == 0) {
                sum += (x_i + x_j) * factor;  // M10
            } else if (p == 0 && q == 1) {
                sum += (y_i + y_j) * factor;  // M01
            } else if (p == 2 && q == 0) {
                sum += (x_i * x_i + x_i * x_j + x_j * x_j) * factor;  // M20
            } else if (p == 0 && q == 2) {
                sum += (y_i * y_i + y_i * y_j + y_j * y_j) * factor;  // M02
            } else if (p == 1 && q == 1) {
                sum += (2 * x_i * y_i + x_i * y_j + x_j * y_i + 2 * x_j * y_j) * factor;  // M11
            }
        }
        
        if (p == 0 && q == 0) {
            return std::abs(sum) / 2.0;
        } else {
            return std::abs(sum) / 6.0;
        }
    }
    
    return moment;
}

/**
 * @brief 计算中心矩 mu_pq
 */
double central_moment(const std::vector<ContourPoint>& contour, int p, int q) {
    double cx, cy;
    contour_centroid(contour, cx, cy);
    return geometric_moment(contour, p, q, cx, cy);
}

/**
 * @brief 计算归一化中心矩 nu_pq
 */
double normalized_central_moment(const std::vector<ContourPoint>& contour, int p, int q) {
    double mu = central_moment(contour, p, q);
    double area = contour_area(contour);
    
    if (area < 1e-10) {
        return 0.0;
    }
    
    // 归一化：nu_pq = mu_pq / mu_00^(p+q+2)/2
    double norm = std::pow(area, (p + q + 2) / 2.0);
    
    return mu / norm;
}

/**
 * @brief 计算Hu不变矩（7个）
 */
std::vector<double> hu_moments(const std::vector<ContourPoint>& contour) {
    std::vector<double> hu(7, 0.0);
    
    if (contour.size() < 3) {
        return hu;
    }
    
    // 计算归一化中心矩
    double nu20 = normalized_central_moment(contour, 2, 0);
    double nu02 = normalized_central_moment(contour, 0, 2);
    double nu11 = normalized_central_moment(contour, 1, 1);
    double nu30 = normalized_central_moment(contour, 3, 0);
    double nu03 = normalized_central_moment(contour, 0, 3);
    double nu21 = normalized_central_moment(contour, 2, 1);
    double nu12 = normalized_central_moment(contour, 1, 2);
    
    // Hu不变矩公式
    hu[0] = nu20 + nu02;
    
    hu[1] = (nu20 - nu02) * (nu20 - nu02) + 4 * nu11 * nu11;
    
    hu[2] = (nu30 - 3 * nu12) * (nu30 - 3 * nu12) + 
            (3 * nu21 - nu03) * (3 * nu21 - nu03);
    
    hu[3] = (nu30 + nu12) * (nu30 + nu12) + 
            (nu21 + nu03) * (nu21 + nu03);
    
    hu[4] = (nu30 - 3 * nu12) * (nu30 + nu12) * 
            ((nu30 + nu12) * (nu30 + nu12) - 3 * (nu21 + nu03) * (nu21 + nu03)) +
            (3 * nu21 - nu03) * (nu21 + nu03) * 
            (3 * (nu30 + nu12) * (nu30 + nu12) - (nu21 + nu03) * (nu21 + nu03));
    
    hu[5] = (nu20 - nu02) * 
            ((nu30 + nu12) * (nu30 + nu12) - (nu21 + nu03) * (nu21 + nu03)) +
            4 * nu11 * (nu30 + nu12) * (nu21 + nu03);
    
    hu[6] = (3 * nu21 - nu03) * (nu30 + nu12) * 
            ((nu30 + nu12) * (nu30 + nu12) - 3 * (nu21 + nu03) * (nu21 + nu03)) -
            (nu30 - 3 * nu12) * (nu21 + nu03) * 
            (3 * (nu30 + nu12) * (nu30 + nu12) - (nu21 + nu03) * (nu21 + nu03));
    
    // 对数变换（减少数值范围）
    for (int i = 0; i < 7; ++i) {
        if (hu[i] > 0) {
            hu[i] = -std::log(std::abs(hu[i]));
        } else if (hu[i] < 0) {
            hu[i] = std::log(std::abs(hu[i]));
        }
    }
    
    return hu;
}

/**
 * @brief 计算Flusser不变矩（基于复数矩）
 */
std::vector<double> flusser_moments(const std::vector<ContourPoint>& contour) {
    std::vector<double> flusser(4, 0.0);
    
    if (contour.size() < 3) {
        return flusser;
    }
    
    double cx, cy;
    contour_centroid(contour, cx, cy);
    double area = contour_area(contour);
    
    if (area < 1e-10) {
        return flusser;
    }
    
    // 计算复数矩
    auto complex_moment = [&](int p, int q) -> std::complex<double> {
        std::complex<double> sum(0.0, 0.0);
        for (const auto& pt : contour) {
            double dx = pt.x - cx;
            double dy = pt.y - cy;
            std::complex<double> z(dx, dy);
            sum += std::pow(z, p) * std::pow(std::conj(z), q);
        }
        return sum;
    };
    
    // 归一化因子
    double norm = std::pow(area, (2 + 2) / 2.0);
    
    // Flusser不变矩（简化的4个低阶不变矩）
    std::complex<double> c20 = complex_moment(2, 0) / norm;
    std::complex<double> c11 = complex_moment(1, 1) / norm;
    std::complex<double> c30 = complex_moment(3, 0) / norm;
    std::complex<double> c21 = complex_moment(2, 1) / norm;
    
    // I1 = c20 / c11（绝对值）
    flusser[0] = std::abs(c20) / std::abs(c11);
    
    // I2 = Re(c20^2 / c11^3)
    if (std::abs(c11) > 1e-10) {
        std::complex<double> temp = c20 * c20 / std::pow(c11, 3);
        flusser[1] = temp.real();
    }
    
    // I3 = Re(c30 / c11^2)
    if (std::abs(c11) > 1e-10) {
        std::complex<double> temp = c30 / std::pow(c11, 2);
        flusser[2] = temp.real();
    }
    
    // I4 = Im(c30 / c11^2)
    if (std::abs(c11) > 1e-10) {
        std::complex<double> temp = c30 / std::pow(c11, 2);
        flusser[3] = temp.imag();
    }
    
    return flusser;
}

/**
 * @brief 计算凸包（Graham扫描算法）
 */
std::vector<ContourPoint> convex_hull(const std::vector<ContourPoint>& contour) {
    if (contour.size() < 3) {
        return contour;
    }
    
    std::vector<ContourPoint> points = contour;
    
    // 找到最低最左点作为起始点
    size_t start = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        if (points[i].y < points[start].y ||
            (points[i].y == points[start].y && points[i].x < points[start].x)) {
            start = i;
        }
    }
    
    std::swap(points[0], points[start]);
    ContourPoint pivot = points[0];
    
    // 按极角排序
    auto polar_angle = [](const ContourPoint& p, const ContourPoint& ref) -> double {
        double dx = p.x - ref.x;
        double dy = p.y - ref.y;
        return std::atan2(dy, dx);
    };
    
    auto cross_product = [](const ContourPoint& o, const ContourPoint& a, const ContourPoint& b) -> double {
        return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    };
    
    std::sort(points.begin() + 1, points.end(), [&](const ContourPoint& a, const ContourPoint& b) {
        double angle_a = polar_angle(a, pivot);
        double angle_b = polar_angle(b, pivot);
        
        if (angle_a != angle_b) {
            return angle_a < angle_b;
        }
        
        return a.distance_to(pivot) < b.distance_to(pivot);
    });
    
    // Graham扫描
    std::vector<ContourPoint> hull;
    hull.push_back(points[0]);
    
    for (size_t i = 1; i < points.size(); ++i) {
        while (hull.size() > 1 && 
               cross_product(hull[hull.size() - 2], hull[hull.size() - 1], points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }
    
    return hull;
}

/**
 * @brief 计算圆度 circularity = 4*pi*A/P^2
 */
double circularity(double area, double perimeter) {
    if (perimeter < 1e-10) {
        return 0.0;
    }
    return 4.0 * M_PI * area / (perimeter * perimeter);
}

/**
 * @brief 计算矩形度 rectangularity = A/(w*h)
 */
double rectangularity(double area, double width, double height) {
    if (width < 1e-10 || height < 1e-10) {
        return 0.0;
    }
    return area / (width * height);
}

/**
 * @brief 计算凸度 convexity = A_hull/A
 */
double convexity(double area, double hull_area) {
    if (hull_area < 1e-10) {
        return 0.0;
    }
    return area / hull_area;
}

/**
 * @brief 计算紧凑度 compactness = sqrt(4*A/pi)/P
 */
double compactness(double area, double perimeter) {
    if (perimeter < 1e-10) {
        return 0.0;
    }
    return std::sqrt(4.0 * area / M_PI) / perimeter;
}

/**
 * @brief 计算主轴和次轴（基于中心矩）
 */
void compute_axes(const std::vector<ContourPoint>& contour, 
                  double& major_axis, double& minor_axis, double& orientation) {
    if (contour.size() < 3) {
        major_axis = 0.0;
        minor_axis = 0.0;
        orientation = 0.0;
        return;
    }
    
    // 计算中心矩
    double mu20 = central_moment(contour, 2, 0);
    double mu02 = central_moment(contour, 0, 2);
    double mu11 = central_moment(contour, 1, 1);
    
    // 计算方向角
    orientation = 0.5 * std::atan2(2.0 * mu11, mu20 - mu02);
    
    // 计算椭圆参数
    double diff = mu20 - mu02;
    double sum = mu20 + mu02;
    double sqrt_term = std::sqrt(4.0 * mu11 * mu11 + diff * diff);
    
    // 椭圆半轴长度
    major_axis = 2.0 * std::sqrt((sum + sqrt_term) / 2.0);
    minor_axis = 2.0 * std::sqrt((sum - sqrt_term) / 2.0);
    
    if (minor_axis < 1e-10) {
        minor_axis = major_axis;  // 避免除零
    }
}

/**
 * @brief FFT实现（Cooley-Tukey算法）
 */
void fft(std::vector<std::complex<double>>& x) {
    size_t n = x.size();
    
    if (n <= 1) return;
    
    // 检查是否为2的幂
    if ((n & (n - 1)) != 0) {
        // 补零到最近的2的幂
        size_t new_n = 1;
        while (new_n < n) new_n <<= 1;
        x.resize(new_n, std::complex<double>(0.0, 0.0));
        n = new_n;
    }
    
    // 位逆序排列
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        
        if (i < j) {
            std::swap(x[i], x[j]);
        }
    }
    
    // FFT迭代计算
    for (size_t len = 2; len <= n; len <<= 1) {
        double angle = -2.0 * M_PI / len;
        std::complex<double> wlen(std::cos(angle), std::sin(angle));
        
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1.0, 0.0);
            for (size_t j = 0; j < len / 2; ++j) {
                std::complex<double> u = x[i + j];
                std::complex<double> v = x[i + j + len / 2] * w;
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
}

/**
 * @brief 计算Fourier描述符
 */
std::vector<std::complex<double>> fourier_descriptors(const std::vector<ContourPoint>& contour, int num_descriptors) {
    std::vector<std::complex<double>> descriptors;
    
    if (contour.size() < 4) {
        return descriptors;
    }
    
    // 构建复数序列
    std::vector<std::complex<double>> signal;
    for (const auto& pt : contour) {
        signal.emplace_back(pt.x, pt.y);
    }
    
    // FFT
    fft(signal);
    
    // 提取低频分量作为描述符
    int half = num_descriptors / 2;
    
    // DC分量（零频）
    descriptors.push_back(signal[0]);
    
    // 低频正分量
    for (int i = 1; i <= half && i < static_cast<int>(signal.size() / 2); ++i) {
        descriptors.push_back(signal[i]);
    }
    
    // 低频负分量（高频端）
    for (int i = static_cast<int>(signal.size()) - half; i < static_cast<int>(signal.size()) && descriptors.size() < num_descriptors; ++i) {
        descriptors.push_back(signal[i]);
    }
    
    // 归一化（使用FD1作为基准）
    if (signal.size() > 1 && std::abs(signal[1]) > 1e-10) {
        for (auto& fd : descriptors) {
            fd /= signal[1];
        }
    }
    
    return descriptors;
}

/**
 * @brief 计算曲率描述符
 */
std::vector<double> curvature_descriptor(const std::vector<ContourPoint>& contour, int num_points) {
    std::vector<double> curvatures;
    
    if (contour.size() < 3) {
        return curvatures;
    }
    
    // 重采样轮廓
    std::vector<ContourPoint> sampled;
    double perimeter = contour_perimeter(contour);
    double step = perimeter / num_points;
    
    double accumulated = 0.0;
    size_t current_idx = 0;
    
    for (int i = 0; i < num_points; ++i) {
        double target_dist = i * step;
        
        while (current_idx < contour.size() - 1 && accumulated < target_dist) {
            accumulated += contour[current_idx].distance_to(contour[current_idx + 1]);
            ++current_idx;
        }
        
        sampled.push_back(contour[current_idx % contour.size()]);
    }
    
    // 计算曲率（使用三点公式）
    for (int i = 0; i < num_points; ++i) {
        int prev = (i - 1 + num_points) % num_points;
        int next = (i + 1) % num_points;
        
        ContourPoint p0 = sampled[prev];
        ContourPoint p1 = sampled[i];
        ContourPoint p2 = sampled[next];
        
        // 计算曲率 kappa = |x'(t)y''(t) - y'(t)x''(t)| / (x'(t)^2 + y'(t)^2)^(3/2)
        double dx1 = p1.x - p0.x;
        double dy1 = p1.y - p0.y;
        double dx2 = p2.x - p1.x;
        double dy2 = p2.y - p1.y;
        
        double ddx = dx2 - dx1;  // x''
        double ddy = dy2 - dy1;  // y''
        
        double denom = std::pow(dx1 * dx1 + dy1 * dy1, 1.5);
        
        if (denom > 1e-10) {
            curvatures.push_back(std::abs(dx1 * ddy - dy1 * ddx) / denom);
        } else {
            curvatures.push_back(0.0);
        }
    }
    
    return curvatures;
}

/**
 * @brief 计算轮廓签名（距离签名）
 */
std::vector<double> contour_signature(const std::vector<ContourPoint>& contour, int num_samples) {
    std::vector<double> signature;
    
    if (contour.size() < 3) {
        return signature;
    }
    
    // 计算质心
    double cx, cy;
    contour_centroid(contour, cx, cy);
    ContourPoint centroid(static_cast<float>(cx), static_cast<float>(cy));
    
    // 重采样轮廓
    double perimeter = contour_perimeter(contour);
    double step = perimeter / num_samples;
    
    double accumulated = 0.0;
    size_t current_idx = 0;
    
    for (int i = 0; i < num_samples; ++i) {
        double target_dist = i * step;
        
        while (current_idx < contour.size() - 1 && accumulated < target_dist) {
            accumulated += contour[current_idx].distance_to(contour[current_idx + 1]);
            ++current_idx;
        }
        
        ContourPoint pt = contour[current_idx % contour.size()];
        signature.push_back(pt.distance_to(centroid));
    }
    
    // 归一化
    double max_dist = 0.0;
    for (double d : signature) {
        max_dist = std::max(max_dist, d);
    }
    
    if (max_dist > 1e-10) {
        for (double& d : signature) {
            d /= max_dist;
        }
    }
    
    return signature;
}

/**
 * @brief 计算形状上下文描述符（简化版）
 */
std::vector<std::vector<int>> shape_context(const std::vector<ContourPoint>& contour,
                                             int num_points,
                                             int num_r_bins,
                                             int num_theta_bins) {
    std::vector<std::vector<int>> contexts;
    
    if (contour.size() < 3) {
        return contexts;
    }
    
    // 重采样轮廓
    double perimeter = contour_perimeter(contour);
    double step = perimeter / num_points;
    
    std::vector<ContourPoint> sampled;
    double accumulated = 0.0;
    size_t current_idx = 0;
    
    for (int i = 0; i < num_points; ++i) {
        double target_dist = i * step;
        
        while (current_idx < contour.size() - 1 && accumulated < target_dist) {
            accumulated += contour[current_idx].distance_to(contour[current_idx + 1]);
            ++current_idx;
        }
        
        sampled.push_back(contour[current_idx % contour.size()]);
    }
    
    // 计算平均距离作为参考尺度
    double mean_dist = 0.0;
    for (int i = 0; i < num_points; ++i) {
        for (int j = i + 1; j < num_points; ++j) {
            mean_dist += sampled[i].distance_to(sampled[j]);
        }
    }
    mean_dist /= (num_points * (num_points - 1) / 2.0);
    
    // 构建对数距离分箱
    std::vector<double> r_bins(num_r_bins + 1);
    for (int i = 0; i <= num_r_bins; ++i) {
        r_bins[i] = mean_dist * std::pow(2.0, i - num_r_bins / 2.0);
    }
    
    // 角度分箱
    std::vector<double> theta_bins(num_theta_bins + 1);
    for (int i = 0; i <= num_theta_bins; ++i) {
        theta_bins[i] = 2.0 * M_PI * i / num_theta_bins;
    }
    
    // 计算每个点的形状上下文
    for (int i = 0; i < num_points; ++i) {
        std::vector<int> histogram(num_r_bins * num_theta_bins, 0);
        
        for (int j = 0; j < num_points; ++j) {
            if (i == j) continue;
            
            double dx = sampled[j].x - sampled[i].x;
            double dy = sampled[j].y - sampled[i].y;
            
            double dist = std::sqrt(dx * dx + dy * dy);
            double angle = std::atan2(dy, dx);
            
            // 确定距离分箱
            int r_bin = -1;
            for (int k = 0; k < num_r_bins; ++k) {
                if (dist >= r_bins[k] && dist < r_bins[k + 1]) {
                    r_bin = k;
                    break;
                }
            }
            if (r_bin < 0) r_bin = num_r_bins - 1;
            
            // 确定角度分箱
            double normalized_angle = angle + M_PI;  // 映射到 [0, 2*pi]
            if (normalized_angle >= 2.0 * M_PI) {
                normalized_angle -= 2.0 * M_PI;
            }
            int theta_bin = static_cast<int>(normalized_angle * num_theta_bins / (2.0 * M_PI));
            if (theta_bin >= num_theta_bins) theta_bin = num_theta_bins - 1;
            
            // 填充直方图
            int bin_idx = r_bin * num_theta_bins + theta_bin;
            histogram[bin_idx]++;
        }
        
        contexts.push_back(histogram);
    }
    
    return contexts;
}

} // namespace shape_utils

// ==================== 不变矩节点实现 ====================

HuMomentsNode::HuMomentsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo HuMomentsNode::make_info() {
    NodeInfo info;
    info.id = "HuMoments";
    info.name = "Hu不变矩";
    info.category = "形状描述";
    info.description = "计算形状的7个Hu不变矩（旋转缩放平移不变量）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("hu1", "Hu不变矩h1", DataType::Number));
    info.outputs.push_back(DataPort("hu2", "Hu不变矩h2", DataType::Number));
    info.outputs.push_back(DataPort("hu3", "Hu不变矩h3", DataType::Number));
    info.outputs.push_back(DataPort("hu4", "Hu不变矩h4", DataType::Number));
    info.outputs.push_back(DataPort("hu5", "Hu不变矩h5", DataType::Number));
    info.outputs.push_back(DataPort("hu6", "Hu不变矩h6", DataType::Number));
    info.outputs.push_back(DataPort("hu7", "Hu不变矩h7", DataType::Number));
    
    return info;
}

Result<void> HuMomentsNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为二值图像
    ImageData binary;
    if (input.channels != 1) {
        binary.width = input.width;
        binary.height = input.height;
        binary.channels = 1;
        binary.format = ImageFormat::Mono8;
        binary.data.resize(binary.width * binary.height);
        
        for (size_t i = 0; i < binary.data.size(); ++i) {
            if (input.channels >= 3) {
                uint8_t b = input.data[i * 3];
                uint8_t g = input.data[i * 3 + 1];
                uint8_t r = input.data[i * 3 + 2];
                binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
            } else {
                binary.data[i] = input.data[i] > 128 ? 255 : 0;
            }
        }
    } else {
        binary.width = input.width;
        binary.height = input.height;
        binary.channels = 1;
        binary.format = ImageFormat::Mono8;
        binary.data.resize(binary.width * binary.height);
        for (size_t i = 0; i < binary.data.size(); ++i) {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    // 提取轮廓
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    // 计算Hu不变矩
    auto hu = shape_utils::hu_moments(contour);
    
    // 输出结果
    set_output("hu1", Data(hu[0]));
    set_output("hu2", Data(hu[1]));
    set_output("hu3", Data(hu[2]));
    set_output("hu4", Data(hu[3]));
    set_output("hu5", Data(hu[4]));
    set_output("hu6", Data(hu[5]));
    set_output("hu7", Data(hu[6]));
    
    OVF_INFO() << "Hu moments computed: h1=" << hu[0] << ", h2=" << hu[1] 
               << ", h3=" << hu[2] << ", h4=" << hu[3];
    
    return Result<void>::success();
}

ShapeMomentsNode::ShapeMomentsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ShapeMomentsNode::make_info() {
    NodeInfo info;
    info.id = "ShapeMoments";
    info.name = "形状矩";
    info.category = "形状描述";
    info.description = "计算形状的面积矩和中心矩";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("centroid_x", "质心X", DataType::Number));
    info.outputs.push_back(DataPort("centroid_y", "质心Y", DataType::Number));
    info.outputs.push_back(DataPort("m20", "几何矩m20", DataType::Number));
    info.outputs.push_back(DataPort("m02", "几何矩m02", DataType::Number));
    info.outputs.push_back(DataPort("m11", "几何矩m11", DataType::Number));
    info.outputs.push_back(DataPort("mu20", "中心矩mu20", DataType::Number));
    info.outputs.push_back(DataPort("mu02", "中心矩mu02", DataType::Number));
    info.outputs.push_back(DataPort("mu11", "中心矩mu11", DataType::Number));
    
    return info;
}

Result<void> ShapeMomentsNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    // 提取轮廓
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    // 计算各种矩
    double area = shape_utils::contour_area(contour);
    double cx, cy;
    shape_utils::contour_centroid(contour, cx, cy);
    
    double m20 = shape_utils::geometric_moment(contour, 2, 0, 0, 0);
    double m02 = shape_utils::geometric_moment(contour, 0, 2, 0, 0);
    double m11 = shape_utils::geometric_moment(contour, 1, 1, 0, 0);
    
    double mu20 = shape_utils::central_moment(contour, 2, 0);
    double mu02 = shape_utils::central_moment(contour, 0, 2);
    double mu11 = shape_utils::central_moment(contour, 1, 1);
    
    // 输出结果
    set_output("area", Data(area));
    set_output("centroid_x", Data(cx));
    set_output("centroid_y", Data(cy));
    set_output("m20", Data(m20));
    set_output("m02", Data(m02));
    set_output("m11", Data(m11));
    set_output("mu20", Data(mu20));
    set_output("mu02", Data(mu02));
    set_output("mu11", Data(mu11));
    
    OVF_INFO() << "Shape moments: area=" << area << ", centroid=(" << cx << "," << cy << ")";
    
    return Result<void>::success();
}

FlusserMomentsNode::FlusserMomentsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FlusserMomentsNode::make_info() {
    NodeInfo info;
    info.id = "FlusserMoments";
    info.name = "Flusser不变矩";
    info.category = "形状描述";
    info.description = "计算形状的Flusser不变矩（基于复数矩）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("f1", "Flusser不变矩I1", DataType::Number));
    info.outputs.push_back(DataPort("f2", "Flusser不变矩I2", DataType::Number));
    info.outputs.push_back(DataPort("f3", "Flusser不变矩I3", DataType::Number));
    info.outputs.push_back(DataPort("f4", "Flusser不变矩I4", DataType::Number));
    
    return info;
}

Result<void> FlusserMomentsNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    // 提取轮廓
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    // 计算Flusser不变矩
    auto flusser = shape_utils::flusser_moments(contour);
    
    // 输出结果
    set_output("f1", Data(flusser[0]));
    set_output("f2", Data(flusser[1]));
    set_output("f3", Data(flusser[2]));
    set_output("f4", Data(flusser[3]));
    
    OVF_INFO() << "Flusser moments: I1=" << flusser[0] << ", I2=" << flusser[1];
    
    return Result<void>::success();
}

MomentInvariantsNode::MomentInvariantsNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo MomentInvariantsNode::make_info() {
    NodeInfo info;
    info.id = "MomentInvariants";
    info.name = "矩不变量";
    info.category = "形状描述";
    info.description = "计算形状的矩不变量（综合Hu和Flusser）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("hu_moments", "Hu不变矩列表", DataType::Array));
    info.outputs.push_back(DataPort("flusser_moments", "Flusser不变矩列表", DataType::Array));
    info.outputs.push_back(DataPort("normalized_moments", "归一化中心矩列表", DataType::Array));
    
    return info;
}

Result<void> MomentInvariantsNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    // 提取轮廓
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    // 计算各种不变矩
    auto hu = shape_utils::hu_moments(contour);
    auto flusser = shape_utils::flusser_moments(contour);
    
    // 计算归一化中心矩
    std::vector<double> norm_moments;
    norm_moments.push_back(shape_utils::normalized_central_moment(contour, 2, 0));
    norm_moments.push_back(shape_utils::normalized_central_moment(contour, 0, 2));
    norm_moments.push_back(shape_utils::normalized_central_moment(contour, 1, 1));
    norm_moments.push_back(shape_utils::normalized_central_moment(contour, 3, 0));
    norm_moments.push_back(shape_utils::normalized_central_moment(contour, 0, 3));
    norm_moments.push_back(shape_utils::normalized_central_moment(contour, 2, 1));
    norm_moments.push_back(shape_utils::normalized_central_moment(contour, 1, 2));
    
    // 输出结果（简化为数组形式）
    // 注意：由于Data类没有直接的数组存储，我们输出多个端口
    for (size_t i = 0; i < hu.size(); ++i) {
        set_output("hu" + std::to_string(i + 1), Data(hu[i]));
    }
    for (size_t i = 0; i < flusser.size(); ++i) {
        set_output("f" + std::to_string(i + 1), Data(flusser[i]));
    }
    for (size_t i = 0; i < norm_moments.size(); ++i) {
        set_output("nu" + std::to_string(i + 1), Data(norm_moments[i]));
    }
    
    OVF_INFO() << "Moment invariants computed successfully";
    
    return Result<void>::success();
}

// ==================== 形状特征节点实现 ====================

ShapeFeaturesNode::ShapeFeaturesNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ShapeFeaturesNode::make_info() {
    NodeInfo info;
    info.id = "ShapeFeatures";
    info.name = "形状特征";
    info.category = "形状描述";
    info.description = "计算形状的综合特征（圆度、矩形度、凸度等）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("perimeter", "周长", DataType::Number));
    info.outputs.push_back(DataPort("circularity", "圆度", DataType::Number));
    info.outputs.push_back(DataPort("rectangularity", "矩形度", DataType::Number));
    info.outputs.push_back(DataPort("convexity", "凸度", DataType::Number));
    info.outputs.push_back(DataPort("compactness", "紧凑度", DataType::Number));
    info.outputs.push_back(DataPort("anisometry", "各向异性", DataType::Number));
    info.outputs.push_back(DataPort("eccentricity", "偏心率", DataType::Number));
    info.outputs.push_back(DataPort("orientation", "方向角", DataType::Number));
    info.outputs.push_back(DataPort("major_axis", "主轴长度", DataType::Number));
    info.outputs.push_back(DataPort("minor_axis", "次轴长度", DataType::Number));
    info.outputs.push_back(DataPort("width", "边界框宽度", DataType::Number));
    info.outputs.push_back(DataPort("height", "边界框高度", DataType::Number));
    
    return info;
}

Result<void> ShapeFeaturesNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    // 提取轮廓
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    // 计算基本特征
    double area = shape_utils::contour_area(contour);
    double perimeter = shape_utils::contour_perimeter(contour);
    
    // 计算边界框
    float min_x = contour[0].x, max_x = contour[0].x;
    float min_y = contour[0].y, max_y = contour[0].y;
    for (const auto& pt : contour) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }
    double width = max_x - min_x + 1;
    double height = max_y - min_y + 1;
    
    // 计算凸包面积
    auto hull = shape_utils::convex_hull(contour);
    double hull_area = shape_utils::contour_area(hull);
    
    // 计算各种形状特征
    double circularity = shape_utils::circularity(area, perimeter);
    double rectangularity = shape_utils::rectangularity(area, width, height);
    double convexity = shape_utils::convexity(area, hull_area);
    double compactness = shape_utils::compactness(area, perimeter);
    
    // 计算主轴和次轴
    double major_axis, minor_axis, orientation;
    shape_utils::compute_axes(contour, major_axis, minor_axis, orientation);
    
    double anisometry = major_axis / minor_axis;
    double eccentricity = std::sqrt(1.0 - (minor_axis * minor_axis) / (major_axis * major_axis));
    
    // 输出结果
    set_output("area", Data(area));
    set_output("perimeter", Data(perimeter));
    set_output("circularity", Data(circularity));
    set_output("rectangularity", Data(rectangularity));
    set_output("convexity", Data(convexity));
    set_output("compactness", Data(compactness));
    set_output("anisometry", Data(anisometry));
    set_output("eccentricity", Data(eccentricity));
    set_output("orientation", Data(orientation));
    set_output("major_axis", Data(major_axis));
    set_output("minor_axis", Data(minor_axis));
    set_output("width", Data(width));
    set_output("height", Data(height));
    
    OVF_INFO() << "Shape features: circularity=" << circularity 
               << ", convexity=" << convexity
               << ", anisometry=" << anisometry;
    
    return Result<void>::success();
}

CompactnessNode::CompactnessNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CompactnessNode::make_info() {
    NodeInfo info;
    info.id = "Compactness";
    info.name = "紧凑度";
    info.category = "形状描述";
    info.description = "计算形状的紧凑度（sqrt(4*A/pi)/P）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("compactness", "紧凑度", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("perimeter", "周长", DataType::Number));
    
    return info;
}

Result<void> CompactnessNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    double area = shape_utils::contour_area(contour);
    double perimeter = shape_utils::contour_perimeter(contour);
    double compactness = shape_utils::compactness(area, perimeter);
    
    set_output("compactness", Data(compactness));
    set_output("area", Data(area));
    set_output("perimeter", Data(perimeter));
    
    OVF_INFO() << "Compactness: " << compactness;
    
    return Result<void>::success();
}

ConvexityNode::ConvexityNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ConvexityNode::make_info() {
    NodeInfo info;
    info.id = "Convexity";
    info.name = "凸度";
    info.category = "形状描述";
    info.description = "计算形状的凸度（A/A_hull）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("convexity", "凸度", DataType::Number));
    info.outputs.push_back(DataPort("area", "面积", DataType::Number));
    info.outputs.push_back(DataPort("hull_area", "凸包面积", DataType::Number));
    info.outputs.push_back(DataPort("hull_perimeter", "凸包周长", DataType::Number));
    
    return info;
}

Result<void> ConvexityNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    double area = shape_utils::contour_area(contour);
    auto hull = shape_utils::convex_hull(contour);
    double hull_area = shape_utils::contour_area(hull);
    double hull_perimeter = shape_utils::contour_perimeter(hull);
    double convexity = shape_utils::convexity(area, hull_area);
    
    set_output("convexity", Data(convexity));
    set_output("area", Data(area));
    set_output("hull_area", Data(hull_area));
    set_output("hull_perimeter", Data(hull_perimeter));
    
    OVF_INFO() << "Convexity: " << convexity << ", hull area: " << hull_area;
    
    return Result<void>::success();
}

AnisometryNode::AnisometryNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo AnisometryNode::make_info() {
    NodeInfo info;
    info.id = "Anisometry";
    info.name = "各向异性";
    info.category = "形状描述";
    info.description = "计算形状的各向异性（主轴/次轴）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("anisometry", "各向异性", DataType::Number));
    info.outputs.push_back(DataPort("major_axis", "主轴长度", DataType::Number));
    info.outputs.push_back(DataPort("minor_axis", "次轴长度", DataType::Number));
    info.outputs.push_back(DataPort("orientation", "方向角", DataType::Number));
    info.outputs.push_back(DataPort("eccentricity", "偏心率", DataType::Number));
    
    return info;
}

Result<void> AnisometryNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    double major_axis, minor_axis, orientation;
    shape_utils::compute_axes(contour, major_axis, minor_axis, orientation);
    
    double anisometry = major_axis / minor_axis;
    double eccentricity = std::sqrt(1.0 - (minor_axis * minor_axis) / (major_axis * major_axis));
    
    set_output("anisometry", Data(anisometry));
    set_output("major_axis", Data(major_axis));
    set_output("minor_axis", Data(minor_axis));
    set_output("orientation", Data(orientation));
    set_output("eccentricity", Data(eccentricity));
    
    OVF_INFO() << "Anisometry: " << anisometry << ", orientation: " << orientation;
    
    return Result<void>::success();
}

// ==================== 轮廓描述符节点实现 ====================

FourierDescriptorNode::FourierDescriptorNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo FourierDescriptorNode::make_info() {
    NodeInfo info;
    info.id = "FourierDescriptor";
    info.name = "Fourier轮廓描述符";
    info.category = "形状描述";
    info.description = "计算轮廓的Fourier描述符（FFT）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("descriptor_count", "描述符数量", DataType::Number));
    info.outputs.push_back(DataPort("fd1_real", "FD1实部", DataType::Number));
    info.outputs.push_back(DataPort("fd1_imag", "FD1虚部", DataType::Number));
    info.outputs.push_back(DataPort("fd2_real", "FD2实部", DataType::Number));
    info.outputs.push_back(DataPort("fd2_imag", "FD2虚部", DataType::Number));
    info.outputs.push_back(DataPort("fd3_real", "FD3实部", DataType::Number));
    info.outputs.push_back(DataPort("fd3_imag", "FD3虚部", DataType::Number));
    info.outputs.push_back(DataPort("fd4_real", "FD4实部", DataType::Number));
    info.outputs.push_back(DataPort("fd4_imag", "FD4虚部", DataType::Number));
    
    info.params.push_back(ParamDef("num_descriptors", "描述符数量", DataType::Number, Data(32)));
    
    return info;
}

Result<void> FourierDescriptorNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int num_descriptors = get_param("num_descriptors", Data(32)).as_int();
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 4) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Contour too small for Fourier descriptor");
    }
    
    auto descriptors = shape_utils::fourier_descriptors(contour, num_descriptors);
    
    set_output("descriptor_count", Data(static_cast<int>(descriptors.size())));
    
    // 输出前几个描述符
    for (size_t i = 0; i < std::min(descriptors.size(), size_t(4)); ++i) {
        set_output("fd" + std::to_string(i + 1) + "_real", Data(descriptors[i].real()));
        set_output("fd" + std::to_string(i + 1) + "_imag", Data(descriptors[i].imag()));
    }
    
    OVF_INFO() << "Fourier descriptors computed: " << descriptors.size() << " descriptors";
    
    return Result<void>::success();
}

ShapeContextNode::ShapeContextNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ShapeContextNode::make_info() {
    NodeInfo info;
    info.id = "ShapeContext";
    info.name = "形状上下文描述符";
    info.category = "形状描述";
    info.description = "计算形状上下文描述符";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("point_count", "采样点数量", DataType::Number));
    info.outputs.push_back(DataPort("r_bins", "距离分箱数", DataType::Number));
    info.outputs.push_back(DataPort("theta_bins", "角度分箱数", DataType::Number));
    info.outputs.push_back(DataPort("total_bins", "总分箱数", DataType::Number));
    
    info.params.push_back(ParamDef("num_points", "采样点数量", DataType::Number, Data(64)));
    info.params.push_back(ParamDef("num_r_bins", "距离分箱数", DataType::Number, Data(5)));
    info.params.push_back(ParamDef("num_theta_bins", "角度分箱数", DataType::Number, Data(12)));
    
    return info;
}

Result<void> ShapeContextNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int num_points = get_param("num_points", Data(64)).as_int();
    int num_r_bins = get_param("num_r_bins", Data(5)).as_int();
    int num_theta_bins = get_param("num_theta_bins", Data(12)).as_int();
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    auto contexts = shape_utils::shape_context(contour, num_points, num_r_bins, num_theta_bins);
    
    set_output("point_count", Data(static_cast<int>(contexts.size())));
    set_output("r_bins", Data(num_r_bins));
    set_output("theta_bins", Data(num_theta_bins));
    set_output("total_bins", Data(num_r_bins * num_theta_bins));
    
    OVF_INFO() << "Shape context computed: " << contexts.size() << " points, "
               << num_r_bins * num_theta_bins << " bins per point";
    
    return Result<void>::success();
}

CurvatureDescriptorNode::CurvatureDescriptorNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo CurvatureDescriptorNode::make_info() {
    NodeInfo info;
    info.id = "CurvatureDescriptor";
    info.name = "曲率描述符";
    info.category = "形状描述";
    info.description = "计算轮廓的曲率描述符";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("point_count", "采样点数量", DataType::Number));
    info.outputs.push_back(DataPort("max_curvature", "最大曲率", DataType::Number));
    info.outputs.push_back(DataPort("mean_curvature", "平均曲率", DataType::Number));
    info.outputs.push_back(DataPort("std_curvature", "曲率标准差", DataType::Number));
    
    info.params.push_back(ParamDef("num_points", "采样点数量", DataType::Number, Data(64)));
    
    return info;
}

Result<void> CurvatureDescriptorNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int num_points = get_param("num_points", Data(64)).as_int();
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    auto curvatures = shape_utils::curvature_descriptor(contour, num_points);
    
    if (curvatures.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not compute curvature");
    }
    
    double max_curv = 0.0, sum_curv = 0.0;
    for (double c : curvatures) {
        max_curv = std::max(max_curv, c);
        sum_curv += c;
    }
    double mean_curv = sum_curv / curvatures.size();
    
    double var_sum = 0.0;
    for (double c : curvatures) {
        var_sum += (c - mean_curv) * (c - mean_curv);
    }
    double std_curv = std::sqrt(var_sum / curvatures.size());
    
    set_output("point_count", Data(static_cast<int>(curvatures.size())));
    set_output("max_curvature", Data(max_curv));
    set_output("mean_curvature", Data(mean_curv));
    set_output("std_curvature", Data(std_curv));
    
    OVF_INFO() << "Curvature descriptor: max=" << max_curv 
               << ", mean=" << mean_curv << ", std=" << std_curv;
    
    return Result<void>::success();
}

ContourSignatureNode::ContourSignatureNode(const String& instance_id)
    : INode(instance_id, make_info()) {}

NodeInfo ContourSignatureNode::make_info() {
    NodeInfo info;
    info.id = "ContourSignature";
    info.name = "轮廓签名";
    info.category = "形状描述";
    info.description = "计算轮廓签名（距离签名）";
    info.version = "0.1.0";
    
    info.inputs.push_back(DataPort("image", "输入图像（二值）", DataType::Image, true));
    
    info.outputs.push_back(DataPort("sample_count", "采样点数量", DataType::Number));
    info.outputs.push_back(DataPort("max_distance", "最大距离", DataType::Number));
    info.outputs.push_back(DataPort("min_distance", "最小距离", DataType::Number));
    info.outputs.push_back(DataPort("mean_distance", "平均距离", DataType::Number));
    
    info.params.push_back(ParamDef("num_samples", "采样点数量", DataType::Number, Data(64)));
    
    return info;
}

Result<void> ContourSignatureNode::execute(FlowContext& context) {
    auto input_data = get_input("image");
    if (!input_data.is_image()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input is not an image");
    }
    
    ImageData input = input_data.as_image();
    if (input.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Input image is empty");
    }
    
    int num_samples = get_param("num_samples", Data(64)).as_int();
    
    // 转换为二值图像
    ImageData binary;
    binary.width = input.width;
    binary.height = input.height;
    binary.channels = 1;
    binary.format = ImageFormat::Mono8;
    binary.data.resize(binary.width * binary.height);
    
    for (size_t i = 0; i < binary.data.size(); ++i) {
        if (input.channels >= 3) {
            uint8_t b = input.data[i * 3];
            uint8_t g = input.data[i * 3 + 1];
            uint8_t r = input.data[i * 3 + 2];
            binary.data[i] = static_cast<uint8_t>(0.11 * b + 0.59 * g + 0.30 * r) > 128 ? 255 : 0;
        } else {
            binary.data[i] = input.data[i] > 128 ? 255 : 0;
        }
    }
    
    auto contour = shape_utils::extract_contour(binary);
    
    if (contour.size() < 3) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not extract valid contour");
    }
    
    auto signature = shape_utils::contour_signature(contour, num_samples);
    
    if (signature.empty()) {
        return Result<void>::failure(ErrorCode::InvalidImage, "Could not compute signature");
    }
    
    double max_dist = 0.0, min_dist = 1.0, sum_dist = 0.0;
    for (double d : signature) {
        max_dist = std::max(max_dist, d);
        min_dist = std::min(min_dist, d);
        sum_dist += d;
    }
    double mean_dist = sum_dist / signature.size();
    
    set_output("sample_count", Data(static_cast<int>(signature.size())));
    set_output("max_distance", Data(max_dist));
    set_output("min_distance", Data(min_dist));
    set_output("mean_distance", Data(mean_dist));
    
    OVF_INFO() << "Contour signature: max=" << max_dist 
               << ", min=" << min_dist << ", mean=" << mean_dist;
    
    return Result<void>::success();
}

// ==================== 节点注册 ====================

OVF_REGISTER_NODE(HuMomentsNode, "HuMoments", HuMomentsNode::make_info())
OVF_REGISTER_NODE(ShapeMomentsNode, "ShapeMoments", ShapeMomentsNode::make_info())
OVF_REGISTER_NODE(FlusserMomentsNode, "FlusserMoments", FlusserMomentsNode::make_info())
OVF_REGISTER_NODE(MomentInvariantsNode, "MomentInvariants", MomentInvariantsNode::make_info())

OVF_REGISTER_NODE(ShapeFeaturesNode, "ShapeFeatures", ShapeFeaturesNode::make_info())
OVF_REGISTER_NODE(CompactnessNode, "Compactness", CompactnessNode::make_info())
OVF_REGISTER_NODE(ConvexityNode, "Convexity", ConvexityNode::make_info())
OVF_REGISTER_NODE(AnisometryNode, "Anisometry", AnisometryNode::make_info())

OVF_REGISTER_NODE(FourierDescriptorNode, "FourierDescriptor", FourierDescriptorNode::make_info())
OVF_REGISTER_NODE(ShapeContextNode, "ShapeContext", ShapeContextNode::make_info())
OVF_REGISTER_NODE(CurvatureDescriptorNode, "CurvatureDescriptor", CurvatureDescriptorNode::make_info())
OVF_REGISTER_NODE(ContourSignatureNode, "ContourSignature", ContourSignatureNode::make_info())

} // namespace algorithm
} // namespace ovf