/**
 * @file hand_eye_calibration.h
 * @brief 手眼标定和联合标定模块
 * @author OpenVisionFlow Team
 * @version 0.1.0
 */

#pragma once

#include "ovf/core/types.h"
#include "ovf/core/node.h"
#include "ovf/core/data.h"
#include <vector>
#include <cmath>

namespace ovf {
namespace algorithm {

// ============================================================================
// 数据结构定义
// ============================================================================

/**
 * @brief 机器人位姿结构（TCP位姿）
 */
struct RobotPose {
    // 位置（mm）
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    // 姿态（角度，欧拉角或四元数）
    double rx = 0.0;  // 绕X轴旋转（度）
    double ry = 0.0;  // 绕Y轴旋转（度）
    double rz = 0.0;  // 绕Z轴旋转（度）
    
    // 旋转矩阵（3x3）
    float R[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
    
    bool valid = false;
    
    RobotPose() = default;
    
    RobotPose(double x_, double y_, double z_, double rx_, double ry_, double rz_)
        : x(x_), y(y_), z(z_), rx(rx_), ry(ry_), rz(rz_), valid(true) {}
};

/**
 * @brief 手眼标定结果结构
 */
struct HandEyeResult {
    // 手眼变换矩阵（相机到机器人末端或基座）
    // 4x4变换矩阵
    float transform[4][4];
    
    // 旋转部分（3x3）
    float R[3][3];
    
    // 平移部分（3x1）
    float T[3];
    
    // 标定误差
    float rotation_error = 0.0f;  // 旋转误差（度）
    float translation_error = 0.0f;  // 平移误差（mm）
    float reprojection_error = 0.0f;  // 重投影误差
    
    // 标定质量评估
    float quality_score = 0.0f;  // 0-1分数
    bool valid = false;
    
    HandEyeResult() {
        // 初始化为单位矩阵
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                transform[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R[i][j] = (i == j) ? 1.0f : 0.0f;
            }
            T[i] = 0.0f;
        }
    }
};

/**
 * @brief 多相机联合标定结果
 */
struct MultiCameraResult {
    // 相机数量
    int num_cameras = 0;
    
    // 各相机外参（相对于参考相机或世界坐标系）
    Vector<Transform3D> extrinsics;
    
    // 相机之间的相对位姿
    Vector<Transform3D> relative_transforms;
    
    // 标定误差
    float reprojection_error = 0.0f;
    float max_error = 0.0f;
    
    // 各相机重投影误差
    Vector<float> camera_errors;
    
    bool valid = false;
};

/**
 * @brief 标定配置参数
 */
struct CalibrationConfig {
    // 标定模式
    enum class Mode {
        EyeInHand,     // 相机安装在机器人末端（眼在手上）
        EyeToHand      // 相机固定在外部（眼在手外）
    };
    
    Mode mode = Mode::EyeInHand;
    
    // 标定姿态数量（至少15个）
    int num_poses = 15;
    
    // 优化方法
    enum class OptimizationMethod {
        Tsai,          // Tsai-Lenz方法
        Lenz,          // Lenz方法
        Daniilidis,    // Daniilidis方法（使用四元数）
        Park           // Park方法
    };
    
    OptimizationMethod optimization_method = OptimizationMethod::Tsai;
    
    // 标定板参数
    int pattern_width = 9;        // 标定板宽度（角点数）
    int pattern_height = 6;       // 标定板高度（角点数）
    float square_size = 25.0f;    // 方格尺寸（mm）
    
    // 误差阈值
    float max_rotation_error = 2.0f;    // 最大旋转误差（度）
    float max_translation_error = 5.0f; // 最大平移误差（mm）
    
    // 是否启用优化
    bool enable_refinement = true;
    
    CalibrationConfig() = default;
};

// ============================================================================
// 手眼标定工具函数命名空间
// ============================================================================

namespace hand_eye_utils {

/**
 * @brief 欧拉角转旋转矩阵（RPY顺序）
 * @param rx 绕X轴旋转角度（度）
 * @param ry 绕Y轴旋转角度（度）
 * @param rz 绕Z轴旋转角度（度）
 * @param R 输出旋转矩阵（3x3）
 */
void euler_to_rotation_matrix(double rx, double ry, double rz, float R[3][3]);

/**
 * @brief 旋转矩阵转欧拉角（RPY顺序）
 * @param R 输入旋转矩阵（3x3）
 * @param rx 输出绕X轴旋转角度（度）
 * @param ry 输出绕Y轴旋转角度（度）
 * @param rz 输出绕Z轴旋转角度（度）
 */
void rotation_matrix_to_euler(const float R[3][3], double& rx, double& ry, double& rz);

/**
 * @brief 四元数转旋转矩阵
 * @param qw, qx, qy, qz 四元数分量
 * @param R 输出旋转矩阵（3x3）
 */
void quaternion_to_rotation_matrix(float qw, float qx, float qy, float qz, float R[3][3]);

/**
 * * @brief 旋转矩阵转四元数
 * @param R 输入旋转矩阵（3x3）
 * @param qw, qx, qy, qz 输出四元数分量
 */
void rotation_matrix_to_quaternion(const float R[3][3], float& qw, float& qx, float& qy, float& qz);

/**
 * @brief 构造4x4变换矩阵
 * @param R 旋转矩阵（3x3）
 * @param T 平移向量（3x1）
 * @param transform 输出变换矩阵（4x4）
 */
void build_transform_matrix(const float R[3][3], const float T[3], float transform[4][4]);

/**
 * @brief 从变换矩阵提取旋转和平移
 * @param transform 输入变换矩阵（4x4）
 * @param R 输出旋转矩阵（3x3）
 * @param T 输出平移向量（3x1）
 */
void extract_from_transform_matrix(const float transform[4][4], float R[3][3], float T[3]);

/**
 * @brief 矩阵乘法（4x4）
 * @param A 第一个矩阵
 * @param B 第二个矩阵
 * @param C 输出矩阵 C = A * B
 */
void multiply_transform(const float A[4][4], const float B[4][4], float C[4][4]);

/**
 * @brief 矩阵求逆（4x4变换矩阵）
 * @param transform 输入矩阵
 * @param inverse 输出逆矩阵
 */
void invert_transform(const float transform[4][4], float inverse[4][4]);

/**
 * @brief Tsai-Lenz手眼标定算法
 * @param robot_poses 机器人末端位姿列表（相对于基座）
 * @param camera_poses 相机观测到的标定板位姿列表（相对于相机）
 * @param result 手眼标定结果
 * @param mode 标定模式（eye-in-hand或eye-to-hand）
 * @return 错误码
 */
ErrorCode tsai_lenz_calibration(const Vector<RobotPose>& robot_poses,
                                const Vector<RobotPose>& camera_poses,
                                HandEyeResult& result,
                                CalibrationConfig::Mode mode);

/**
 * * @brief Daniilidis手眼标定算法（使用四元数）
 * @param robot_poses 机器人末端位姿列表
 * @param camera_poses 相机观测到的标定板位姿列表
 * @param result 手眼标定结果
 * @param mode 标定模式
 * @return 错误码
 */
ErrorCode daniilidis_calibration(const Vector<RobotPose>& robot_poses,
                                 const Vector<RobotPose>& camera_poses,
                                 HandEyeResult& result,
                                 CalibrationConfig::Mode mode);

/**
 * @brief Park手眼标定算法
 * @param robot_poses 机器人末端位姿列表
 * @param camera_poses 相机观测到的标定板位姿列表
 * @param result 手眼标定结果
 * @param mode 标定模式
 * @return 错误码
 */
ErrorCode park_calibration(const Vector<RobotPose>& robot_poses,
                           const Vector<RobotPose>& camera_poses,
                           HandEyeResult& result,
                           CalibrationConfig::Mode mode);

/**
 * @brief SVD分解求解旋转矩阵
 * @param A 矩阵A（用于AX=XB问题）
 * @param X 输出旋转矩阵X
 * @return 错误码
 */
ErrorCode solve_rotation_svd(const Vector<float>& A, float X[3][3]);

/**
 * @brief 最小二乘求解平移向量
 * @param A 矩阵A
 * @param b 向量b
 * @param x 输出平移向量
 * @return 错误码
 */
ErrorCode solve_translation_ls(const Vector<Vector<float>>& A,
                               const Vector<float>& b,
                               float x[3]);

/**
 * @brief 计算旋转误差（两个旋转矩阵之间的角度差）
 * @param R1 第一个旋转矩阵
 * @param R2 第二个旋转矩阵
 * @return 旋转误差（度）
 */
float compute_rotation_error(const float R1[3][3], const float R2[3][3]);

/**
 * @brief 计算平移误差
 * @param T1 第一个平移向量
 * @param T2 第二个平移向量
 * @return 平移误差（mm）
 */
float compute_translation_error(const float T1[3], const float T2[3]);

/**
 * @brief 验证手眼标定结果
 * @param robot_poses 机器人位姿列表
 * @param camera_poses 相机位姿列表
 * @param result 手眼标定结果
 * @param mode 标定模式
 * @param rotation_error 输出旋转误差
 * @param translation_error 输出平移误差
 * @return 错误码
 */
ErrorCode verify_hand_eye(const Vector<RobotPose>& robot_poses,
                          const Vector<RobotPose>& camera_poses,
                          const HandEyeResult& result,
                          CalibrationConfig::Mode mode,
                          float& rotation_error,
                          float& translation_error);

/**
 * @brief 应用手眼变换计算目标位姿
 * @param robot_pose 当前机器人位姿
 * @param object_pose 相机观测到的物体位姿
 * @param hand_eye 手眼标定结果
 * @param mode 标定模式
 * @param target_pose 输出目标位姿（机器人需要移动到的位姿）
 * @return 错误码
 */
ErrorCode apply_hand_eye_transform(const RobotPose& robot_pose,
                                   const RobotPose& object_pose,
                                   const HandEyeResult& hand_eye,
                                   CalibrationConfig::Mode mode,
                                   RobotPose& target_pose);

/**
 * @brief 计算抓取姿态（物体位姿到工具姿态）
 * @param object_pose 物体位姿（相对于基座）
 * @param gripper_offset 报手相对于TCP的偏移
 * @param gripper_pose 输出报手姿态
 * @return 错误码
 */
ErrorCode compute_gripper_pose(const RobotPose& object_pose,
                               const Transform3D& gripper_offset,
                               RobotPose& gripper_pose);

} // namespace hand_eye_utils

// ============================================================================
// 多相机标定工具函数命名空间
// ============================================================================

namespace multi_camera_utils {

/**
 * @brief 多相机联合标定
 * @param camera_images 各相机图像列表
 * @param corner_points 各相机检测到的角点列表
 * @param pattern_width 标定板宽度
 * @param pattern_height 标定板高度
 * @param square_size 方格尺寸
 * @param result 多相机标定结果
 * @return 错误码
 */
ErrorCode multi_camera_calibrate(const Vector<Vector<ImageData>>& camera_images,
                                 const Vector<Vector<Vector<Point2D<float>>>>& corner_points,
                                 int pattern_width, int pattern_height,
                                 float square_size,
                                 MultiCameraResult& result);

/**
 * @brief 计算相机之间的相对位姿
 * @param extrinsics1 第一个相机的外参
 * @param extrinsics2 第二个相机的外参
 * @param relative 输出相对位姿（从相机1到相机2）
 * @return 错误码
 */
ErrorCode compute_relative_pose(const Transform3D& extrinsics1,
                                const Transform3D& extrinsics2,
                                Transform3D& relative);

/**
 * @brief 联合优化多相机外参
 * @param initial_extrinsics 初始外参列表
 * @param observations 观测数据（各相机看到的标定板位姿）
 * @param optimized_extrinsics 输出优化后的外参
 * @return 错误码
 */
ErrorCode optimize_extrinsics_joint(const Vector<Transform3D>& initial_extrinsics,
                                    const Vector<Vector<RobotPose>>& observations,
                                    Vector<Transform3D>& optimized_extrinsics);

/**
 * @brief 计算重投影误差
 * @param extrinsics 相机外参
 * @param object_points 物体点（世界坐标）
 * @param image_points 图像点
 * @param intrinsics 相机内参
 * @return 重投影误差
 */
float compute_reprojection_error(const Transform3D& extrinsics,
                                 const Vector<Point3Df>& object_points,
                                 const Vector<Point2D<float>>& image_points,
                                 const float intrinsics[4]);

/**
 * @brief 标定结果精细化（非线性优化）
 * @param result 多相机标定结果
 * @param max_iterations 最大迭代次数
 * @param convergence_threshold 收敛阈值
 * @return 错误码
 */
ErrorCode refine_calibration(MultiCameraResult& result,
                            int max_iterations = 100,
                            float convergence_threshold = 1e-6f);

/**
 * @brief 验证多相机标定结果
 * @param result 多相机标定结果
 * @param test_images 测试图像
 * @param test_corners 测试角点
 * @param avg_error 输出平均误差
 * @return 错误码
 */
ErrorCode verify_multi_camera(const MultiCameraResult& result,
                              const Vector<Vector<ImageData>>& test_images,
                              const Vector<Vector<Vector<Point2D<float>>>>& test_corners,
                              float& avg_error);

} // namespace multi_camera_utils

// ============================================================================
// 手眼标定节点定义（6个）
// ============================================================================

/**
 * @brief 手眼标定核心节点
 */
class HandEyeCalibrationNode : public INode {
public:
    HandEyeCalibrationNode(const String& instance_id);
    ~HandEyeCalibrationNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    const HandEyeResult& get_result() const { return result_; }
    
private:
    CalibrationConfig config_;
    Vector<RobotPose> robot_poses_;
    Vector<RobotPose> camera_poses_;
    HandEyeResult result_;
};

/**
 * @brief 手眼姿态估计节点
 */
class HandEyePoseEstimateNode : public INode {
public:
    HandEyePoseEstimateNode(const String& instance_id);
    ~HandEyePoseEstimateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    RobotPose estimated_pose_;
};

/**
 * @brief 机器人姿态转换节点
 */
class RobotPoseConvertNode : public INode {
public:
    RobotPoseConvertNode(const String& instance_id);
    ~RobotPoseConvertNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    RobotPose converted_pose_;
    enum class ConvertType {
        EulerToQuaternion,
        QuaternionToEuler,
        EulerToMatrix,
        MatrixToEuler,
        PoseToTransform,
        TransformToPose
    };
    ConvertType convert_type_ = ConvertType::EulerToMatrix;
};

/**
 * @brief 手眼标定验证节点
 */
class HandEyeVerifyNode : public INode {
public:
    HandEyeVerifyNode(const String& instance_id);
    ~HandEyeVerifyNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    float rotation_error_ = 0.0f;
    float translation_error_ = 0.0f;
    bool is_valid_ = false;
};

/**
 * @brief 手眼补偿计算节点
 */
class HandEyeCompensateNode : public INode {
public:
    HandEyeCompensateNode(const String& instance_id);
    ~HandEyeCompensateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    RobotPose compensated_pose_;
    float offset_x_ = 0.0f;
    float offset_y_ = 0.0f;
    float offset_z_ = 0.0f;
};

/**
 * @brief 报手姿态计算节点
 */
class GripperPoseCalcNode : public INode {
public:
    GripperPoseCalcNode(const String& instance_id);
    ~GripperPoseCalcNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    RobotPose gripper_pose_;
    Transform3D gripper_offset_;
};

// ============================================================================
// 联合标定节点定义（4个）
// ============================================================================

/**
 * @brief 多相机联合标定节点
 */
class MultiCameraCalibrateNode : public INode {
public:
    MultiCameraCalibrateNode(const String& instance_id);
    ~MultiCameraCalibrateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
    const MultiCameraResult& get_result() const { return result_; }
    
private:
    int num_cameras_ = 2;
    MultiCameraResult result_;
};

/**
 * @brief 双目立体标定节点（扩展版）
 */
class StereoCalibrateNode : public INode {
public:
    StereoCalibrateNode(const String& instance_id);
    ~StereoCalibrateNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    int pattern_width_ = 9;
    int pattern_height_ = 6;
    float square_size_ = 25.0f;
    MultiCameraResult result_;
};

/**
 * @brief 相机阵列标定节点
 */
class CalibrateCamerasNode : public INode {
public:
    CalibrateCamerasNode(const String& instance_id);
    ~CalibrateCamerasNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    int num_cameras_ = 4;
    MultiCameraResult result_;
};

/**
 * @brief 标定结果优化节点
 */
class CalibrationRefineNode : public INode {
public:
    CalibrationRefineNode(const String& instance_id);
    ~CalibrationRefineNode() override = default;
    
    Result<void> execute(FlowContext& context) override;
    static NodeInfo make_info();
    
private:
    int max_iterations_ = 100;
    float convergence_threshold_ = 1e-6f;
    MultiCameraResult refined_result_;
};

} // namespace algorithm
} // namespace ovf