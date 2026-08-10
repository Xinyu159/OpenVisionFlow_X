/**
 * @file test_core_types.cpp
 * @brief OpenVisionFlow 核心类型测试
 * 
 * 测试内容：
 * 1. ImageData创建/赋值/格式转换
 * 2. Data容器类型转换
 * 3. Point/Region/Pose结构
 * 4. ErrorCode错误处理
 * 5. Result结果类型
 */

#include "test_framework.h"
#include "ovf/core/types.h"
#include "ovf/core/data.h"
#include "ovf/core/error.h"

using namespace ovf;
using namespace ovf_test;

// ============================================================================
// ImageData 测试
// ============================================================================

TEST(CoreTypes, ImageData_CreateEmpty) {
    ImageData img;
    ASSERT_TRUE(img.empty());
    ASSERT_EQ(0u, img.width);
    ASSERT_EQ(0u, img.height);
    ASSERT_EQ(1u, img.channels);  // 默认值为1，不是0
    ASSERT_EQ(0u, img.size());
}

TEST(CoreTypes, ImageData_CreateMono8) {
    ImageData img;
    img.width = 100;
    img.height = 50;
    img.channels = 1;
    img.format = ImageFormat::Mono8;
    img.data.resize(100 * 50, 128);
    
    ASSERT_FALSE(img.empty());
    ASSERT_EQ(100u, img.width);
    ASSERT_EQ(50u, img.height);
    ASSERT_EQ(1u, img.channels);
    ASSERT_EQ(5000u, img.size());
    ASSERT_EQ(ImageFormat::Mono8, img.format);
}

TEST(CoreTypes, ImageData_CreateRGB8) {
    ImageData img;
    img.width = 64;
    img.height = 64;
    img.channels = 3;
    img.format = ImageFormat::RGB8;
    img.data.resize(64 * 64 * 3);
    
    ASSERT_FALSE(img.empty());
    ASSERT_EQ(64u, img.width);
    ASSERT_EQ(64u, img.height);
    ASSERT_EQ(3u, img.channels);
    ASSERT_EQ(12288u, img.size());
}

TEST(CoreTypes, ImageData_CreateMono16) {
    ImageData img;
    img.width = 100;
    img.height = 100;
    img.channels = 1;
    img.format = ImageFormat::Mono16;
    img.data.resize(100 * 100 * 2);
    
    ASSERT_EQ(ImageFormat::Mono16, img.format);
    ASSERT_EQ(20000u, img.size());
}

TEST(CoreTypes, ImageData_Timestamp) {
    ImageData img;
    img.timestamp = 1234567890;
    img.frame_id = 100;
    img.source_id = "camera_01";
    
    ASSERT_EQ(1234567890u, img.timestamp);
    ASSERT_EQ(100u, img.frame_id);
    ASSERT_EQ("camera_01", img.source_id);
}

// ============================================================================
// Data 容器测试
// ============================================================================

TEST(CoreTypes, Data_CreateNone) {
    Data data;
    ASSERT_TRUE(data.is_none());
    ASSERT_FALSE(data.is_valid());
    ASSERT_EQ(DataType::None, data.type());
}

TEST(CoreTypes, Data_CreateNumber) {
    Data data1(42);
    ASSERT_TRUE(data1.is_number());
    ASSERT_EQ(DataType::Number, data1.type());
    ASSERT_EQ(42.0, data1.as_number());
    ASSERT_EQ(42, data1.as_int());
    
    Data data2(3.14159f);
    ASSERT_TRUE(data2.is_number());
    ASSERT_NEAR(3.14159, data2.as_number(), 0.00001);
    
    Data data3(-100);
    ASSERT_EQ(-100, data3.as_int());
}

TEST(CoreTypes, Data_CreateBool) {
    Data data_true(true);
    ASSERT_TRUE(data_true.is_bool());
    ASSERT_TRUE(data_true.as_bool());
    
    Data data_false(false);
    ASSERT_TRUE(data_false.is_bool());
    ASSERT_FALSE(data_false.as_bool());
}

TEST(CoreTypes, Data_CreateString) {
    Data data("Hello World");
    ASSERT_TRUE(data.is_string());
    ASSERT_EQ("Hello World", data.as_string());
    
    Data data2(String("测试汉字"));
    ASSERT_EQ("测试汉字", data2.as_string());
}

TEST(CoreTypes, Data_CreateImage) {
    ImageData img;
    img.width = 10;
    img.height = 10;
    img.channels = 1;
    img.format = ImageFormat::Mono8;
    img.data.resize(100, 255);
    
    Data data(img);
    ASSERT_TRUE(data.is_image());
    ASSERT_EQ(DataType::Image, data.type());
    
    const ImageData& ref = data.as_image();
    ASSERT_EQ(10u, ref.width);
    ASSERT_EQ(10u, ref.height);
    ASSERT_EQ(255, ref.data[0]);
}

TEST(CoreTypes, Data_CreateRegion) {
    Region region(10, 20, 100, 50, 0.0f);
    Data data(region);
    
    ASSERT_TRUE(data.is_region());
    ASSERT_EQ(DataType::Region, data.type());
    
    const Region& ref = data.as_region();
    ASSERT_EQ(10, ref.x);
    ASSERT_EQ(20, ref.y);
    ASSERT_EQ(100, ref.width);
    ASSERT_EQ(50, ref.height);
}

TEST(CoreTypes, Data_CreatePose) {
    Pose pose;
    pose.x = 100.0;
    pose.y = 200.0;
    pose.z = 50.0;
    pose.rx = 0.0;
    pose.ry = 45.0;
    pose.rz = 90.0;
    
    Data data(pose);
    ASSERT_TRUE(data.is_pose());
    
    const Pose& ref = data.as_pose();
    ASSERT_NEAR(100.0, ref.x, 0.001);
    ASSERT_NEAR(200.0, ref.y, 0.001);
    ASSERT_NEAR(50.0, ref.z, 0.001);
}

TEST(CoreTypes, Data_CreatePoint3D) {
    Point3Df point(1.5f, 2.5f, 3.5f);
    Data data(point);
    
    ASSERT_TRUE(data.is_point());
    
    const Point3Df& ref = data.as_point();
    ASSERT_NEAR(1.5f, ref.x, 0.001f);
    ASSERT_NEAR(2.5f, ref.y, 0.001f);
    ASSERT_NEAR(3.5f, ref.z, 0.001f);
}

TEST(CoreTypes, Data_CreatePointCloud) {
    PointCloudData cloud;
    cloud.add_point(0.0f, 0.0f, 0.0f);
    cloud.add_point(1.0f, 1.0f, 1.0f);
    cloud.add_point(2.0f, 2.0f, 2.0f);
    
    Data data(cloud);
    ASSERT_TRUE(data.is_pointcloud());
    ASSERT_EQ(DataType::PointCloud, data.type());
    
    const PointCloudData& ref = data.as_pointcloud();
    ASSERT_EQ(3u, ref.size());
    ASSERT_FALSE(ref.empty());
}

TEST(CoreTypes, Data_ToString) {
    Data data_none;
    ASSERT_EQ("None", data_none.to_string());
    
    Data data_num(123.456);
    std::string num_str = data_num.to_string();
    ASSERT_TRUE(num_str.find("123") != std::string::npos);
    
    Data data_bool(true);
    ASSERT_EQ("true", data_bool.to_string());
    
    Data data_str("hello");
    ASSERT_EQ("hello", data_str.to_string());
}

TEST(CoreTypes, Data_Clone) {
    Data original(100.0);
    Data cloned = original.clone();
    
    ASSERT_TRUE(cloned.is_number());
    ASSERT_EQ(100.0, cloned.as_number());
    
    // 修改克隆不应影响原始
    // 注意：克隆是独立的对象，修改克隆不会影响原始对象
    // 这里的示例仅用于说明克隆的概念
}

TEST(CoreTypes, Data_DefaultValues) {
    Data data;
    ASSERT_EQ(0.0, data.as_number(0.0));
    ASSERT_EQ(-1.0, data.as_number(-1.0));
    ASSERT_EQ(0, data.as_int(0));
    ASSERT_EQ(-999, data.as_int(-999));
    ASSERT_EQ("", data.as_string(""));
    ASSERT_FALSE(data.as_bool(false));
}

// ============================================================================
// Point 结构测试
// ============================================================================

TEST(CoreTypes, Point2D_Create) {
    Point2D<int> p1(10, 20);
    ASSERT_EQ(10, p1.x);
    ASSERT_EQ(20, p1.y);
    
    Point2D<float> p2(1.5f, 2.5f);
    ASSERT_NEAR(1.5f, p2.x, 0.001f);
    ASSERT_NEAR(2.5f, p2.y, 0.001f);
    
    Point2D<double> p3(100.5, 200.5);
    ASSERT_NEAR(100.5, p3.x, 0.001);
    ASSERT_NEAR(200.5, p3.y, 0.001);
}

TEST(CoreTypes, Point3D_Create) {
    Point3Df p(1.0f, 2.0f, 3.0f);
    ASSERT_NEAR(1.0f, p.x, 0.001f);
    ASSERT_NEAR(2.0f, p.y, 0.001f);
    ASSERT_NEAR(3.0f, p.z, 0.001f);
}

TEST(CoreTypes, Point3D_Distance) {
    Point3Df p1(0.0f, 0.0f, 0.0f);
    Point3Df p2(1.0f, 0.0f, 0.0f);
    ASSERT_NEAR(1.0f, p1.distance_to(p2), 0.001f);
    
    Point3Df p3(0.0f, 0.0f, 0.0f);
    Point3Df p4(1.0f, 1.0f, 1.0f);
    ASSERT_NEAR(1.73205f, p3.distance_to(p4), 0.001f); // sqrt(3)
}

TEST(CoreTypes, Point3D_Operations) {
    Point3Df p1(1.0f, 2.0f, 3.0f);
    Point3Df p2(4.0f, 5.0f, 6.0f);
    
    // 加法
    Point3Df sum = p1 + p2;
    ASSERT_NEAR(5.0f, sum.x, 0.001f);
    ASSERT_NEAR(7.0f, sum.y, 0.001f);
    ASSERT_NEAR(9.0f, sum.z, 0.001f);
    
    // 减法
    Point3Df diff = p2 - p1;
    ASSERT_NEAR(3.0f, diff.x, 0.001f);
    ASSERT_NEAR(3.0f, diff.y, 0.001f);
    ASSERT_NEAR(3.0f, diff.z, 0.001f);
    
    // 数乘
    Point3Df scaled = p1 * 2.0f;
    ASSERT_NEAR(2.0f, scaled.x, 0.001f);
    ASSERT_NEAR(4.0f, scaled.y, 0.001f);
    ASSERT_NEAR(6.0f, scaled.z, 0.001f);
}

TEST(CoreTypes, Point3D_VectorOps) {
    Point3Df p1(1.0f, 2.0f, 3.0f);
    Point3Df p2(4.0f, 5.0f, 6.0f);
    
    // 点积
    float dot = p1.dot(p2);
    ASSERT_NEAR(32.0f, dot, 0.001f); // 1*4 + 2*5 + 3*6 = 32
    
    // 长度
    Point3Df p3(3.0f, 4.0f, 0.0f);
    ASSERT_NEAR(5.0f, p3.length(), 0.001f); // sqrt(9+16) = 5
    
    // 归一化
    Point3Df norm = p3.normalized();
    ASSERT_NEAR(1.0f, norm.length(), 0.001f);
}

TEST(CoreTypes, Point3D_CrossProduct) {
    Point3Df p1(1.0f, 0.0f, 0.0f);
    Point3Df p2(0.0f, 1.0f, 0.0f);
    
    Point3Df cross = p1.cross(p2);
    ASSERT_NEAR(0.0f, cross.x, 0.001f);
    ASSERT_NEAR(0.0f, cross.y, 0.001f);
    ASSERT_NEAR(1.0f, cross.z, 0.001f); // Z方向
}

// ============================================================================
// Region 结构测试
// ============================================================================

TEST(CoreTypes, Region_Create) {
    Region r(10, 20, 100, 50);
    ASSERT_EQ(10, r.x);
    ASSERT_EQ(20, r.y);
    ASSERT_EQ(100, r.width);
    ASSERT_EQ(50, r.height);
    ASSERT_NEAR(0.0f, r.angle, 0.001f);
}

TEST(CoreTypes, Region_WithAngle) {
    Region r(10, 20, 100, 50, 45.0f);
    ASSERT_NEAR(45.0f, r.angle, 0.001f);
}

// ============================================================================
// Pose 结构测试
// ============================================================================

TEST(CoreTypes, Pose_Create) {
    Pose pose;
    pose.x = 100.0;
    pose.y = 200.0;
    pose.z = 50.0;
    pose.rx = 0.0;
    pose.ry = 45.0;
    pose.rz = 90.0;
    
    ASSERT_NEAR(100.0, pose.x, 0.001);
    ASSERT_NEAR(200.0, pose.y, 0.001);
    ASSERT_NEAR(50.0, pose.z, 0.001);
    ASSERT_NEAR(0.0, pose.rx, 0.001);
    ASSERT_NEAR(45.0, pose.ry, 0.001);
    ASSERT_NEAR(90.0, pose.rz, 0.001);
}

// ============================================================================
// PointCloudData 结构测试
// ============================================================================

TEST(CoreTypes, PointCloud_Empty) {
    PointCloudData cloud;
    ASSERT_TRUE(cloud.empty());
    ASSERT_EQ(0u, cloud.size());
}

TEST(CoreTypes, PointCloud_AddPoints) {
    PointCloudData cloud;
    cloud.add_point(1.0f, 2.0f, 3.0f);
    cloud.add_point(Point3Df(4.0f, 5.0f, 6.0f));
    
    ASSERT_FALSE(cloud.empty());
    ASSERT_EQ(2u, cloud.size());
}

TEST(CoreTypes, PointCloud_Clear) {
    PointCloudData cloud;
    cloud.add_point(1.0f, 2.0f, 3.0f);
    cloud.clear();
    
    ASSERT_TRUE(cloud.empty());
    ASSERT_EQ(0u, cloud.size());
}

TEST(CoreTypes, PointCloud_Intensities) {
    PointCloudData cloud;
    cloud.add_point(1.0f, 2.0f, 3.0f);
    
    ASSERT_FALSE(cloud.has_intensities()); // 未添加强度数据
    
    cloud.intensities.resize(1, 255);
    ASSERT_TRUE(cloud.has_intensities());
}

TEST(CoreTypes, PointCloud_Colors) {
    PointCloudData cloud;
    cloud.add_point(1.0f, 2.0f, 3.0f);
    
    ASSERT_FALSE(cloud.has_colors());
    
    cloud.colors.push_back(ColorRGB(255, 0, 0));
    ASSERT_TRUE(cloud.has_colors());
}

TEST(CoreTypes, PointCloud_Organized) {
    PointCloudData cloud;
    cloud.width = 10;
    cloud.height = 10;
    cloud.is_organized = true;
    
    ASSERT_TRUE(cloud.is_organized);
    ASSERT_EQ(10, cloud.width);
    ASSERT_EQ(10, cloud.height);
}

// ============================================================================
// DepthImageData 结构测试
// ============================================================================

TEST(CoreTypes, DepthImage_Create) {
    DepthImageData depth_data;
    
    depth_data.depth.width = 100;
    depth_data.depth.height = 100;
    depth_data.depth.format = ImageFormat::Mono16;
    depth_data.depth.data.resize(100 * 100 * 2);
    
    ASSERT_FALSE(depth_data.empty());
    ASSERT_FALSE(depth_data.has_color());
}

TEST(CoreTypes, DepthImage_WithColor) {
    DepthImageData depth_data;
    
    depth_data.depth.width = 100;
    depth_data.depth.height = 100;
    depth_data.depth.data.resize(100 * 100);
    
    depth_data.color.width = 100;
    depth_data.color.height = 100;
    depth_data.color.channels = 3;
    depth_data.color.data.resize(100 * 100 * 3);
    
    ASSERT_TRUE(depth_data.has_color());
}

TEST(CoreTypes, DepthImage_Parameters) {
    DepthImageData depth_data;
    depth_data.depth_scale = 0.001f;
    depth_data.depth_offset = 100.0f;
    depth_data.focal_length_x = 500.0f;
    depth_data.focal_length_y = 500.0f;
    depth_data.center_x = 320.0f;
    depth_data.center_y = 240.0f;
    
    ASSERT_NEAR(0.001f, depth_data.depth_scale, 0.0001f);
    ASSERT_NEAR(100.0f, depth_data.depth_offset, 0.1f);
}

// ============================================================================
// ErrorCode 测试
// ============================================================================

TEST(CoreTypes, ErrorCode_Basic) {
    ErrorCode code1 = ErrorCode::Success;
    ASSERT_EQ(static_cast<int>(ErrorCode::Success), static_cast<int>(code1));
    
    ErrorCode code2 = ErrorCode::InvalidParameter;
    ASSERT_EQ(2, static_cast<int>(code2));
    
    ErrorCode code3 = ErrorCode::AlgorithmExecFailed;
    ASSERT_EQ(301, static_cast<int>(code3));
}

TEST(CoreTypes, ErrorCode_Ranges) {
    // 通用错误范围 (1-99)
    int unknown = static_cast<int>(ErrorCode::Unknown);
    int ioError = static_cast<int>(ErrorCode::IOError);
    ASSERT_LE(99, unknown);  // unknown <= 99
    ASSERT_LE(99, ioError);  // ioError <= 99
    ASSERT_GE(1, unknown);   // unknown >= 1
    ASSERT_GE(1, ioError);   // ioError >= 1

    // 流程引擎错误范围 (100-199)
    int flowNotFound = static_cast<int>(ErrorCode::FlowNotFound);
    int cyclicDependency = static_cast<int>(ErrorCode::CyclicDependency);
    ASSERT_GE(100, flowNotFound);   // flowNotFound >= 100
    ASSERT_LE(199, cyclicDependency); // cyclicDependency <= 199

    // 算法错误范围 (300-399)
    int algoInitFailed = static_cast<int>(ErrorCode::AlgorithmInitFailed);
    int calibrationFailed = static_cast<int>(ErrorCode::CalibrationFailed);
    ASSERT_GE(300, algoInitFailed);    // algoInitFailed >= 300
    ASSERT_LE(399, calibrationFailed); // calibrationFailed <= 399
}

// ============================================================================
// Result<T> 测试
// ============================================================================

TEST(CoreTypes, Result_Success) {
    Result<int> result = Result<int>::success(42);
    ASSERT_TRUE(result.is_success());
    ASSERT_FALSE(result.is_failure());
    ASSERT_TRUE(static_cast<bool>(result));
    ASSERT_EQ(42, result.value());
    ASSERT_EQ(ErrorCode::Success, result.code());
}

TEST(CoreTypes, Result_Failure) {
    Result<int> result = Result<int>::failure(ErrorCode::InvalidParameter, "Invalid value");
    ASSERT_FALSE(result.is_success());
    ASSERT_TRUE(result.is_failure());
    ASSERT_EQ(ErrorCode::InvalidParameter, result.code());
    ASSERT_EQ("Invalid value", result.message());
}

TEST(CoreTypes, Result_Void_Success) {
    Result<void> result = Result<void>::success();
    ASSERT_TRUE(result.is_success());
    ASSERT_TRUE(static_cast<bool>(result));
    ASSERT_EQ(ErrorCode::Success, result.code());
}

TEST(CoreTypes, Result_Void_Failure) {
    Result<void> result = Result<void>::failure(ErrorCode::NotFound, "Not found");
    ASSERT_FALSE(result.is_success());
    ASSERT_EQ(ErrorCode::NotFound, result.code());
}

TEST(CoreTypes, Result_OperatorStar) {
    Result<int> result = Result<int>::success(100);
    ASSERT_EQ(100, *result);
    
    const Result<int>& const_result = result;
    ASSERT_EQ(100, *const_result);
}

TEST(CoreTypes, Result_OperatorArrow) {
    struct TestStruct { int value = 42; };
    TestStruct ts;
    
    Result<TestStruct> result = Result<TestStruct>::success(ts);
    ASSERT_EQ(42, result->value);
}

TEST(CoreTypes, Result_ExceptionOnFailure) {
    Result<int> result = Result<int>::failure(ErrorCode::InvalidData, "Test error");
    
    ASSERT_THROW(result.value(), ovf::Exception);
}

// ============================================================================
// Transform3D 测试
// ============================================================================

TEST(CoreTypes, Transform3D_Identity) {
    Transform3D t;
    
    // 检查单位矩阵
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float expected = (i == j) ? 1.0f : 0.0f;
            ASSERT_NEAR(expected, t.m[i][j], 0.001f);
        }
    }
}

TEST(CoreTypes, Transform3D_Translation) {
    Transform3D t;
    t.set_translation(10.0f, 20.0f, 30.0f);
    
    ASSERT_NEAR(10.0f, t.m[0][3], 0.001f);
    ASSERT_NEAR(20.0f, t.m[1][3], 0.001f);
    ASSERT_NEAR(30.0f, t.m[2][3], 0.001f);
}

TEST(CoreTypes, Transform3D_RotationX) {
    Transform3D t;
    t.set_rotation_x(0.0f); // 0度旋转
    
    ASSERT_NEAR(1.0f, t.m[1][1], 0.001f);
    ASSERT_NEAR(0.0f, t.m[1][2], 0.001f);
}

TEST(CoreTypes, Transform3D_TransformPoint) {
    Transform3D t;
    t.set_translation(100.0f, 200.0f, 300.0f);
    
    Point3Df p(0.0f, 0.0f, 0.0f);
    Point3Df transformed = t.transform(p);
    
    ASSERT_NEAR(100.0f, transformed.x, 0.001f);
    ASSERT_NEAR(200.0f, transformed.y, 0.001f);
    ASSERT_NEAR(300.0f, transformed.z, 0.001f);
}

// ============================================================================
// BoundingBox3D 测试
// ============================================================================

TEST(CoreTypes, BoundingBox3D_Create) {
    BoundingBox3D box;
    box.min_pt = Point3Df(0.0f, 0.0f, 0.0f);
    box.max_pt = Point3Df(10.0f, 20.0f, 30.0f);
    box.update();
    
    ASSERT_NEAR(5.0f, box.center.x, 0.001f);
    ASSERT_NEAR(10.0f, box.center.y, 0.001f);
    ASSERT_NEAR(15.0f, box.center.z, 0.001f);
    
    ASSERT_NEAR(10.0f, box.size.x, 0.001f);
    ASSERT_NEAR(20.0f, box.size.y, 0.001f);
    ASSERT_NEAR(30.0f, box.size.z, 0.001f);
}

// ============================================================================
// Plane3D 测试
// ============================================================================

TEST(CoreTypes, Plane3D_Create) {
    Plane3D plane;
    plane.center = Point3Df(0.0f, 0.0f, 0.0f);
    plane.normal = Point3Df(0.0f, 0.0f, 1.0f); // Z方向法向量
    plane.d = 0.0f;
    plane.valid = true;
    
    ASSERT_TRUE(plane.valid);
    ASSERT_NEAR(1.0f, plane.normal.length(), 0.001f);
}

// ============================================================================
// ColorRGB 测试
// ============================================================================

TEST(CoreTypes, ColorRGB_Create) {
    ColorRGB color(255, 128, 64);
    ASSERT_EQ(255, color.r);
    ASSERT_EQ(128, color.g);
    ASSERT_EQ(64, color.b);
}

TEST(CoreTypes, ColorRGB_Default) {
    ColorRGB color;
    ASSERT_EQ(0, color.r);
    ASSERT_EQ(0, color.g);
    ASSERT_EQ(0, color.b);
}

// ============================================================================
// ParamSet 测试
// ============================================================================

TEST(CoreTypes, ParamSet_SetGet) {
    ParamSet params;
    params.set("value1", Data(100));
    params.set("value2", Data("hello"));
    
    ASSERT_TRUE(params.has("value1"));
    ASSERT_TRUE(params.has("value2"));
    ASSERT_FALSE(params.has("nonexistent"));
    
    ASSERT_EQ(100, params.get_int("value1"));
    ASSERT_EQ("hello", params.get_string("value2"));
}

TEST(CoreTypes, ParamSet_DefaultValues) {
    ParamSet params;
    ASSERT_EQ(0, params.get_int("nonexistent", 0));
    ASSERT_EQ(-1, params.get_int("nonexistent", -1));
    ASSERT_EQ("default", params.get_string("nonexistent", "default"));
}

TEST(CoreTypes, ParamSet_Remove) {
    ParamSet params;
    params.set("key", Data(100));
    ASSERT_TRUE(params.has("key"));
    
    params.remove("key");
    ASSERT_FALSE(params.has("key"));
}

TEST(CoreTypes, ParamSet_Clear) {
    ParamSet params;
    params.set("key1", Data(1));
    params.set("key2", Data(2));
    ASSERT_EQ(2u, params.size());
    
    params.clear();
    ASSERT_EQ(0u, params.size());
}

TEST(CoreTypes, ParamSet_NumericTypes) {
    ParamSet params;
    params.set("int_val", Data(42));
    params.set("float_val", Data(3.14f));
    params.set("double_val", Data(2.71828));
    
    ASSERT_EQ(42, params.get_int("int_val"));
    ASSERT_NEAR(3.14, params.get_number("float_val"), 0.01);
    ASSERT_NEAR(2.71828, params.get_number("double_val"), 0.0001);
}

TEST(CoreTypes, ParamSet_BoolType) {
    ParamSet params;
    params.set("true_val", Data(true));
    params.set("false_val", Data(false));
    
    ASSERT_TRUE(params.get_bool("true_val"));
    ASSERT_FALSE(params.get_bool("false_val"));
    ASSERT_FALSE(params.get_bool("nonexistent", false));
}

// ============================================================================
// DataPort 测试
// ============================================================================

TEST(CoreTypes, DataPort_Create) {
    DataPort port("input1", "Input Image", DataType::Image, true);
    
    ASSERT_EQ("input1", port.id);
    ASSERT_EQ("Input Image", port.name);
    ASSERT_EQ(DataType::Image, port.data_type);
    ASSERT_TRUE(port.required);
}

TEST(CoreTypes, DataPort_Optional) {
    DataPort port("output1", "Output Result", DataType::Number, false);
    
    ASSERT_FALSE(port.required);
}

// ============================================================================
// ParamDef 测试
// ============================================================================

TEST(CoreTypes, ParamDef_Create) {
    ParamDef def("threshold", "Threshold Value", DataType::Number, Data(128));
    
    ASSERT_EQ("threshold", def.id);
    ASSERT_EQ("Threshold Value", def.name);
    ASSERT_EQ(DataType::Number, def.type);
    ASSERT_EQ(128, def.default_value.as_int());
}

// ============================================================================
// 版本信息测试
// ============================================================================

TEST(CoreTypes, VersionInfo) {
    ASSERT_EQ("0.2.0", VERSION);
    ASSERT_EQ(0, VERSION_MAJOR);
    ASSERT_EQ(2, VERSION_MINOR);
    ASSERT_EQ(0, VERSION_PATCH);
}

// ============================================================================
// 类型别名测试
// ============================================================================

TEST(CoreTypes, TypeAliases) {
    String str = "test";
    ASSERT_EQ("test", str);
    
    ByteArray bytes = {0, 1, 2, 3, 4};
    ASSERT_EQ(5u, bytes.size());
    
    Vector<int> vec = {1, 2, 3};
    ASSERT_EQ(3u, vec.size());
    
    HashMap<String, int> map;
    map["key"] = 100;
    ASSERT_EQ(100, map["key"]);
}

// ============================================================================
// 主程序入口
// ============================================================================

int main() {
    // 运行所有测试
    TestStats stats = TestRunner::run_all_tests();
    
    // 保存测试报告
    TestRunner::save_report(stats, "test_core_types_report.json");
    
    // 返回失败测试数量作为退出码
    return stats.failed_tests;
}