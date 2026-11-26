/**
 * @file studio_main.cpp
 * @brief OVF Studio — 桌面工作台（对标 HDevelop 的交互形态，浏览器方案退役）
 *
 * 布局（HDevelop 式）：
 *   - 算子库（左）：NodeFactory 实时列出全部可用算子，双击插入流程
 *   - 程序窗口（左中）：线性步骤列表（增/删/上移/下移/单步/运行到此处）
 *   - 参数窗口（右）：选中步骤的 NodeInfo.params 自动生成控件
 *   - 图形窗口（中央）：上一步的图像输出，滚轮缩放/拖拽平移
 *   - 日志（底）：引擎日志尾随
 *
 * 执行模型：线性链 —— 每步的 image 类输出喂给下一步的 "image" 输入口。
 * DAG 编排是后续版本的事，先把 HDevelop 的"程序列表"手感做对。
 */

#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif

#include "imgui.h"
#include "imgui_internal.h"  // DockBuilder（默认布局）
#include "imgui_impl_win32.h"
#include "imgui_impl_dx11.h"

#include "ovf/core/node.h"
#include "ovf/core/logger.h"
#include "ovf/nodes/nodes.h"

#include <d3d11.h>
#include <tchar.h>
#include <commdlg.h>
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image.h"
#include "stb_image_write.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <string>

// --------------------------------------------------------------------------
// Win32 + D3D11 样板（ImGui 官方示例精简）
// --------------------------------------------------------------------------
static ID3D11Device*           g_device = nullptr;
static ID3D11DeviceContext*    g_ctx = nullptr;
static IDXGISwapChain*         g_swap = nullptr;
static ID3D11RenderTargetView* g_rtv = nullptr;

static void CreateRenderTarget(HWND hwnd) {
    ID3D11Texture2D* back = nullptr;
    g_swap->GetBuffer(0, IID_PPV_ARGS(&back));
    if (back) {
        g_device->CreateRenderTargetView(back, nullptr, &g_rtv);
        back->Release();
    }
    (void)hwnd;
}
static void CleanupRenderTarget() { if (g_rtv) { g_rtv->Release(); g_rtv = nullptr; } }

static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
static bool CreateDeviceD3D(HWND hWnd);
static void CleanupDeviceD3D();

// --------------------------------------------------------------------------
// Studio 状态
// --------------------------------------------------------------------------
using namespace ovf;

struct Step {
    INode::Ptr        node;
    String            type_id;
    bool              enabled = true;
    Result<void>      last_result = Result<void>::success();
};

struct StudioState {
    std::vector<Step>          steps;         // 线性程序
    int                        selected = -1; // 参数面板对应的步骤
    int                        run_to = -1;    // "运行到此处"目标
    ImageData                  image;         // 图形窗口当前图
    std::vector<char>          log_buf;       // 日志环形缓冲
    bool                       demo = false;

    // 图像视图
    float                      zoom = 1.0f;
    ImVec2                     pan{0, 0};

    // 数据集浏览：当前目录的图像清单 + 索引
    std::vector<std::string>   dir_files;
    int                        dir_idx = -1;
    std::string                dir_root;

    // D3D11 纹理（图像显示用）
    ID3D11Texture2D*           tex = nullptr;
    ID3D11ShaderResourceView*  tex_srv = nullptr;
    int                        tex_w = 0, tex_h = 0;
};

static StudioState g_s;

// 引擎日志 → studio 日志面板
class StudioLogSink : public ILogSink {
public:
    void write(LogLevel level, const String& msg, const String&, int,
               const String&) override {
        auto& buf = g_s.log_buf;
        const char* tag = "INFO";
        if (level == LogLevel::Warning) tag = "WARN";
        else if (level == LogLevel::Error) tag = "ERR ";
        const std::string line = std::string("[") + tag + "] " + msg + "\n";
        buf.insert(buf.end(), line.begin(), line.end());
        if (buf.size() > (1 << 20)) buf.erase(buf.begin(), buf.end() - (1 << 19));
    }
};

// --------------------------------------------------------------------------
// 执行：线性链跑 steps[0..run_to]，image 输出逐步下传
// --------------------------------------------------------------------------
static void RunChain(int upto) {
    if (g_s.steps.empty()) return;
    if (upto < 0 || upto >= (int)g_s.steps.size()) upto = (int)g_s.steps.size() - 1;

    Data carry;    // 逐步传递的图像
    Data carry_r;  // 逐步传递的 region（如 OnnxInfer 的检出框 → CropROI）
    for (int i = 0; i <= upto; ++i) {
        Step& st = g_s.steps[i];
        if (!st.enabled) continue;

        // 把上一步的图喂给本步的 image 口（有该输入口才喂）
        if (carry.is_image()) {
            for (const auto& in : st.node->info().inputs) {
                if (in.data_type == DataType::Image) { st.node->set_input(in.id, carry); break; }
            }
        }
        // region 口同理（可选输入，上一步有就喂）
        if (carry_r.is_region()) {
            for (const auto& in : st.node->info().inputs) {
                if (in.data_type == DataType::Region) { st.node->set_input(in.id, carry_r); break; }
            }
        }
        FlowContext ctx;
        st.last_result = st.node->execute(ctx);

        // 找 image 类输出口往下传（优先与输入同名，其次第一个 image 口）
        String pick;
        for (const auto& out : st.node->info().outputs) {
            if (out.data_type != DataType::Image) continue;
            if (out.id == "image" || pick.empty()) pick = out.id;
            if (out.id == "image") break;
        }
        if (!pick.empty()) carry = st.node->get_output(pick);
        else carry = Data();
        String pick_r;
        for (const auto& out : st.node->info().outputs)
            if (out.data_type == DataType::Region) { pick_r = out.id; break; }
        if (!pick_r.empty()) carry_r = st.node->get_output(pick_r);
    }
    if (carry.is_image()) g_s.image = carry.as_image();
}

// 生成测试图（无相机时的画布）：暗底 + 一根竖亮条 + 一根斜亮条
static ImageData MakeTestImage() {
    ImageData img;
    img.width = 320; img.height = 240; img.channels = 1;
    img.format = ImageFormat::Mono8;
    img.data.assign(320 * 240, 30);
    // 一对平行灯条（装甲板的左右灯条，带 6° 倾角，模拟实拍）
    const double tilt = 0.10;
    for (int t = -80; t <= 80; ++t) {
        for (int d = -3; d <= 3; ++d) {
            auto put = [&](int cx) {
                const int x = (int)(cx + t * tilt - d * 1.0);
                const int y = 120 + t;
                if (x >= 0 && x < 320 && y >= 0 && y < 240)
                    img.data[y * 320 + x] = 235;
            };
            put(90); put(230);   // 左右灯条
        }
    }
    return img;
}

static void UploadImage(const ImageData& img);

// 从磁盘加载图像（stb：JPG/PNG/BMP），转 Mono8
static bool LoadImageFile(const std::string& path) {
    int w = 0, h = 0, n = 0;
    unsigned char* px = stbi_load(path.c_str(), &w, &h, &n, 0);
    if (!px || w <= 0 || h <= 0) return false;
    ImageData img;
    img.width = (uint32_t)w; img.height = (uint32_t)h;
    const int ch = (n >= 3 ? 3 : 1);
    img.channels = (uint32_t)ch;
    img.format = (ch == 3) ? ImageFormat::RGB8 : ImageFormat::Mono8;
    img.data.assign((size_t)w * h * ch, 0);
    if (ch == 3) {
        for (size_t i = 0; i < (size_t)w * h; ++i) {
            img.data[i * 3 + 0] = px[i * 3 + 0];
            img.data[i * 3 + 1] = px[i * 3 + 1];
            img.data[i * 3 + 2] = px[i * 3 + 2];
        }
    } else {
        for (size_t i = 0; i < (size_t)w * h; ++i) img.data[i] = px[i];
    }
    stbi_image_free(px);
    g_s.image = std::move(img);
    UploadImage(g_s.image);
    return true;
}

// 扫描目录里的图像（不递归）
static void ScanDir(const std::string& dir) {
    g_s.dir_files.clear();
    g_s.dir_root = dir;
    WIN32_FIND_DATAA fd;
    HANDLE h = FindFirstFileA((dir + "\*").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return;
    do {
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
        std::string n = fd.cFileName;
        auto ends = [&](const char* e) {
            size_t L = strlen(e);
            return n.size() >= L && _stricmp(n.c_str() + n.size() - L, e) == 0; };
        if (ends(".jpg") || ends(".jpeg") || ends(".png") || ends(".bmp"))
            g_s.dir_files.push_back(n);
    } while (FindNextFileA(h, &fd));
    FindClose(h);
}

// 打开对话框 + 浏览
static void OpenImageDialog() {
    wchar_t file[MAX_PATH] = L"";
    OPENFILENAMEW ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = L"图像 (*.jpg;*.png;*.bmp) *.jpg;*.jpeg;*.png;*.bmp 所有文件 *.* ";
    ofn.lpstrFile = file; ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameW(&ofn)) {
        char path[MAX_PATH * 2];
        wcstombs(path, file, sizeof(path));
        std::string p(path);
        if (LoadImageFile(p)) {
            auto slash = p.find_last_of("\/");
            ScanDir(p.substr(0, slash));
            const std::string name = p.substr(slash + 1);
            for (size_t i = 0; i < g_s.dir_files.size(); ++i)
                if (g_s.dir_files[i] == name) g_s.dir_idx = (int)i;
            RunChain((int)g_s.steps.size() - 1);   // 打开即跑当前流程
        }
    }
}

static void BrowseRelative(int delta) {
    if (g_s.dir_files.empty()) return;
    g_s.dir_idx = (g_s.dir_idx + delta + (int)g_s.dir_files.size()) % (int)g_s.dir_files.size();
    LoadImageFile(g_s.dir_root + "\\" + g_s.dir_files[g_s.dir_idx]);
    RunChain((int)g_s.steps.size() - 1);
}

// 一键装甲板示例：整条链自动配好并运行（新手入口，不用手动配任何参数）
static const char* kDefaultArmorModel =
    "C:/Users/ASUS/Desktop/projects/rm_vision/models/0526.onnx";

static void LoadArmorDemo() {
    g_s.steps.clear();
    auto& factory = NodeFactory::instance();

    auto add = [&](const char* type) -> Step* {
        Step st;
        st.type_id = type;
        st.node = factory.create(type, "step_" + std::to_string(g_s.steps.size()));
        if (!st.node) return nullptr;
        g_s.steps.push_back(std::move(st));
        return &g_s.steps.back();
    };

    if (Step* s1 = add("nodes.OnnxInfer")) {
        s1->node->set_param("model_path", Data(String(kDefaultArmorModel)));
        s1->node->set_param("format", Data(String("rp")));
        s1->node->set_param("conf", Data(0.85));
    }
    if (Step* s2 = add("nodes.CropROI"))
        s2->node->set_param("expand", Data(10.0));
    add("nodes.LightbarDetect");

    // 优先真实装甲板帧：从 exe 目录逐级向上找 datasets/real_armor
    if (g_s.image.data.empty()) {
        static const char* dirs[] = {
            "datasets/real_armor",
            "../../datasets/real_armor",
            "../../../datasets/real_armor",
            "../../../../datasets/real_armor",
        };
        bool loaded = false;
        for (const char* d : dirs) {
            WIN32_FIND_DATAA fd;
            HANDLE h = FindFirstFileA((std::string(d) + "\*.jpg").c_str(), &fd);
            if (h == INVALID_HANDLE_VALUE) continue;
            std::vector<std::string> names;
            do { if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
                     names.push_back(fd.cFileName); } while (FindNextFileA(h, &fd));
            FindClose(h);
            if (!names.empty()) {
                std::sort(names.begin(), names.end());
                loaded = LoadImageFile(std::string(d) + "\\" + names[1 < names.size() ? 1 : 0]);
                ScanDir(d);
                g_s.dir_idx = 1;
                if (loaded) break;
            }
        }
        if (!loaded) g_s.image = MakeTestImage();   // 兜底才是示意图
    }
    RunChain(-1);
    UploadImage(g_s.image);
    g_s.selected = 0;   // 参数面板直接显示第一步
}

// --------------------------------------------------------------------------
// UI 面板
// --------------------------------------------------------------------------
static void UploadImage(const ImageData& img) {
    if (!img.data.empty() && g_device && (int)img.width > 0) {
        if (g_s.tex) { g_s.tex->Release(); g_s.tex = nullptr; }
        if (g_s.tex_srv) { g_s.tex_srv->Release(); g_s.tex_srv = nullptr; }

        const int W = (int)img.width, H = (int)img.height;
        // 统一转 RGBA8 上传
        std::vector<unsigned char> rgba((size_t)W * H * 4, 255);
        const uint32_t ch = img.channels ? img.channels : 1;
        for (size_t i = 0; i < (size_t)W * H; ++i) {
            if (ch >= 3) {   // 彩色直出
                rgba[i * 4 + 0] = img.data[i * ch + 0];
                rgba[i * 4 + 1] = img.data[i * ch + 1];
                rgba[i * 4 + 2] = img.data[i * ch + 2];
            } else {         // 灰度
                rgba[i * 4 + 0] = rgba[i * 4 + 1] = rgba[i * 4 + 2] = img.data[i * ch];
            }
        }
        D3D11_TEXTURE2D_DESC td{};
        td.Width = W; td.Height = H; td.MipLevels = 1; td.ArraySize = 1;
        td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
        td.SampleDesc.Count = 1; td.Usage = D3D11_USAGE_DEFAULT;
        td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        D3D11_SUBRESOURCE_DATA sd{};
        sd.pSysMem = rgba.data(); sd.SysMemPitch = W * 4;
        if (SUCCEEDED(g_device->CreateTexture2D(&td, &sd, &g_s.tex))) {
            D3D11_SHADER_RESOURCE_VIEW_DESC rv{};
            rv.Format = td.Format; rv.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
            rv.Texture2D.MipLevels = 1;
            g_device->CreateShaderResourceView(g_s.tex, &rv, &g_s.tex_srv);
            g_s.tex_w = W; g_s.tex_h = H;
        }
    }
}

static void PanelOperatorLibrary() {
    if (!ImGui::Begin("算子库")) { ImGui::End(); return; }
    auto& factory = NodeFactory::instance();
    const auto types = factory.get_all_types();
    ImGui::Text("共 %zu 个算子（双击插入流程）", types.size());
    ImGui::Separator();
    for (const auto& id : types) {
        const NodeInfo* info = factory.get_info(id);
        const char* name = info ? info->name.c_str() : id.c_str();
        if (ImGui::Selectable(("##op_" + id).c_str(), false,
                              ImGuiSelectableFlags_AllowDoubleClick)) {
            if (ImGui::IsMouseDoubleClicked(0)) {
                Step st;
                st.type_id = id;
                st.node = factory.create(id, "step_" + std::to_string(g_s.steps.size()));
                if (st.node) {
                    g_s.steps.push_back(std::move(st));
                    g_s.selected = (int)g_s.steps.size() - 1;
                }
            }
        }
        ImGui::SameLine();
        ImGui::Text("%s  (%s)", name, id.c_str());
    }
    ImGui::End();
}

static void PanelProgram() {
    if (!ImGui::Begin("程序（步骤链）")) { ImGui::End(); return; }
    const ImVec2 btn(110, 34);
    if (ImGui::Button("打开图像...", btn)) OpenImageDialog();
    ImGui::SameLine();
    if (ImGui::Button("一键装甲板示例", btn)) LoadArmorDemo();
    ImGui::SameLine();
    if (ImGui::Button("运行全部", btn)) { RunChain(-1); UploadImage(g_s.image); }
    ImGui::SameLine();
    if (ImGui::Button("运行到选中", btn)) {
        if (g_s.selected >= 0) { RunChain(g_s.selected); UploadImage(g_s.image); }
    }
    ImGui::SameLine();
    if (ImGui::Button("<< 上一张", btn)) BrowseRelative(-1);
    ImGui::SameLine();
    if (ImGui::Button("下一张 >>", btn)) BrowseRelative(1);
    ImGui::Spacing();
    if (!g_s.dir_files.empty() && g_s.dir_idx >= 0)
        ImGui::Text("浏览: %d/%zu  %s", g_s.dir_idx + 1, g_s.dir_files.size(),
                    g_s.dir_files[g_s.dir_idx].c_str());
    ImGui::Separator();

    for (int i = 0; i < (int)g_s.steps.size(); ++i) {
        Step& st = g_s.steps[i];
        ImGui::PushID(i);
        const bool sel = (g_s.selected == i);
        const char* flag = st.last_result.is_success() ? "" : "  [失败]";
        if (ImGui::Selectable(("[" + std::to_string(i + 1) + "] " +
                               st.type_id + flag).c_str(), sel)) {
            g_s.selected = i;
        }
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 84);
        ImGui::PushItemWidth(36);
        if (ImGui::Button(st.enabled ? "启用" : "停用")) st.enabled = !st.enabled;
        ImGui::SameLine();
        if (ImGui::Button("删")) { g_s.steps.erase(g_s.steps.begin() + i--); }
        ImGui::PopItemWidth();
        ImGui::PopID();
    }
    ImGui::End();
}

static void PanelParams() {
    if (!ImGui::Begin("参数")) { ImGui::End(); return; }
    if (g_s.selected < 0 || g_s.selected >= (int)g_s.steps.size()) {
        ImGui::TextUnformatted("（未选中步骤）");
        ImGui::End();
        return;
    }
    Step& st = g_s.steps[g_s.selected];
    const NodeInfo& info = st.node->info();
    ImGui::Text("%s", info.name.c_str());
    ImGui::TextWrapped("%s", info.description.c_str());
    ImGui::Separator();

    for (const auto& p : info.params) {
        ImGui::PushID(p.id.c_str());
        ImGui::SetNextItemWidth(180);
        if (p.type == DataType::Number) {
            double v = st.node->get_param(p.id, p.default_value).as_number();
            float f = (float)v;
            const bool has_range = p.min_value.is_number() && p.max_value.is_number();
            const double lo = has_range ? p.min_value.as_number() : -1e9;
            const double hi = has_range ? p.max_value.as_number() : 1e9;
            if (ImGui::SliderFloat(p.name.c_str(), &f, (float)lo, (float)hi, "%.3f"))
                st.node->set_param(p.id, Data((double)f));
        } else if (p.type == DataType::Boolean) {
            bool b = st.node->get_param(p.id, p.default_value).as_bool();
            if (ImGui::Checkbox(p.name.c_str(), &b)) st.node->set_param(p.id, Data(b));
        } else if (p.type == DataType::String) {
            char buf[512];
            strncpy(buf, st.node->get_param(p.id, p.default_value).as_string().c_str(),
                    sizeof(buf) - 1);
            buf[sizeof(buf) - 1] = 0;
            if (ImGui::InputText(p.name.c_str(), buf, sizeof(buf)))
                st.node->set_param(p.id, Data(String(buf)));
            if (p.id.find("path") != String::npos) {   // 路径参数给个整行浏览按钮
                if (ImGui::Button("浏览文件... (选 .onnx 模型)", ImVec2(-1, 0))) {
                    wchar_t file[MAX_PATH] = L"";
                    OPENFILENAMEW ofn{};
                    ofn.lStructSize = sizeof(ofn);
                    ofn.lpstrFilter = L"模型 (*.onnx) *.onnx 所有文件 *.* ";
                    ofn.lpstrFile = file; ofn.nMaxFile = MAX_PATH;
                    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
                    if (GetOpenFileNameW(&ofn)) {
                        char path[512];
                        wcstombs(path, file, sizeof(path));
                        st.node->set_param(p.id, Data(String(path)));
                    }
                }
            }
        }
        ImGui::PopID();
    }
    ImGui::Separator();
    // 运行结果摘要（输出端口当前值）
    for (const auto& out : info.outputs) {
        const Data d = st.node->get_output(out.id);
        ImGui::Text("%s: %s", out.name.c_str(), d.to_string().c_str());
    }
    ImGui::End();
}

static void PanelGraphics() {
    if (!ImGui::Begin("图形窗口")) { ImGui::End(); return; }
    if (!g_s.tex_srv) {
        ImGui::TextUnformatted("（无图像 —— 插入算子后点\"运行\"，或载入测试图）");
    } else {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float scale = g_s.zoom;
        ImVec2 sz((float)g_s.tex_w * scale, (float)g_s.tex_h * scale);
        ImGui::SetCursorPos(ImVec2(
            ImGui::GetCursorPos().x + g_s.pan.x,
            ImGui::GetCursorPos().y + g_s.pan.y));
        ImGui::Image((ImTextureID)(void*)g_s.tex_srv, sz);
        if (ImGui::IsItemHovered()) {
            g_s.zoom *= ImGui::GetIO().MouseWheel > 0 ? 1.1f
                       : ImGui::GetIO().MouseWheel < 0 ? 0.9f : 1.0f;
            if (g_s.zoom < 0.05f) g_s.zoom = 0.05f;
            if (g_s.zoom > 20.0f) g_s.zoom = 20.0f;
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                ImVec2 d = ImGui::GetIO().MouseDelta;
                g_s.pan.x += d.x; g_s.pan.y += d.y;
            }
        }
        ImGui::SetCursorPos(ImVec2(8, 8));
        ImGui::Text("%dx%d  zoom=%.2f", g_s.tex_w, g_s.tex_h, g_s.zoom);
    }
    ImGui::End();
}

static void PanelLog() {
    if (!ImGui::Begin("日志")) { ImGui::End(); return; }
    ImGui::TextUnformatted(g_s.log_buf.data(),
                           g_s.log_buf.data() + g_s.log_buf.size());
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4)
        ImGui::SetScrollHereY(1.0f);
    ImGui::End();
}

// --------------------------------------------------------------------------
// Win32 入口
// --------------------------------------------------------------------------
int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR, int show_cmd) {
    // 引擎初始化：算子注册（静态期已完成）+ 模块自检 + 日志
    auto log_sink = std::make_shared<StudioLogSink>();
    Logger::instance().add_sink(log_sink);
    try {
        ovf::nodes::require_user_node_module_loaded("ovf-studio");
        ovf::nodes::initialize_user_nodes();
    } catch (const Exception& e) {
        MessageBoxA(nullptr, e.what(), "OVF Studio - 算子模块自检失败", MB_ICONERROR);
        return 1;
    }

    WNDCLASSEXW wc{sizeof(wc), CS_CLASSDC, WndProc, 0L, 0L,
                   GetModuleHandle(nullptr), nullptr, nullptr, nullptr, nullptr,
                   L"OVFStudio", nullptr};
    RegisterClassExW(&wc);
    HWND hwnd = CreateWindowW(wc.lpszClassName, L"OVF Studio — 自建算子平台工作台",
                              WS_OVERLAPPEDWINDOW, 100, 100, 1440, 900,
                              nullptr, nullptr, wc.hInstance, nullptr);
    if (!CreateDeviceD3D(hwnd)) { CleanupDeviceD3D(); UnregisterClassW(wc.lpszClassName, wc.hInstance); return 1; }
    ShowWindow(hwnd, show_cmd);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    ImGui::StyleColorsDark();
    // 中文字体：ImGui 内置字体只有拉丁字形，不加载中文字体时界面全是 "?"
    {
        ImGuiIO& font_io = ImGui::GetIO();
        const char* candidates[] = {
            "C:/Windows/Fonts/msyh.ttc",      // 微软雅黑
            "C:/Windows/Fonts/simhei.ttf",    // 黑体
            "C:/Windows/Fonts/simsun.ttc",    // 宋体
        };
        bool loaded = false;
        for (const char* f : candidates) {
            if (font_io.Fonts->AddFontFromFileTTF(
                    f, 22.0f, nullptr,
                    font_io.Fonts->GetGlyphRangesChineseSimplifiedCommon())) {
                loaded = true;
                break;
            }
        }
        if (!loaded) font_io.Fonts->AddFontDefault();   // 兜底，至少能显示英文
    }
    ImGui_ImplWin32_Init(hwnd);
    ImGui_ImplDX11_Init(g_device, g_ctx);

    g_s.image = MakeTestImage();
    UploadImage(g_s.image);

    bool done = false;
    while (!done) {
        MSG msg;
        while (PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
            TranslateMessage(&msg); DispatchMessage(&msg);
            if (msg.message == WM_QUIT) done = true;
        }
        if (done) break;

        ImGui_ImplDX11_NewFrame();
        ImGui_ImplWin32_NewFrame();
        ImGui::NewFrame();

        // 固定五区布局：左(算子库/程序) | 中(图形) | 右(参数) | 底(日志)
        // 首次启动摆好，用户拖乱后可"窗口->重置布局"恢复
        static bool first_layout = true;
        ImGuiID dock = ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());
        if (first_layout) {
            first_layout = false;
            ImGui::DockBuilderRemoveNode(dock);
            ImGui::DockBuilderAddNode(dock, ImGuiDockNodeFlags_DockSpace);
            ImGui::DockBuilderSetNodeSize(dock, ImGui::GetMainViewport()->WorkSize);
            // HDevelop 式：左侧 算子库+程序+参数（参数在最下，紧贴操作对象），
            // 中间大图形窗口为主视野，底部细日志条
            ImGuiID left, bottom, mid, right, lib, main;
            ImGui::DockBuilderSplitNode(dock, ImGuiDir_Left, 0.30f, &left, &main);
            ImGui::DockBuilderSplitNode(main, ImGuiDir_Down, 0.18f, &bottom, &mid);
            ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.34f, &right, &left);  // left下段=参数
            ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.45f, &lib, &left);    // left中=程序,上=算子库
            ImGui::DockBuilderDockWindow("算子库", lib);
            ImGui::DockBuilderDockWindow("程序（步骤链）", left);
            ImGui::DockBuilderDockWindow("参数", right);
            ImGui::DockBuilderDockWindow("图形窗口", mid);
            ImGui::DockBuilderDockWindow("日志", bottom);
            ImGui::DockBuilderFinish(dock);
        }
        if (g_s.demo) ImGui::ShowDemoWindow();
        PanelOperatorLibrary();
        PanelProgram();
        PanelParams();
        PanelGraphics();
        PanelLog();

        const float clear[] = {0.08f, 0.08f, 0.10f, 1.0f};
        g_ctx->OMSetRenderTargets(1, &g_rtv, nullptr);
        g_ctx->ClearRenderTargetView(g_rtv, clear);
        ImGui::Render();
        ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());
        g_swap->Present(1, 0);
    }

    if (g_s.tex) g_s.tex->Release();
    if (g_s.tex_srv) g_s.tex_srv->Release();
    ImGui_ImplDX11_Shutdown();
    ImGui_ImplWin32_Shutdown();
    ImGui::DestroyContext();
    CleanupDeviceD3D();
    DestroyWindow(hwnd);
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return 0;
}

// D3D 设备创建 + 消息泵（放最后，样板）
static bool CreateDeviceD3D(HWND hWnd) {
    DXGI_SWAP_CHAIN_DESC sd{};
    sd.BufferCount = 2;
    sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    sd.BufferDesc.RefreshRate.Numerator = 60; sd.BufferDesc.RefreshRate.Denominator = 1;
    sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
    sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
    sd.OutputWindow = hWnd;
    sd.SampleDesc.Count = 1; sd.Windowed = TRUE;
    sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

    UINT flags = 0;
    D3D_FEATURE_LEVEL fl;
    if (D3D11CreateDeviceAndSwapChain(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
                                      flags, nullptr, 0, D3D11_SDK_VERSION, &sd,
                                      &g_swap, &g_device, &fl, &g_ctx) != S_OK)
        return false;
    CreateRenderTarget(hWnd);
    return true;
}
static void CleanupDeviceD3D() {
    CleanupRenderTarget();
    if (g_swap) { g_swap->Release(); g_swap = nullptr; }
    if (g_ctx) { g_ctx->Release(); g_ctx = nullptr; }
    if (g_device) { g_device->Release(); g_device = nullptr; }
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND, UINT, WPARAM, LPARAM);
static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return 1;
    switch (msg) {
        case WM_SIZE:
            if (g_device && wParam != SIZE_MINIMIZED) {
                CleanupRenderTarget();
                g_swap->ResizeBuffers(0, (UINT)LOWORD(lParam), (UINT)HIWORD(lParam),
                                      DXGI_FORMAT_UNKNOWN, 0);
                CreateRenderTarget(hWnd);
            }
            return 0;
        case WM_DESTROY: PostQuitMessage(0); return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}
