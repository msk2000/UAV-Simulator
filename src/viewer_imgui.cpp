/********************************************************************
 * @file viewer_imgui.cpp
 * @brief Adapter for Easy3D ImGui integration for the UAV Simulator.
 *
 * This implementation is based on the Easy3D ImGui tutorial and is
 * responsible for GUI rendering, input handling, and custom overlays
 * such as the UAV Simulator logo and framerate display.
 ********************************************************************/
 
#include "viewer_imgui.h"
#include <implot.h>
 
#include <easy3d/util/file_system.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/renderer/camera.h>
 
#include <3rd_party/glfw/include/GLFW/glfw3.h>  // for glfw functions
 
#include <3rd_party/imgui/misc/fonts/imgui_fonts_droid_sans.h>
#include <3rd_party/imgui/imgui.h>
#include <3rd_party/imgui/backends/imgui_impl_glfw.h>
#include <3rd_party/imgui/backends/imgui_impl_opengl3.h>
 
 
namespace easy3d 
{
 
    // \cond
 
    ImGuiContext* ViewerImGui::context_ = nullptr;
 
    /**
    * @brief Constructor for the ViewerImGui class.
    *
    * Initializes an ImGui-based viewer window using the provided settings.
    *
    * @param title        Title of the application window.
    * @param samples      Anti-aliasing samples.
    * @param gl_major     Major version of OpenGL context.
    * @param gl_minor     Minor version of OpenGL context.
    * @param full_screen  Whether to launch in fullscreen.
    * @param resizable    Whether the window is resizable.
    * @param depth_bits   Bits allocated to the depth buffer.
    * @param stencil_bits Bits allocated to the stencil buffer.
    * @param width        Initial window width in pixels.
    * @param height       Initial window height in pixels.
    */
    ViewerImGui::ViewerImGui(
        const std::string& title /* = "Easy3D ImGui Viewer" */,
        int samples /* = 4 */,
        int gl_major /* = 3 */,
        int gl_minor /* = 2 */,
        bool full_screen /* = false */,
        bool resizable /* = true */,
        int depth_bits /* = 24 */,
        int stencil_bits /* = 8 */,
        int width /* = 960 */,
        int height /* = 800 */
    ) : Viewer(title, samples, gl_major, gl_minor, full_screen, resizable, depth_bits, stencil_bits, width, height)
    {
#if defined(_WIN32) && defined(_MSC_VER)
        // Liangliang: the internal glfw won't be shared across dll boundaries (But seems ok on macOS. That is weird!)
        // see https://github.com/LiangliangNan/Easy3D/issues/178
        glfwInit();
#endif
    }
 
    /**
    * @brief Destructor for ViewerImGui.
    *
    * Properly shuts down ImGui and its OpenGL/GLFW bindings.
    */
    ViewerImGui::~ViewerImGui() 
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImPlot::DestroyContext();
        ImGui::DestroyContext(context_);

 
        // Not needed: it will be called in the destructor of the base class
        //Viewer::cleanup();
    }
 
    /**
    * @brief Initializes the ImGui GUI system.
    *
    * Must be called after window creation and OpenGL context initialization.
    */
    void ViewerImGui::init() 
    {
        Viewer::init();
 
        if (!context_) 
        {
            // Setup ImGui binding
            IMGUI_CHECKVERSION();
 
            context_ = ImGui::CreateContext();
            ImPlot::CreateContext();
 
            const char* glsl_version = "#version 150";
            ImGui_ImplGlfw_InitForOpenGL(window_, true);
            ImGui_ImplOpenGL3_Init(glsl_version);
            ImGuiIO& io = ImGui::GetIO();
            io.WantCaptureKeyboard = true;
            io.WantTextInput = true;
            io.IniFilename = nullptr;
            ImGui::StyleColorsLight();
            ImGuiStyle& style = ImGui::GetStyle();
            style.FrameRounding = 5.0f;
 
            // load font
            reload_font();
        }
    }
 
    /**
    * @brief Computes the device pixel ratio for HiDPI displays.
    * @return Pixel ratio between framebuffer and viewport width.
    */
    float ViewerImGui::pixel_ratio() 
    {
        // Computes pixel ratio for hidpi devices
        int fw = framebuffer_width();
        int vw = width();
        return static_cast<float>(fw) / static_cast<float>(vw);
    }
 
 /**
 * @brief Reloads ImGui font with the specified size.
 *
 * Also sets global font scaling based on pixel ratio.
 *
 * @param font_size Desired font size (default = 15).
 */
    void ViewerImGui::reload_font(int font_size)
    {
        ImGuiIO& io = ImGui::GetIO();
        io.Fonts->Clear();
        io.Fonts->AddFontFromMemoryCompressedTTF(droid_sans_compressed_data, droid_sans_compressed_size, static_cast<float>(font_size) * dpi_scaling());
        io.FontGlobalScale = 1.0f / pixel_ratio();
        ImGui_ImplOpenGL3_DestroyDeviceObjects();
    }
 
/**
 * @brief Handles resizing of the viewer window.
 *
 * Updates ImGui's display size accordingly.
 *
 * @param w New window width.
 * @param h New window height.
 */
    void ViewerImGui::post_resize(int w, int h) 
    {
        Viewer::post_resize(w, h);
        if (context_) {
            ImGui::GetIO().DisplaySize.x = float(w);
            ImGui::GetIO().DisplaySize.y = float(h);
        }
    }
 
 /**
 * @brief Handles mouse cursor movement.
 * @return true if ImGui captures the mouse; otherwise delegates to base.
 */
    bool ViewerImGui::callback_event_cursor_pos(double x, double y) 
    {
        if (ImGui::GetIO().WantCaptureMouse)
            return true;
        return Viewer::callback_event_cursor_pos(x, y);
    }
 
/**
 * @brief Handles mouse button events.
 * @return true if ImGui captures the mouse input; otherwise delegates to base.
 */
    bool ViewerImGui::callback_event_mouse_button(int button, int action, int modifiers) 
    {
        if (ImGui::GetIO().WantCaptureMouse)
            return true;
        return Viewer::callback_event_mouse_button(button, action, modifiers);
    }
 
 /**
 * @brief Handles keyboard key events.
 * @return true if ImGui captures the keyboard input; otherwise delegates to base.
 */
    bool ViewerImGui::callback_event_keyboard(int key, int action, int modifiers) 
    {
        if (ImGui::GetIO().WantCaptureKeyboard)
            return true;
        return Viewer::callback_event_keyboard(key, action, modifiers);
    }
 
/**
 * @brief Handles character input (e.g., typing).
 * @return true if ImGui captures the keyboard input; otherwise delegates to base.
 */
    bool ViewerImGui::callback_event_character(unsigned int codepoint) 
    {
        if (ImGui::GetIO().WantCaptureKeyboard)
            return true;
        return Viewer::callback_event_character(codepoint);
    }
 
/**
 * @brief Handles scroll wheel events.
 * @return true if ImGui captures the scroll event; otherwise delegates to base.
 */
    bool ViewerImGui::callback_event_scroll(double dx, double dy) 
    {
        if (ImGui::GetIO().WantCaptureMouse)
            return true;
        return Viewer::callback_event_scroll(dx, dy);
    }
 
/**
 * @brief Begins a new ImGui frame and calls the base pre_draw.
 */
    void ViewerImGui::pre_draw() 
    {
        ImGui_ImplGlfw_NewFrame();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui::NewFrame();   
 
        Viewer::pre_draw(); 
    }
 
/**
 * @brief Renders ImGui UI and overlays after scene rendering.
 *
 * Also renders custom logo and framerate text using Easy3D's TextRenderer.
 */
    void ViewerImGui::post_draw() 
    {
        static bool show_about = false;
        if (show_about) {
            ImGui::SetNextWindowPos(ImVec2(static_cast<float>(width()) * 0.5f, static_cast<float>(height()) * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
            ImGui::Begin("About UAV Simulator", &show_about, ImGuiWindowFlags_NoResize);
            ImGui::Text("An Open Source Flight Simulator for students, academics, researchers and enthusiasts!");
            ImGui::Separator();
            ImGui::Text(
                "\n"
                "M Safat Khan\n"
                "khan.safat.m@gmail.com\n"
                "https://www.linkedin.com/in/gncf\n"
            );
            ImGui::End();
        }
 
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 8));
        if (ImGui::BeginMainMenuBar())
        {
            draw_menu_file();
            draw_menu_view();
 
            if (ImGui::BeginMenu("Help")) 
            {
                ImGui::MenuItem("About", nullptr, &show_about);
                ImGui::EndMenu();
            }
            menu_height_ = ImGui::GetWindowHeight();
            ImGui::EndMainMenuBar();
        }
        ImGui::PopStyleVar();
 
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData()); 

 
        // workaround to draw the Easy3D logo and framerate at a different location (due to the menu bar)
        auto show_logo = show_easy3d_logo_;
        auto show_fps = show_frame_rate_;
        show_easy3d_logo_ = true;
        show_frame_rate_ = true;
        //Viewer::post_draw();
        show_easy3d_logo_ = show_logo;
        show_frame_rate_ = show_fps;

        // draw Easy3D logo (on the right side)
        if (texter_)
        {
            const float font_size = 15.0f;
            const float offset_x = (width() - texter_->string_width("Easy3D", font_size) - 50.0f) * dpi_scaling();
            if (show_easy3d_logo_)
            {
                const float offset_y = (20.0f + menu_height_) * dpi_scaling();
                texter_->draw("Easy3D", offset_x, offset_y, font_size, 0);
            }

            // draw the framerate
            if (show_frame_rate_)
            {
                const float offset_y = (200.0f + menu_height_) * dpi_scaling();
                texter_->draw(framerate_, offset_x, offset_y, font_size, 1);
            }
        }

    }
 
 /**
 * @brief Draws the "File" menu in the main menu bar.
 *
 * Provides Open, Save As, and Quit options.
 */
    void ViewerImGui::draw_menu_file() 
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open", "Ctrl+O"))
                open();
            if (ImGui::MenuItem("Save As...", "Ctrl+S"))
                save();
 
            ImGui::Separator();
            if (ImGui::MenuItem("Quit", "Alt+F4"))
                glfwSetWindowShouldClose(window_, GLFW_TRUE);
 
            ImGui::EndMenu();
        }
    }
 
/**
 * @brief Draws the "View" menu in the main menu bar.
 *
 * Provides camera snapshot option.
 */
    void ViewerImGui::draw_menu_view() {
        if (ImGui::BeginMenu("View"))
        {
            if (ImGui::MenuItem("Snapshot", nullptr))
                snapshot();
 
            ImGui::EndMenu();
        }
    }
 
    // \endcond
 
}

