/********************************************************************
 * @file viewer_imgui.h
 * @brief ViewerImGui class header based on the Easy3D ImGui example.
 *
 * This class extends Easy3D's Viewer to include an ImGui-based GUI
 * system for rendering widgets, menus, and overlays in an OpenGL
 * context. It integrates ImGui input handling and layout management.
 ********************************************************************/
 
#ifndef EASY3D_TUTORIAL_VIEWER_IMGUI_H
#define EASY3D_TUTORIAL_VIEWER_IMGUI_H
 
 
#include <easy3d/viewer/viewer.h>
 

 
// Forward declaration to avoid including full ImGui headers in this file
struct ImGuiContext;
 
 
namespace easy3d
{
/**
 * @class ViewerImGui
 * @brief An Easy3D viewer with ImGui GUI support for custom rendering and input.
 *
 * This viewer integrates the Dear ImGui library into the Easy3D rendering
 * pipeline, allowing the use of custom toolbars, overlays, and event handling
 * in a platform-independent manner.
 */
    class ViewerImGui : public Viewer
    {
    public:
        /**
     * @brief Constructor for ViewerImGui.
     *
     * Initializes a viewer window with OpenGL and ImGui support.
     *
     * @param title        Title of the application window.
     * @param samples      Number of anti-aliasing samples.
     * @param gl_major     Major version of OpenGL.
     * @param gl_minor     Minor version of OpenGL.
     * @param full_screen  Launch in fullscreen mode.
     * @param resizable    Window resizable flag.
     * @param depth_bits   Depth buffer precision.
     * @param stencil_bits Stencil buffer precision.
     * @param width        Initial window width in pixels.
     * @param height       Initial window height in pixels.
     */
        explicit ViewerImGui(
            const std::string& title = "Easy3D ViewerImGui",
            int samples = 4,
            int gl_major = 3,   // must >= 3
            int gl_minor = 2,   // must >= 2
            bool full_screen = false,
            bool resizable = true,
            int depth_bits = 24,
            int stencil_bits = 8,
            int width = 800,
            int height = 600
        );
        /**
        * @brief Destructor. Cleans up ImGui and OpenGL resources.
        */
        ~ViewerImGui() override;
 
    protected:
 
        /**
        * @brief Initializes ImGui and binds it to the OpenGL and GLFW context.
        */
        void init() override;
 
        /**
     * @brief Called before rendering the scene.
     *
     * Prepares a new ImGui frame.
     */
        void pre_draw() override;
 
        /**
     * @brief Called after rendering the scene.
     *
     * Draws ImGui widgets, menus, and overlays (e.g., logo and FPS).
     */
        void post_draw() override;
        /**
     * @brief Handles window resize events.
     *
     * Updates ImGui's display size.
     *
     * @param w New window width.
     * @param h New window height.
     */
        void post_resize(int w, int h) override;
 /**
     * @brief Handles mouse movement.
     *
     * @return true if ImGui captures the input.
     */
        bool callback_event_cursor_pos(double x, double y) override;
       /**
     * @brief Handles mouse button events.
     *
     * @return true if ImGui captures the input.
     */
        bool callback_event_mouse_button(int button, int action, int modifiers) override;
        /**
     * @brief Handles keyboard key events.
     *
     * @return true if ImGui captures the input.
     */
        bool callback_event_keyboard(int key, int action, int modifiers) override;
        /**
     * @brief Handles character input events.
     *
     * @return true if ImGui captures the input.
     */
        bool callback_event_character(unsigned int codepoint) override;
        /**
     * @brief Handles mouse scroll events.
     *
     * @return true if ImGui captures the input.
     */
        bool callback_event_scroll(double dx, double dy) override;
        /**
     * @brief Draws the "File" menu in the main menu bar.
     */
        void draw_menu_file();
        /**
     * @brief Draws the "View" menu in the main menu bar.
     */
        void draw_menu_view();
 
    protected:
        /**
     * @brief Computes the pixel ratio between framebuffer and window size.
     *
     * Useful for supporting HiDPI displays.
     *
     * @return Pixel ratio (usually 1.0 for standard DPI).
     */
        float pixel_ratio();
        /**
     * @brief Calculates the widget scaling factor.
     *
     * This accounts for DPI scaling and pixel ratio.
     *
     * @return Scaling factor for ImGui widgets.
     */
        float widget_scaling() { return dpi_scaling() / pixel_ratio(); }
 
        /**
     * @brief Reloads the ImGui font at a given font size.
     *
     * This clears the font atlas and applies new scaling.
     *
     * @param font_size Desired font size (default = 16).
     */
        void  reload_font(int font_size = 16);
 
    protected:
        /**
     * @brief Global ImGui context shared across all viewer instances.
     *
     * Can be overridden to support per-window contexts, but not typical.
     */
        static ImGuiContext *   context_;
        /**
     * @brief Height of the menu bar, used for adjusting overlay positions.
     */
        float menu_height_;
    };
 
}
 
#endif  // EASY3D_TUTORIAL_VIEWER_IMGUI_H
