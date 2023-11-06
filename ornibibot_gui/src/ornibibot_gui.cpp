#include <iostream>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_data.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_gui.hpp"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "implot.h"
#include <GLFW/glfw3.h>
#include <cmath>

void error_callback(int error, const char* description)
{
    std::cerr << "Error: " << description << std::endl;
}




int main(int argc, char** argv)
{
    // Set this callback before you call glfwInit()
    glfwSetErrorCallback(error_callback);
    // Setup GLFW window
    if (!glfwInit()) std::cerr << "Error init glfw";
    // Set the OpenGL context version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    GLFWwindow* gui_window = glfwCreateWindow(1280, 720, "GUI", NULL, NULL);

    if (gui_window == NULL) std::cerr << "window is null";

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("converter");
    
    glfwMakeContextCurrent(gui_window);
    glfwSwapInterval(1); // Enable vsync

    // Setup ImGui binding
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui_ImplGlfw_InitForOpenGL(gui_window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Setup style
    ImGui::StyleColorsDark();

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Main loop
    while (!glfwWindowShouldClose(gui_window) && rclcpp::ok())
    {
        // Poll and handle events (inputs, window resize, etc.)
        glfwPollEvents();

        // Start the ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Show a simple window that we create ourselves.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

            ImGui::Text("This is some useful text."); // Display some text (you can use a format string too)
            ImGui::SliderFloat("float", &f, 0.0f, 1.0f); // Edit 1 float using a slider from 0.0f to 1.0f

            ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bool storing our window open/close state
            ImGui::Checkbox("Another Window", &show_another_window);

            if (ImGui::Button("Button")) // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            // Create a simple plot using ImPlot
            static float values[100];

            if (ImPlot::BeginPlot("Simple Plot")) {
                ImPlot::PlotLine("My Line Plot", values, 100);
                ImPlot::EndPlot();
            }

            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(gui_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(gui_window);
    }
    rclcpp::shutdown();

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(gui_window);
    glfwTerminate();

    return 0;
}
