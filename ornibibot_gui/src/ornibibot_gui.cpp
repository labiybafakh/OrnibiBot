#include <iostream>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_data.hpp"
#include "ornibibot_msgs/msg/ornibi_bot_gui.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "optitrack_msgs/msg/optitrack_data.hpp"

#include "GL/glew.h"
#include <GLFW/glfw3.h>


#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "implot.h"
#include <cmath>
#include <memory>

struct marker{
    float x;
    float y;
    float z;
};

const size_t n_marker = 8;

std::unique_ptr<marker[]> wing_marker(new marker[n_marker]);

using namespace std::chrono_literals;
using std::placeholders::_1;


void error_callback(int error, const char* description)
{
    std::cerr << "Error: " << description << std::endl;
}

std::atomic<float> force_x;


void visualize_frame(){
    return 0;
}

void visualize_marker(){
    glBegin(GL_POLYGON);
    glClear( GL_COLOR_BUFFER_BIT| GL_DEPTH_BUFFER_BIT);


    glBegin(GL_TRIANGLES);
    glVertex3f(-0.5f, -0.5f, 0.5f);
    glVertex3f(0.5f, -0.5f, 0.5f);
    glVertex3f(0.0f, 0.5f, 0.5f);
    glEnd();

    // for (int i = 0; i < (uint8_t) n_marker; i++){
    //     glColor3f(0.55f, 0.55f, 0.55f);
    //     glVertex3f(wing_marker[i].x, wing_marker[i].y, wing_marker[i].z);

    //     std::cout << i << "==>" << wing_marker[i].x << ", " << wing_marker[i].y << ", " << wing_marker[i].z << std::endl;
    // }
    glEnd();

}
    glVertex3f(-0.5f, -0.5f, 0.5f);
    glVertex3f(0.5f, -0.5f, 0.5f);
    glVertex3f(0.0f, 0.5f, 0.5f);
    glEnd();


void callback_optitrack(const optitrack_msgs::msg::OptitrackData::SharedPtr optitrack_data){
    wing_marker[0].x = optitrack_data->marker1.x;
    wing_marker[0].y = optitrack_data->marker1.y;
    wing_marker[0].z = optitrack_data->marker1.z;
    
    wing_marker[1].x = optitrack_data->marker2.x;
    wing_marker[1].y = optitrack_data->marker2.y;
    wing_marker[1].z = optitrack_data->marker2.z;
    
    wing_marker[2].x = optitrack_data->marker3.x;
    wing_marker[2].y = optitrack_data->marker3.y;
    wing_marker[3].z = optitrack_data->marker3.z;
    
    wing_marker[3].x = optitrack_data->marker4.x;
    wing_marker[3].y = optitrack_data->marker4.y;
    wing_marker[3].z = optitrack_data->marker4.z;
    
    wing_marker[4].x = optitrack_data->marker5.x;
    wing_marker[4].y = optitrack_data->marker5.y;
    wing_marker[4].z = optitrack_data->marker5.z;
    
    wing_marker[5].x = optitrack_data->marker6.x;
    wing_marker[5].y = optitrack_data->marker6.y;
    wing_marker[5].z = optitrack_data->marker6.z;
    
    wing_marker[6].x = optitrack_data->marker7.x;
    wing_marker[6].y = optitrack_data->marker7.y;
    wing_marker[6].z = optitrack_data->marker7.z;
    
    wing_marker[7].x = optitrack_data->marker8.x;
    wing_marker[7].y = optitrack_data->marker8.y;
    wing_marker[7].z = optitrack_data->marker8.z;
        
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
    auto node = rclcpp::Node::make_shared("gui");
    // auto gui_pub = node->create_publisher<
    auto force_sub = node->create_subscription<optitrack_msgs::msg::OptitrackData>("optitrack_data", 5, callback_optitrack);

    
    glfwMakeContextCurrent(gui_window);
    glfwSwapInterval(1); // Enable vsync
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

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
        
        rclcpp::spin_some(node);
        // Poll and handle events (inputs, window resize, etc.)]
        glfwPollEvents();

        // Start the ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();


        // Show a simple window that we create ourselves.
        {
            static int flapping_frequency = 0.0f;
            static int amplitude = 0;
            static int offset = 0;
            static int counter = 0;

            ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

            ImGui::Text("This is some useful text."); // Display some text (you can use a format string too)
            ImGui::SliderInt("Flapping Frequency", &flapping_frequency, 0, 5); // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::SliderInt("Amplitude", &amplitude, 0, 45);
            ImGui::SliderInt("Offset1", &offset, -45, 45);


            ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bool storing our window open/close state
            ImGui::Checkbox("Another Window", &show_another_window);

            if (ImGui::Button("Button")) // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::End();
        }
        {
            ImGui::Begin("TESTING");
            visualize_marker();
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
