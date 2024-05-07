#include "ornibibot_gui.hpp"

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


char data[17] = "  Record Data  ";

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
    
}

void visualize_marker(){
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

}

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
    auto gui_pub = node->create_publisher<ornibibot_msgs::msg::OrnibiBotGUI>("ornibibot_gui", 5);
    auto gui_data = std::make_shared<ornibibot_msgs::msg::OrnibiBotGUI>(); 

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
    bool flag_record = false;

    flapping_param flapping_param_;

    flapping_param_.offset =  0;
    flapping_param_.amplitude = 0;
    flapping_param_.patterns = 0;
    flapping_param_.frequency = 0;
    flapping_param_.down_stroke_periode = 50;

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


            ImGui::Begin("Control Panel"); // Create a window called "Hello, world!" and append into it.
            ImGui::Text("Flapping parameters:");
        

            ImGui::Text("Pattern:");
            ImGui::SameLine();
            if(ImGui::Button("Sine")) flapping_param_.patterns = sine;
            ImGui::SameLine();
            if(ImGui::Button("Square")) flapping_param_.patterns = square;
            ImGui::SameLine();
            if(ImGui::Button("Triangle")) flapping_param_.patterns = triangle;
            ImGui::SameLine();
            if(ImGui::Button("Saw")) flapping_param_.patterns = saw;
            ImGui::SameLine();
            if(ImGui::Button("Rev-Saw")) flapping_param_.patterns = rev_saw;
            ImGui::SameLine();
            if(ImGui::Button("Adjusted Sine")) flapping_param_.patterns = adjusted_sine;
        
            
            if(ImGui::Button(" - "))    if(flapping_param_.frequency > 0) flapping_param_.frequency-= 0.5;
            ImGui::SameLine();
            ImGui::Text("Frequency: %.2f", flapping_param_.frequency );
            ImGui::SameLine();
            if(ImGui::Button(" + "))    flapping_param_.frequency  += 0.5;
            ImGui::SameLine();
            if(ImGui::Button("Zero Frequency"))   flapping_param_.frequency  = 0;       
    
            // if(ImGui::Button("-"))
            //     if(flapping_param_.down_stroke_periode > 10)flapping_param_.down_stroke_periode -= 10;
            // ImGui::SameLine();
            // ImGui::Text("Down-stroke Periode: %d", flapping_param_.down_stroke_periode);
            // ImGui::SameLine();
            // if(ImGui::Button("+"))
            //     if(flapping_param_.down_stroke_periode < 90)flapping_param_.down_stroke_periode += 10;
            // ImGui::SameLine();
            // if(ImGui::Button("Neutral")) flapping_param_.down_stroke_periode = 50; 
        
            if(ImGui::Button("  -  ")) 
                if(flapping_param_.offset > -25) flapping_param_.offset -= 5;
            ImGui::SameLine();
            ImGui::Text("Offset: %d", flapping_param_.offset);
            ImGui::SameLine();
            if(ImGui::Button("  +  "))
                if(flapping_param_.offset < 25) flapping_param_.offset += 5;
            ImGui::SameLine();
            if(ImGui::Button("Zero Offset")) flapping_param_.offset = 0;               
         
            if(ImGui::Button("-"))
                if(flapping_param_.amplitude > 5)flapping_param_.amplitude -= 5;
            ImGui::SameLine();
            ImGui::Text("Amplitude: %d", flapping_param_.amplitude);
            ImGui::SameLine();
            if(ImGui::Button("+"))
                if(flapping_param_.amplitude < 60)flapping_param_.amplitude += 5;
            ImGui::SameLine();
            if(ImGui::Button("Zero Amplitude")) flapping_param_.amplitude = 0;    

            ImGui::SliderInt("Down-stroke Periode", &flapping_param_.down_stroke_periode, 10, 90);

            if(ImGui::Button(data)) {
                if (!strcmp(data, "  Record Data  ")) {
                    strcpy(data, " Finish Record ");
                    flag_record = 1;
                }
                else {
                    strcpy(data, "  Record Data  ");
                    flag_record = 0;
                }
            }

            // gui_data->time = node::TimerBase::now();
            gui_data->flapping_amplitude = flapping_param_.amplitude;
            gui_data->flapping_frequency = flapping_param_.frequency;
            gui_data->flapping_offset = flapping_param_.offset;
            gui_data->flapping_mode = flapping_param_.patterns;
            gui_data->flapping_down_stroke_percentage = flapping_param_.down_stroke_periode;
            gui_data->record_data = flag_record;

            gui_pub->publish(*gui_data);

            ImGui::End();
        }

        ImGui::Begin("Visualization");
        {
        // Using a Child allow to fill all the space of the window.
        // It also alows customization
        ImGui::BeginChild("Marker Window");
        // Get the size of the child (i.e. the whole draw size of the windows).
        ImVec2 wsize = ImGui::GetWindowSize();
        // Because I use the texture from OpenGL, I need to invert the V from the UV.
        // ImGui::Image(reinterpret_cast<ImTextureID>(GL_TEXTURE_HANDLE), wsize, ImVec2(0, 1), ImVec2(1, 0));
        ImGui::EndChild();
        }
        ImGui::End();

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
