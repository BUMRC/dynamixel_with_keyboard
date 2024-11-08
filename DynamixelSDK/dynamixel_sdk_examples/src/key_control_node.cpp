#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
#include <ncurses.h>  // ncurses library for handling keyboard input

using namespace std::chrono_literals;

class KeyControlNode : public rclcpp::Node
{
public:
    KeyControlNode()
        : Node("key_control_node"), id_1_position(0), id_2_position(0), id_3_position(0), id_4_position(0), id_5_position(0), id_6_position(0)
    {
        // Initialize the publisher for the set_position topic
        this_publisher = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
        
         // Print instructions to the user
        RCLCPP_INFO(this->get_logger(), "Keyboard Control Instructions:");
        RCLCPP_INFO(this->get_logger(), "Press 'q' to increment position of ID 1.");
        RCLCPP_INFO(this->get_logger(), "Press 'a' to decrement position of ID 1.");
        RCLCPP_INFO(this->get_logger(), "Press 'w' to increment position of ID 2.");
        RCLCPP_INFO(this->get_logger(), "Press 's' to decrement position of ID 2.");
        RCLCPP_INFO(this->get_logger(), "Press 'e' to increment position of ID 3.");
        RCLCPP_INFO(this->get_logger(), "Press 'd' to decrement position of ID 3.");
        RCLCPP_INFO(this->get_logger(), "Press 'r' to increment position of ID 4.");
        RCLCPP_INFO(this->get_logger(), "Press 'f' to decrement position of ID 4.");
        RCLCPP_INFO(this->get_logger(), "Press 't' to increment position of ID 5.");
        RCLCPP_INFO(this->get_logger(), "Press 'g' to decrement position of ID 5.");
        RCLCPP_INFO(this->get_logger(), "Press 'y' to increment position of ID 6.");
        RCLCPP_INFO(this->get_logger(), "Press 'h' to decrement position of ID 6.");
        RCLCPP_INFO(this->get_logger(), "Press 'x' to exit the program.");
        
        // Initialize ncurses
        initscr();              // Initialize ncurses
        raw();                  // Disable line buffering
        keypad(stdscr, TRUE);   // Enable special keys (e.g., arrow keys)
        noecho();               // Don't display keypresses
        
        // Reset motors to position 0 at startup
        for(int i=1;i<7;i++){
        	resetMotorPosition(i, 0);
        }
        
        // Start a thread for reading keyboard input
        keyboard_thread_ = std::make_shared<std::thread>(&KeyControlNode::readKeyboard, this);
    }

    ~KeyControlNode()
    {
        if (keyboard_thread_ && keyboard_thread_->joinable())
        {
            keyboard_thread_->join();
        }
        
        // Clean up ncurses
        endwin();
    }

private:
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr this_publisher;
    std::shared_ptr<std::thread> keyboard_thread_;
    int id_1_position;
    int id_2_position;
    int id_3_position;
    int id_4_position;
    int id_5_position;
    int id_6_position;
    
    void resetMotorPosition(int id, int position){
        auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
        message.id = id;
        message.position = position;
        this_publisher->publish(message);

        RCLCPP_INFO(this->get_logger(), "Motor ID %d reset to position: %d", id, position);
    }

    void readKeyboard(){
        while (rclcpp::ok()){
        
            int ch = getch();  // Get the pressed key (non-blocking)

            // Control for ID 1
            if (ch == 'q'){  // Increment ID 1 position
                id_1_position += 400;
                if (id_1_position > 4000){
                    id_1_position = 4000;
                }
                RCLCPP_INFO(this->get_logger(), "ID 1 Incremented to position: %d", id_1_position);
                publishPosition(1, id_1_position);
                
            }else if (ch == 'a'){  // Decrement ID 1 position
            
                id_1_position -= 400;
                if (id_1_position < 0){
                    id_1_position= 0;
                }
                RCLCPP_INFO(this->get_logger(), "ID 1 Decremented to position: %d", id_1_position);
                publishPosition(1, id_1_position);
            }

            // Control for ID 2
            else if (ch == 'w'){  // Increment ID 2 position
            
                id_2_position += 400;
                if (id_2_position > 4000){
                    id_2_position = 4000;
                }
                RCLCPP_INFO(this->get_logger(), "ID 2 Incremented to position: %d", id_2_position);
                publishPosition(2, id_2_position);
            }else if (ch == 's'){  // Decrement ID 2 position

                id_2_position -= 400;
                if (id_2_position < 0){
                    id_2_position = 0;
                }
                RCLCPP_INFO(this->get_logger(), "ID 2 Decremented to position: %d", id_2_position);
                publishPosition(2, id_2_position);
            }
            
            // Control for ID 3
            else if (ch == 'e'){  // Increment ID 3 position
            
                id_3_position += 400;
                if (id_3_position > 4000){
                    id_3_position = 4000;
                }
                RCLCPP_INFO(this->get_logger(), "ID 3 Incremented to position: %d", id_3_position);
                publishPosition(3, id_3_position);
            }else if (ch == 'd'){  // Decrement ID 3 position

                id_3_position -= 400;
                if (id_3_position < 0){
                    id_3_position = 0;
                }
                RCLCPP_INFO(this->get_logger(), "ID 3 Decremented to position: %d", id_3_position);
                publishPosition(3, id_3_position);
            }
            
            // Control for ID 4
            else if (ch == 'r'){  // Increment ID 4 position
            
                id_4_position += 400;
                if (id_4_position > 4000){
                    id_4_position = 4000;
                }
                RCLCPP_INFO(this->get_logger(), "ID 4 Incremented to position: %d", id_4_position);
                publishPosition(4, id_4_position);
            }else if (ch == 'f'){  // Decrement ID 4 position

                id_4_position -= 400;
                if (id_4_position < 0){
                    id_4_position = 0;
                }
                RCLCPP_INFO(this->get_logger(), "ID 4 Decremented to position: %d", id_4_position);
                publishPosition(4, id_4_position);
            }
            
            // Control for ID 5
            else if (ch == 't'){  // Increment ID 5 position
            
                id_5_position += 400;
                if (id_5_position > 4000){
                    id_5_position = 4000;
                }
                RCLCPP_INFO(this->get_logger(), "ID 5 Incremented to position: %d", id_5_position);
                publishPosition(5, id_5_position);
            }else if (ch == 'g'){  // Decrement ID 5 position

                id_5_position -= 400;
                if (id_5_position < 0){
                    id_5_position = 0;
                }
                RCLCPP_INFO(this->get_logger(), "ID 5 Decremented to position: %d", id_5_position);
                publishPosition(5, id_5_position);
            }
            
            // Control for ID 6
            else if (ch == 'y'){  // Increment ID 6 position
            
                id_6_position += 400;
                if (id_6_position > 4000){
                    id_6_position = 4000;
                }
                RCLCPP_INFO(this->get_logger(), "ID 6 Incremented to position: %d", id_6_position);
                publishPosition(6, id_6_position);
            }else if (ch == 'h'){  // Decrement ID 6 position

                id_6_position -= 400;
                if (id_6_position < 0){
                    id_6_position = 0;
                }
                RCLCPP_INFO(this->get_logger(), "ID 6 Decremented to position: %d", id_6_position);
                publishPosition(6, id_6_position);
            }

            // Quit the program when 'x' is pressed
            if (ch == 'x') {
		RCLCPP_INFO(this->get_logger(), "Shutting down the node.");
		rclcpp::shutdown();  // Gracefully shut down the ROS 2 node
		break;  // Exit the loop
	    }

            std::this_thread::sleep_for(100ms);  // To avoid high CPU usage
        }
    }

    void publishPosition(int id, int position)
    {
        auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
        message.id = id;
        message.position = position;
        this_publisher->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyControlNode>());
    rclcpp::shutdown();
    return 0;
}

