#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <ncurses.h>

class CommandNode : public rclcpp::Node {
    CommandNode()
    : Node("command_node"), position(0){

        // Initialize the publisher for the set_position topic
        this_publisher = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
        
        // Initialize ncurses
        initscr();              // Initialize ncurses
        raw();                  // Disable line buffering
        keypad(stdscr, TRUE);   // Enable special keys (e.g., arrow keys)
        noecho();               // Don't display keypresses
        
        // Reset motors to position 0 at startup
        for(int i=1;i<5;i++){
        	resetMotorPosition(i, 0);
        }
        
        // Start a thread for reading keyboard input
        keyboard_thread = std::make_shared<std::thread>(&CommandNode::readKeyboard, this);
    }

    ~CommandNode(){
        if (keyboard_thread && keyboard_thread->joinable())
        {
            keyboard_thread->join();
        }
        
        // Clean up ncurses
        endwin();
    }

private:
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr this_publisher;
    std::shared_ptr<std::thread> keyboard_thread;
    int position;
    
    void resetMotorPosition(int id, int position){
        auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
        message.id = id;
        message.position = position;
        this_publisher->publish(message);

        RCLCPP_INFO(this->get_logger(), "Motor ID %d reset to position: %d", id, position);
    }

    void readKeyboard(){
        while (rclcpp::ok()) {

            std::string id_input;
            std::cout << "Enter ID (or '0' to exit): ";
            std::cin >> id_input;

            if (id_input == "x") {
                RCLCPP_INFO(this->get_logger(), "Exiting program.");
                rclcpp::shutdown();
                return;
            }

            int motor_id = std::stoi(id_input);
            RCLCPP_INFO(this->get_logger(), "Entered ID: %d", motor_id);
            controlMotor(motor_id);
        }
    }

    void controlMotor(int motor_id){

        while(rclcpp::ok()){

            RCLCPP_INFO(this->get_logger(), "Press 'Q' to increment motor, 'A' to decrement motor, or 'X' to back to ID changing mode");

            int ch = getch();

            if (ch == 'q'){
                position += 100;
                if (position > 1000){
                    position = 1000;
                }
                RCLCPP_INFO(this->get_logger(), "Motor Incremented to position: %d", position);
                publishPosition(motor_id, position);
                
            }else if (ch == 'a'){
                position -= 100;
                if (position < 0){
                    position= 0;
                }
                RCLCPP_INFO(this->get_logger(), "Motor Decremented to position: %d", position);
                publishPosition(motor_id, position);

            }else if(ch == 'x'){
                RCLCPP_INFO(this->get_logger(), "Exiting Q/A mode.");
                break;  // Return to ID prompt mode
            }
        }
    }
    void publishPosition(int id, int position){
        auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();
        message.id = id;
        message.position = position;
        this_publisher->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto commandnode = std::make_shared<CommandNode>();

    rclcpp::spin(commandnode);
    rclcpp::shutdown();
    return 0;
}