#include "ximu3_ros_driver/connection.hpp"


enum class ahrsmsgtype : uint8_t
{
    quaternion,
    rotationMatrix,
    eulerAngles,
    linearAcceleration,
    earthAcceleration
};

int main(int argc, char** argv)
{
    std::string _inputStr;
    std::string _inputValue;
    std::cout << "Type what string to send" << std::endl;
    std::cin >> _inputStr;
    std::cout << "Type the value to send" << std::endl;
    std::cin >> _inputValue;

    

    return 0;

}