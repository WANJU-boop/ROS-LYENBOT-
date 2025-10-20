/***
@作者: 古月居(www.guyuehome.com) & Gemini
@说明: ROS2话题示例-订阅“Hello World”话题消息, 并使用环形缓冲区解析打印
***/

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"              // ROS2 C++接口库
#include "std_msgs/msg/string.hpp"        // 字符串消息类型

// 使用 std::placeholders::_1 来绑定回调函数
using std::placeholders::_1;

// 指令的最小长度
#define COMMAND_MIN_LENGTH 4
// 循环缓冲区大小
#define BUFFER_SIZE 128
// 循环缓冲区
uint8_t buffer[BUFFER_SIZE];
// 循环缓冲区读索引
uint8_t readIndex = 0;
// 循环缓冲区写索引
uint8_t writeIndex = 0;


class Buff_Ring 
{
public:
    // 将数据写入环形缓冲区
    uint8_t Command_Write(const uint8_t *data, uint8_t length)
    {
        if (Command_GetRemain() < length)
            return 0; // 剩余空间不足

        if (writeIndex + length < BUFFER_SIZE)
        {
            memcpy(buffer + writeIndex, data, length);
            writeIndex += length;
        }
        else
        {
            uint8_t firstLength = BUFFER_SIZE - writeIndex;
            memcpy(buffer + writeIndex, data, firstLength);
            memcpy(buffer, data + firstLength, length - firstLength);
            writeIndex = length - firstLength;
        }
        return length;
    }

    // 从环形缓冲区中获取一条完整的指令
    uint8_t Command_GetCommand(uint8_t *command)
    {
        // 循环寻找完整指令
        while (1)
        {
            // 如果缓冲区数据长度小于最小指令长度，则不可能有完整的指令
            if (Command_GetLength() < COMMAND_MIN_LENGTH)
                return 0;

            // 如果不是包头(0xAA)，则跳过一个字节，重新开始寻找
            if (Command_Read(readIndex) != 0xAA)
            {
                Command_AddReadIndex(1);
                continue;
            }

            // 获取指令长度字节
            uint8_t length = Command_Read(readIndex + 1);
            if (length < COMMAND_MIN_LENGTH) // 基本的长度检查
            {
                Command_AddReadIndex(1);
                continue;
            }

            // 如果缓冲区中的数据长度小于指令宣称的长度，则数据不完整
            if (Command_GetLength() < length)
            {
                return 0;
            }

            // 计算校验和
            uint8_t sum = 0;
            for (uint8_t i = 0; i < length - 1; i++) 
            {
                sum += Command_Read(readIndex + i);
            }

            // 如果校验和不正确，则跳过一个字节，重新开始寻找
            if (sum != Command_Read(readIndex + length - 1)) 
            {
                Command_AddReadIndex(1);
                continue;
            }

            // 如果找到完整指令，则将指令复制到 command 缓冲区，并返回指令长度
            for (uint8_t i = 0; i < length; i++) 
            {
                command[i] = Command_Read(readIndex + i);
            }
            Command_AddReadIndex(length);
            return length;
        }
    }

    // [新增] 打印指令内容的函数
    void Command_Print(uint8_t *command, uint8_t length)
    {
        if (length == 0) return;
        
        // 使用 RCLCPP_INFO 来打印，更符合ROS2的风格
        // 需要一个Node的Logger实例，这里暂时用printf
        printf("Parsed Command (length: %d): ", length);
        for(uint8_t i = 0; i < length; ++i)
        {
            printf("0x%02X ", command[i]);
        }
        printf("\n");
    }

private:
    uint8_t Command_Read(uint8_t index)
    {
        return buffer[index % BUFFER_SIZE];
    }
    
    // [修正] 修正了拼写错误和语法
    void Command_AddReadIndex(uint8_t length)
    {
        readIndex = (readIndex + length) % BUFFER_SIZE;
    }

    uint8_t Command_GetLength()
    {
        return (writeIndex + BUFFER_SIZE - readIndex) % BUFFER_SIZE;
    } 

    uint8_t Command_GetRemain() 
    {
        // 减1是为了防止读写指针重合时无法区分是空还是满
        return BUFFER_SIZE - Command_GetLength() - 1;
    }
};


class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode()
    : Node("topic_helloworld_sub")      // ROS2节点父类初始化
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "RS485", 10, std::bind(&SubscriberNode::topic_callback, this, _1)); // 创建订阅者
    }

private:
    // [修改] 回调函数逻辑
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received raw data with size: %zu", msg->data.length());
        
        // 将收到的数据写入环形缓冲区
        // 注意：std_msgs::msg::String 的 data 是 std::string，可能包含\0，不适合用c_str()后计算长度
        // 我们直接使用 .data() 和 .length()
        ring_buffer_.Command_Write(reinterpret_cast<const uint8_t*>(msg->data.data()), msg->data.length());

        // 循环尝试从缓冲区中解析指令
        while(true)
        {
            uint8_t command_buffer[BUFFER_SIZE];
            uint8_t command_length = ring_buffer_.Command_GetCommand(command_buffer);

            if (command_length > 0)
            {
                // 如果解析成功，调用打印函数
                RCLCPP_INFO(this->get_logger(), "Successfully parsed a command!");
                ring_buffer_.Command_Print(command_buffer, command_length);
            }
            else
            {
                // 如果缓冲区中没有完整的指令了，就退出循环，等待下一批数据
                break;
            }
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; // 订阅者指针
    Buff_Ring ring_buffer_; // 添加一个环形缓冲区对象
};

// ROS2节点主入口main函数
int main(int argc, char * argv[])
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);
    
    // 创建ROS2节点对象并进行初始化
    rclcpp::spin(std::make_shared<SubscriberNode>());
    
    // 关闭ROS2 C++接口
    rclcpp::shutdown();
    
    return 0;
}


