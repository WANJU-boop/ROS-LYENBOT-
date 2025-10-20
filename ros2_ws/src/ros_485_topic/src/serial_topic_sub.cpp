/***
@����: ���¾�(www.guyuehome.com) & Gemini
@˵��: ROS2����ʾ��-���ġ�Hello World��������Ϣ, ��ʹ�û��λ�����������ӡ
***/

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"              // ROS2 C++�ӿڿ�
#include "std_msgs/msg/string.hpp"        // �ַ�����Ϣ����

// ʹ�� std::placeholders::_1 ���󶨻ص�����
using std::placeholders::_1;

// ָ�����С����
#define COMMAND_MIN_LENGTH 4
// ѭ����������С
#define BUFFER_SIZE 128
// ѭ��������
uint8_t buffer[BUFFER_SIZE];
// ѭ��������������
uint8_t readIndex = 0;
// ѭ��������д����
uint8_t writeIndex = 0;


class Buff_Ring 
{
public:
    // ������д�뻷�λ�����
    uint8_t Command_Write(const uint8_t *data, uint8_t length)
    {
        if (Command_GetRemain() < length)
            return 0; // ʣ��ռ䲻��

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

    // �ӻ��λ������л�ȡһ��������ָ��
    uint8_t Command_GetCommand(uint8_t *command)
    {
        // ѭ��Ѱ������ָ��
        while (1)
        {
            // ������������ݳ���С����Сָ��ȣ��򲻿�����������ָ��
            if (Command_GetLength() < COMMAND_MIN_LENGTH)
                return 0;

            // ������ǰ�ͷ(0xAA)��������һ���ֽڣ����¿�ʼѰ��
            if (Command_Read(readIndex) != 0xAA)
            {
                Command_AddReadIndex(1);
                continue;
            }

            // ��ȡָ����ֽ�
            uint8_t length = Command_Read(readIndex + 1);
            if (length < COMMAND_MIN_LENGTH) // �����ĳ��ȼ��
            {
                Command_AddReadIndex(1);
                continue;
            }

            // ����������е����ݳ���С��ָ�����Ƶĳ��ȣ������ݲ�����
            if (Command_GetLength() < length)
            {
                return 0;
            }

            // ����У���
            uint8_t sum = 0;
            for (uint8_t i = 0; i < length - 1; i++) 
            {
                sum += Command_Read(readIndex + i);
            }

            // ���У��Ͳ���ȷ��������һ���ֽڣ����¿�ʼѰ��
            if (sum != Command_Read(readIndex + length - 1)) 
            {
                Command_AddReadIndex(1);
                continue;
            }

            // ����ҵ�����ָ���ָ��Ƶ� command ��������������ָ���
            for (uint8_t i = 0; i < length; i++) 
            {
                command[i] = Command_Read(readIndex + i);
            }
            Command_AddReadIndex(length);
            return length;
        }
    }

    // [����] ��ӡָ�����ݵĺ���
    void Command_Print(uint8_t *command, uint8_t length)
    {
        if (length == 0) return;
        
        // ʹ�� RCLCPP_INFO ����ӡ��������ROS2�ķ��
        // ��Ҫһ��Node��Loggerʵ����������ʱ��printf
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
    
    // [����] ������ƴд������﷨
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
        // ��1��Ϊ�˷�ֹ��дָ���غ�ʱ�޷������ǿջ�����
        return BUFFER_SIZE - Command_GetLength() - 1;
    }
};


class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode()
    : Node("topic_helloworld_sub")      // ROS2�ڵ㸸���ʼ��
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "RS485", 10, std::bind(&SubscriberNode::topic_callback, this, _1)); // ����������
    }

private:
    // [�޸�] �ص������߼�
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received raw data with size: %zu", msg->data.length());
        
        // ���յ�������д�뻷�λ�����
        // ע�⣺std_msgs::msg::String �� data �� std::string�����ܰ���\0�����ʺ���c_str()����㳤��
        // ����ֱ��ʹ�� .data() �� .length()
        ring_buffer_.Command_Write(reinterpret_cast<const uint8_t*>(msg->data.data()), msg->data.length());

        // ѭ�����Դӻ������н���ָ��
        while(true)
        {
            uint8_t command_buffer[BUFFER_SIZE];
            uint8_t command_length = ring_buffer_.Command_GetCommand(command_buffer);

            if (command_length > 0)
            {
                // ��������ɹ������ô�ӡ����
                RCLCPP_INFO(this->get_logger(), "Successfully parsed a command!");
                ring_buffer_.Command_Print(command_buffer, command_length);
            }
            else
            {
                // �����������û��������ָ���ˣ����˳�ѭ�����ȴ���һ������
                break;
            }
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_; // ������ָ��
    Buff_Ring ring_buffer_; // ���һ�����λ���������
};

// ROS2�ڵ������main����
int main(int argc, char * argv[])
{
    // ROS2 C++�ӿڳ�ʼ��
    rclcpp::init(argc, argv);
    
    // ����ROS2�ڵ���󲢽��г�ʼ��
    rclcpp::spin(std::make_shared<SubscriberNode>());
    
    // �ر�ROS2 C++�ӿ�
    rclcpp::shutdown();
    
    return 0;
}


