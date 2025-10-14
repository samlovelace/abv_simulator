#ifndef ROS_TOPIC_MANAGER_H
#define ROS_TOPIC_MANAGER_H

#include <rclcpp/rclcpp.hpp>
#include "robot_idl/msg/abv_command.hpp"
#include "robot_idl/msg/abv_response.hpp"
#include "robot_idl/msg/abv_state.hpp"
#include <map>

class RosTopicManager : public rclcpp::Node
{
public:
    RosTopicManager() : rclcpp::Node("simulator") {}
    ~RosTopicManager() {}

    template<typename T>
    void createPublisher(const std::string& topicName) {
        auto publisher = this->create_publisher<T>(topicName, 10);
        mPublishers[topicName] = std::dynamic_pointer_cast<rclcpp::PublisherBase>(publisher);
    }

    void spinNode()
    {
        std::thread([this]() {
            rclcpp::spin(this->get_node_base_interface());
        }).detach();
    }

    template<typename T>
    void publishMessage(const std::string& topicName, const T& message) 
    {
        auto it = mPublishers.find(topicName);
        
        if (it != mPublishers.end()) 
        {
            // Cast PublisherBase back to Publisher<T>
            auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(it->second);
            if (pub) 
            {
                pub->publish(message);
            } 
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to cast publisher for topic: %s", topicName.c_str());
            }
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Publisher not found for topic: %s", topicName.c_str());
        }
    }

    template<typename T>
    void createSubscriber(const std::string& aTopicName, std::function<void(const typename T::SharedPtr)> aCallback)
    {
        auto subscriber = this->create_subscription<T>(aTopicName, 10, aCallback);
        mSubscribers[aTopicName] = std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscriber);
    }

private: 

    std::map<std::string, std::shared_ptr<rclcpp::PublisherBase>> mPublishers;
    std::map<std::string, std::shared_ptr<rclcpp::SubscriptionBase>> mSubscribers;

};

#endif // ROS_TOPIC_MANAGER_H