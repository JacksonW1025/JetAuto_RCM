#ifndef __VOICE_CONTROL_H_
#define __VOICE_CONTROL_H_

#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <sys/stat.h>
#include <record_param.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include "user_interface.h"
#include "ros_robot_controller_msgs/msg/buzzer_state.hpp"
#include "xf_mic_asr_offline_msgs/srv/get_offline_result.hpp"
using namespace std;

extern UserData asr_data;
extern int whether_finised;
extern char *whole_result;
unsigned char* record_data;

class  SpeechProcess : public rclcpp::Node{
public:
	SpeechProcess(const std::string &node_name);
	~SpeechProcess();
	void run();

private:
	int confidence;
	float time_per_order;
    const char str_ = '|';
    const char str_none = ' ';

	rclcpp::Time start_time,last_time;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_words_pub;
	rclcpp::Publisher<ros_robot_controller_msgs::msg::BuzzerState>::SharedPtr buzzer_pub;
    rclcpp::Service<xf_mic_asr_offline_msgs::srv::GetOfflineResult>::SharedPtr get_offline_result_srv_;

	std::string s2s(const std::string &str);
	Effective_Result show_result(char* str);

	int filesize(const char *fname);
	void finish_record_sound();
	int init_asr_params();
	void get_record_sound(const char *file);
	void business_data_t(unsigned char* record);
	int record_params_init(record_handle_t* pcm_handle,record_params_t* params);
	static snd_pcm_format_t get_formattype_from_params(record_params_t* params);

	bool Get_Offline_Recognise_Result(const std::shared_ptr<xf_mic_asr_offline_msgs::srv::GetOfflineResult::Request>& request,
										std::shared_ptr<xf_mic_asr_offline_msgs::srv::GetOfflineResult::Response>& response);
};

#endif
