#include <cstdio>
extern "C"{
  #include "uwb_robot_algo.h"
  #include "uart_stack.h"
}

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "rclcpp/rclcpp.hpp"
#include "uwb_aoa_pkg/lib_aoa_robot_msg.hpp"
#include "uwb_aoa_pkg/uwb_robot_heart_msg.hpp"
#include "uwb_aoa_pkg/uwb_robot_alive_sentry_msg.hpp"
#include "uwb_aoa_pkg/uwb_robot_pos_raw_msg.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int8.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

//V1.0.2

class libAoa_robot_publisher : public rclcpp::Node
{
  public:
    libAoa_robot_publisher(const char* dev_name)
    : Node("uwb_robot_publisher"),running_(false)
    {

      publisher_ = this->create_publisher<uwb_aoa_pkg::msg::LibAoaRobotMsg>("libAoa_robot_publisher", 10);
      heartpublisher_ = this->create_publisher<uwb_aoa_pkg::msg::UwbRobotHeartMsg>("uwb_robot_heart_publisher",10);
      alivepublisher_ = this->create_publisher<uwb_aoa_pkg::msg::UwbRobotAliveSentryMsg>("uwb_robot_alive_sentry_publisher",10);
      rawdatapublisher_ = this->create_publisher<uwb_aoa_pkg::msg::UwbRobotPosRawMsg>("uwb_robot_pos_rawdata_publisher",10);
      ble_follow_pub_ = this->create_publisher<std_msgs::msg::Int8>("/ble_follow_cmd", 10);
      
      service_ = this->create_service<std_srvs::srv::SetBool>("/uwb_robot_start_radar",std::bind(&libAoa_robot_publisher::handle_request,this,_1,_2));
      
	  try 
	  {

		serial_port_ = open(dev_name, O_RDWR); // linux
		struct termios tty;
		if (tcgetattr(serial_port_, &tty) != 0)
		{
			printf("Error! Check uart connection/permission!\n");
			printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
			return;
		}

		tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
		tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
		tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
		tty.c_cflag |= CS8;            // 8 bits per byte (most common)
		tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
		tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

		tty.c_lflag &= ~ICANON;
		tty.c_lflag &= ~ECHO;                                                        // Disable echo
		tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
		tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
		tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
		tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
		tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

		tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
		tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
		// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
		// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

		tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
		tty.c_cc[VMIN] = 0;

		// Set in/out baud rate to be 9600
		cfsetispeed(&tty, B115200);
		cfsetospeed(&tty, B115200);

		// Save tty settings, also checking for error
		if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
		{
			printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
			return;
		}

		RCLCPP_INFO(this->get_logger(),"open port success: %d\n",serial_port_);
	  } 
	  catch (const std::exception& e)
	  {
		RCLCPP_FATAL(this->get_logger(),"can't open port %s",e.what());
		return;
	  }

	  running_ = true;
	  read_thread_ = std::thread(&libAoa_robot_publisher::readSerialLoop,this);
	  RCLCPP_INFO(this->get_logger(),"start serial publish");
    }

	~libAoa_robot_publisher(){
		running_ = false;
		if (read_thread_.joinable()) read_thread_.join();
		close(serial_port_);
	}

  private:
  
  	void handle_request(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  	std::shared_ptr<std_srvs::srv::SetBool::Response> response){
  		bool flag = request->data;

  		if (flag){
  		    uint8_t radar_start[UWB_RADAR_LEN] = START_UWB_RADAR;
  		    write(serial_port_,radar_start,sizeof(radar_start));
  		    response->success = true;
  		    response->message = "start uwb radar";
  		} else {
  		    uint8_t radar_start[UWB_RADAR_LEN] = CLOSE_UWB_RADAR;
  		    write(serial_port_,radar_start,sizeof(radar_start));
  		    response->success = true;
  		    response->message = "close uwb radar";
  		}
  		
  	}

    void ble_detect(uint8_t byte) {
        static const int BUF_SIZE = 10;
        static uint8_t buf[BUF_SIZE] = {0};
        static int pos = 0;

        buf[pos % BUF_SIZE] = byte;
        pos++;

        if (pos < 5) return;

        int start = (pos - 5) % BUF_SIZE;
        if (buf[start] != 0x11) return;
        if (buf[(start + 1) % BUF_SIZE] != 0x01) return;

        uint16_t length = (buf[(start + 2) % BUF_SIZE] << 8)
                        | buf[(start + 3) % BUF_SIZE];
        if (length < 1) return;

        if (pos < 5 + (int)length) return;

        uint8_t cmd = buf[(start + 4) % BUF_SIZE];
        auto msg = std_msgs::msg::Int8();

        switch (cmd) {
            case 0x01: msg.data = 0;  printf(">>> 开始跟随 (BLE)\n"); break;
            case 0x02: msg.data = 1;  printf("<<< 停止跟随 (BLE)\n"); break;
            case 0x03: msg.data = 2;  printf("--- 返回信令处 (BLE)\n"); break;
            case 0x04: msg.data = 3;  printf("--- 暂停/恢复 (BLE)\n"); break;
            default: return;
        }
        ble_follow_pub_->publish(msg);
        pos = 0;
    }
     
	void readSerialLoop(){

		auto message = uwb_aoa_pkg::msg::LibAoaRobotMsg();
		auto heart_message = uwb_aoa_pkg::msg::UwbRobotHeartMsg();
		auto alive_message = uwb_aoa_pkg::msg::UwbRobotAliveSentryMsg();
		auto rawdata_message = uwb_aoa_pkg::msg::UwbRobotPosRawMsg();

		uint8_t read_buf[2];
		memset(&read_buf, '\0', sizeof(read_buf));

		struct input_data my_input;
		struct output my_output;

		my_input.aoa_frequency = 20;
		my_input.clean_buffer_time = 2;
		my_input.config_r = 0;
		my_input.config_rad = 0; // 角度偏移
		my_input.direction = 1;
		my_input.stopband_rate = 0.1;
		my_input.t = 0;
		my_input.Azimuth = 0;
		my_input.Distance = 0;
		my_input.Pitch = 0;
		my_input.State = 1;
		my_input.StationXOffset = 0;
		my_input.StationYOffset = 0;
		my_input.win_fil = 0;
		my_input.use_pitch_dis = 0;

		my_output.r = 0; my_output.rad = 0; my_output.state = 0;
		my_output.x = 0; my_output.y = 0;

		int c5_len = sizeof(uwb_aoa_fob_pkg_t);
		int merge_len = sizeof(struct output);
		uint8_t *c5_buffer = (uint8_t *)malloc(c5_len + 2);
		uint8_t *merge_buffer = (uint8_t *)malloc(merge_len + 2);
		c5_buffer[0] = 0xc5;
		c5_buffer[1] = c5_len;
		merge_buffer[0] = 0xc6;
		merge_buffer[1] = merge_len;
		
		while (rclcpp::ok() && running_){
			int num_bytes = read(serial_port_, &read_buf, 1);
			if (num_bytes > 0)
			{
				ble_detect(read_buf[0]);
				//printf("%02x", read_buf[0]);
				int ret = uart_receive_byte(read_buf[0]);
				if (ret == 1) {
					printf("\n");

					// uint8_t cmd = uart_protocol_packet_cmd();
					
					void *content_p = NULL;

					uint8_t parse_cmd = uart_protocol_packet_process((void **)&content_p);
					
					if (parse_cmd == 0xc5) {
						uwb_aoa_fob_pkg_t *c5 = static_cast<uwb_aoa_fob_pkg_t*>(content_p);
						
						//printf("[UWB12]: dis: %.3f, agl: %.3f, pitch: %.3f\r\n", c5->distance, c5->angle, c5->pitch);

						time_t start_time = time(NULL);

						my_input.Distance = c5->distance; // update dis
						my_input.Azimuth = c5->angle; //update azi 
						my_input.Pitch = c5->pitch; // update pitch
						my_input.Pos_confidence = c5->pos_confidence; // update pos_confidence
						memcpy(my_input.Uwb_rssi,c5->rssi,6*sizeof(uint8_t));
						my_input.t = start_time; // update time(uint32)
						algo_uwb_aoa_merge(&my_input, &my_output);
						printf("output_result: r:%f,a:%f,x:%f,y:%f,rssi:%d,pos_confi:%d,state:%d\n\n", my_output.r, my_output.rad,my_output.x,my_output.y, my_output.uwb_rssi,my_output.pos_confidence,my_output.state);


						message.x = my_output.x;
						message.y = my_output.y;
						message.r = my_output.r;
						message.a = my_output.rad*180/3.1415926535;
						message.pitch = c5->pitch;
						message.rssi_uwb = my_output.uwb_rssi;
						message.rssi_ble = c5->rssi_ble;
						message.pos_confidence = my_output.pos_confidence;
					
						publisher_->publish(message);
						
						rawdata_message.distance = c5->distance;
						rawdata_message.angle = c5->angle;
						rawdata_message.pitch = c5->pitch;
						rawdata_message.pos_confidence = c5->pos_confidence;
						rawdata_message.rssi_ble = c5->rssi_ble;
						rawdata_message.rssi_uwb = c5->rssi[0];
						
						rawdatapublisher_->publish(rawdata_message);
						
					} else if (parse_cmd == 0x59) {
						uwb_aoa_fob_notify_heart_t *heart = static_cast<uwb_aoa_fob_notify_heart_t*>(content_p);
						printf("heart_cnt:%d,radar_status:%d,range_status:%d\n", heart->heart_cnt, heart->radar_status, heart->range_status);
						heart_message.heart_cnt = heart->heart_cnt;
						heart_message.range_status = heart->range_status;
						heart_message.radar_status = heart->radar_status;
						heartpublisher_->publish(heart_message);
					} else if (parse_cmd == 0x54) {
						uwb_aoa_notify_alive_sentry_status_t *alive = static_cast<uwb_aoa_notify_alive_sentry_status_t*>(content_p);			 
						printf("is_have_living_body:%d,max_radar_v:%f,living_body_first_index_dis_cm:%d\n", alive->is_have_living_body, alive->max_radar_v, alive->living_body_first_index_dis*15);
						alive_message.is_have_living_body = alive->is_have_living_body;
						alive_message.max_radar_v = alive->max_radar_v;
						alive_message.living_body_first_index_dis = alive->living_body_first_index_dis*15;
						alivepublisher_->publish(alive_message);
					}  else if (parse_cmd == 0xc8) {
						uwb_aoa_c8_pkg_t *c8 = static_cast<uwb_aoa_c8_pkg_t*>(content_p);
						printf("c8 result: r:%f,a:%f\n", c8->distance,c8->angle);
						
						message.x = c8->x;
						message.y = c8->y;
						message.r = c8->distance;
						message.a = c8->angle;
						message.pitch = c8->pitch;
						message.rssi_uwb = c8->uwb_rssi;
						message.rssi_ble = c8->ble_rssi;
						message.pos_confidence = c8->pos_confidence;
						
						publisher_->publish(message);
					}
					
				}
			}
		}
	}

    rclcpp::Publisher<uwb_aoa_pkg::msg::LibAoaRobotMsg>::SharedPtr publisher_;
    rclcpp::Publisher<uwb_aoa_pkg::msg::UwbRobotHeartMsg>::SharedPtr heartpublisher_;
    rclcpp::Publisher<uwb_aoa_pkg::msg::UwbRobotAliveSentryMsg>::SharedPtr alivepublisher_;
    rclcpp::Publisher<uwb_aoa_pkg::msg::UwbRobotPosRawMsg>::SharedPtr rawdatapublisher_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr ble_follow_pub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    std::thread read_thread_;
    std::atomic<bool> running_;
    int serial_port_;

    // rclcpp::TimerBase::SharedPtr timer_;
    // size_t count_;
};



int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
	
	rclcpp::init(argc, argv);
	
  	rclcpp::spin(std::make_shared<libAoa_robot_publisher>(argv[1]));
 	rclcpp::shutdown();
	
	return 0;
}
