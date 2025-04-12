#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <rcutils/allocator.h>
#include <rclc/rclc.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <SCServo.h>

SMS_STS st;

#define S_RXD 18    
#define S_TXD 19

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 6000;

// ROS variables
rcl_subscription_t servo_subscriber;
std_msgs__msg__Float32MultiArray servo_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {} }

// Error loop
void error_loop() { while (1) { delay(100); } }

// Callback for Servo angle messages
void servo_callback(const void *msgin) {
    const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msgin;

    for (size_t i = 0; i < msg->data.size; i++) {
    }

    if (msg->data.size >= 2) {
        int first = constrain((int)msg->data.data[0], MIN_ANGLE, MAX_ANGLE);
        int second = constrain((int)msg->data.data[1], MIN_ANGLE, MAX_ANGLE);


        st.WritePosEx(1, first, 250, 50);  // Move Servo 1
        st.WritePosEx(2, second, 250, 50); // Move Servo 2
    }
}

void setup() {
    
    set_microros_transports();
    delay(1000);

    Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
    if (!Serial1) {
        error_loop();
    }
    st.pSerial = &Serial1;
    delay(2000);

    while (!rmw_uros_ping_agent(100, 1)) {
        delay(1000);
    }
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_servo_node", "", &support));

    // Initialize the message memory
    servo_msg.data.capacity = 2;
    servo_msg.data.size = 0;
    static float data_buffer[2];
    servo_msg.data.data = data_buffer;
    if (servo_msg.data.data == NULL) {
        error_loop();
    }

    // Create Servo subscriber
    RCCHECK(rclc_subscription_init_default(
        &servo_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "Servo"
    ));

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);
}
