/*

"listener.cpp" is a C++ code for a listener node in this ROS package.
It subscribe to a topic defined by a ros param defined in launch file.

*/



#include "ros/ros.h"
#include <std_msgs/String.h>



/*

This lines includes all the headers necessary to use the most common
public pieces of the ROS system and the std_msgs/String message, which
resides in the std_msgs package.

*/




// listener object declaration
class listener
{

  public:

    // Constructed, destructer, initialization method and spin method are
    // defined as public.

    // constructer declaration
    listener();



    // destructer declaration
    ~listener();



    // initialization method for ROS
    bool init();



    // ros spin method
    void spin();



  protected:
    // declaration of callback function for the subscriber
    void callbackFunction(const std_msgs::String::ConstPtr& msg);



  private:

    // node pointer declaration
    ros::NodeHandle* node_handle_;



    // declaration of node object
    ros::Subscriber listenerSub;



    // global private variable declarations
    std::string topic = "/talker";



    // the variable that the message data will be stored
    std::string string_data;



    // node name
    std::string node_name = "listener";

};




// contructer definition
listener::listener():
  topic("/talker") {}




// destructive definition
listener::~listener()
{
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}



// initialization method definition
bool listener::init()
{


  int argc = 0;
  char **argv = NULL;


  /*

  Following code shows how the ROS node is initialized. Also, the
  name of the node is specified. Node names must be unique in a
  running system.

  */


  //ROS initialization
  ros::init(argc, argv, this->node_name);


  /*

  Also, the nodeHandle method is linked to the node_handle_ pointer
  to perferm specific tasks related with the node, such as subscribing
  to a topic, creating a timer, creating a publisher and etc.

  */


  // node pointer definition
  this->node_handle_ = new ros::NodeHandle(this->node_name);



  // Node Handler also allows us to retrieve ROS parameters as it is done 
  // in the following code.
  if ( this->node_handle_->getParam("/" + this->node_name + "/topic", this->topic) )
  {
    // Parameter are assigned inside of the getParam(..) function. And the info 
    // is printed to console.
    ROS_INFO("Got param");
  }
  else
  {
    // If the ROS param does not exist, the default value is used.
    ROS_INFO("FAILED listener");
  }



  // Subscriber can be created as shown below. The callback function
  // is pointed to pass into the function as it is defined as
  //  listener::callbackFunction.
  this->listenerSub = this->node_handle_->subscribe(this->topic, 4, &listener::callbackFunction, this);

  // Printing initializing info to console.
  ROS_INFO("Listener Node has been initialized!");

  return 1;
}



// defining spin method
void listener::spin()
{
  // A listener::spin() method is created to call spin() function.
  // spin() function put the node into a loop.
  ros::spin();
}



// defining callback function method
void listener::callbackFunction(const std_msgs::String::ConstPtr& msg)
{
  // The listener::callbackFunction(const std_msgs::String::ConstPtr& msg)
  // takes the pointer of the defined message and assing its data to the
  // predefined variable which is this->string_data.

  // obtaining data from msg pointer
  this->string_data = msg->data;

  // printing the data to console
  ROS_INFO("[listener_node] Retrieved message: %s" , this->string_data.c_str());
}




//defining main function
int main(int argc, char **argv)
{
  // Finally, in the main function, the listener object is defined as lstnr,
  // the init() method is called and the node is spinned.

  // listener object definition
  listener lstnr;

  // calling the method for ROS and Node initialization
  lstnr.init();

  // spinning the ROS Node
  lstnr.spin();
  return 0;
}


