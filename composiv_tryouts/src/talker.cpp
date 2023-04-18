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




// Talker object declaration
class talker
{
  public:

    // Constructed, destructer, initialization method and spin method are
    // constructer declaration
    talker();


    // destructer declaration
    ~talker();


    // declaration initialization method for ROS
    bool init();


    // ros spin method declaration
    void spin();


  protected:

    // declaration of protected callback function for the timer
    void timerCallback(const ros::TimerEvent& event);



  private:

    //declaration of node pointer
    ros::NodeHandle* node_handle_;



    //declaration of publisher object
    ros::Publisher talkerPub;



    //declaration of timer object
    ros::Timer talkerTimer;



    //default timer frequency
    double rate = 1;



    // default topic name
    std::string topic = "/talker";



    // default message
    std::string string_msg = "Hello";



    // the data to be published
    std_msgs::String msg;



    // node name
    std::string node_name = "talker";


};


// constructer definition
talker::talker():
  rate(1) {}



// destructer definition
talker::~talker()
{
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}



//node initialization definition
bool talker::init()
{

  /*

  Following code shows how the ROS node is initialized. Also, the
  name of the node is specified. Node names must be unique in a
  running system.

  */



  int argc = 0;
  char **argv = NULL;



  //ROS initialization
  ros::init(argc, argv, this->node_name);



  /*

  Also, the nodeHandle method is linked to the node_handle_ pointer
  to perferm specific tasks related with the node, such as subscribing
  to a topic, creating a timer, creating a publisher and etc.

  */



  //Node definition
  this->node_handle_ = new ros::NodeHandle(this->node_name);


  // Node Handler also allows us to retrieve ROS parameters as it is done 
  // in the following code.



  //Retrieving topic parameters
  if (  this->node_handle_->getParam("/" + this->node_name + "/topic", this->topic) )
  {
    // Topic parameter is assigned inside of the getParam(..) function. And the info 
    // is printed to console.
    ROS_INFO("Got topic param");
  }
  else
  {
    // If the ROS param does not exist, the default value is used.
    ROS_INFO("Couldn't find ros parameter for topic name. Going on with the default parameter, topic: %s", this->topic.c_str());
  }




  //Retrieving message parameter to be published
  if (  this->node_handle_->getParam("/" + this->node_name + "/message", this->string_msg) )
  {
    // Topic parameter is assigned inside of the getParam(..) function. And the info 
    // is printed to console.
    ROS_INFO("Got message param");
  }
  else
  {
    // If the ROS param does not exist, the default value is used.
    ROS_INFO("Couldn't find ros parameter for topic name. Going on with the default parameter, topic: %s", this->string_msg.c_str());
  }




  //Retrieving timer frequency parameter
  if ( this->node_handle_->getParam("/" + this->node_name + "/rate", this->rate) )
  {
    // Topic parameter is assigned inside of the getParam(..) function. And the info 
    // is printed to console.
    ROS_INFO("Got rate parameter.");
  }
  else
  {
    // If the ROS param does not exist, the default value is used.
    ROS_INFO("Couldn't find ros parameter for rate. Going on with the default parameter, rate: %f", this->rate);
  }



  //Constuction of publisher
  this->talkerPub = this->node_handle_->advertise<std_msgs::String>("/" + this->topic , 3);



  //Construction of timer
  this->talkerTimer = this->node_handle_->createTimer(ros::Duration(1/this->rate), &talker::timerCallback, this);



  return 1;
}


//Definition of spin method
void talker::spin()
{
  /*

    Also, a talker::spin() method is created to call spin() function. spin()
    function put the node into a loop.

  */

  ros::spin();

}



//Definition of callback function for the timer
void talker::timerCallback(const ros::TimerEvent& event)
{
  /*

     The talker::callbackFunction(const ros::TimerEvent& event) the callback function
     for the created timer. In the defined sampling period the timer is triggered and 
     the callback function is called. Then, the message is prepared and the published
     using talkerPub object and the publish method.

  */


  // the data attribute of the std_msgs::String is defined as the default message
  // or the data from ros param
  this->msg.data = this->string_msg;


  //the String object is published using publish method
  this->talkerPub.publish(this->msg);
}


int main(int argc, char **argv)
{


  //class object definition
  talker tk;


  //calling ros initialization method
  tk.init();


  //spinning the ROS node
  tk.spin();
  return 0;
}
