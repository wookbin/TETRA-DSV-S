#include "create_restrict.h"
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <ros/console.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>
#include "rviz/display_context.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/bool_property.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz_custom::Restrict, rviz::Tool)

namespace rviz_custom {

Restrict::Restrict() {

    //shortcut_key_ = 'g';

    Size_property_ = new rviz::IntProperty("Wall_num", 1,"Select of wall number",
                                          getPropertyContainer(), SLOT(updateTopic()), this);

    Remove_all_ = new rviz::BoolProperty("Clear", false, "State of Clear or Draw ",
                                         getPropertyContainer(), SLOT(updateTopic()), this);
}

Restrict::~Restrict() {}

void Restrict::updateTopic()
{
    settings_.x = Size_property_->getInt();
    settings_.y = Remove_all_->getBool(); //clear
    settings_.z = 1; //color

    // // update topic
    // set_rectangle_.x = point_.x;
    // set_rectangle_.y = point_.y;
    // set_rectangle_.z = 0.0;
    
}

void Restrict::initialize()
{
    pub_  = n_.advertise<geometry_msgs::Point>("/mouse_location",1);
    pub2_ = n_.advertise<geometry_msgs::Point>("/restrict_settings",1);
    pub3_ = n_.advertise<geometry_msgs::Point>("/wall_rectangle_data",1);

}
void Restrict::activate()
{
    initialize();
    ROS_INFO("Initialize is starting!");
    operation_=false;
    set_rectangle_.z = 0.0;
}

void Restrict::deactivate(){}

int Restrict::processMouseEvent(rviz::ViewportMouseEvent& event)
{
    int flag = 0;
    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );
    try
    {
        if( rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection ))
            {
                set_rectangle_.x = point_.x = intersection.x;
                set_rectangle_.y = point_.y = intersection.y;
                point_.z = operation_;
                pub_.publish(point_);
                //pub2_.publish(settings_);
            }
    }
    catch (int a)
    {
        ROS_ERROR("Error Occured!!");
        return 0;
    }

    if (event.rightUp())
    {
        ROS_INFO("event.leftDown !!");
        operation_=true;

        set_rectangle_.z++;
        pub3_.publish(set_rectangle_);
    }
    else if( event.leftDown() )
    {
        set_rectangle_.z = 0.0;
        ROS_INFO("event.leftDown !!");
        pub2_.publish(settings_);
        flag |= Finished;
    }

    return flag;
}

}//end namespace

