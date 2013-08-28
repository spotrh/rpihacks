#include "nxt_color_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <ogre_tools/shape.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace nxt_rviz_plugin
{
NXTColorDisplay::NXTColorDisplay( const std::string& name, rviz::VisualizationManager* manager )
  : Display( name, manager )
  , messages_received_(0)
  , tf_filter_(*manager->getTFClient(), "", 10, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  cylinder_ = new ogre_tools::Shape(ogre_tools::Shape::Cylinder, vis_manager_->getSceneManager(), scene_node_);

  scene_node_->setVisible( false );

  setAlpha( 0.5f );
  setDisplayLength( 0.003f );

  Ogre::Vector3 scale( 0, 0, 0);
  rviz::scaleRobotToOgre( scale );
  cylinder_->setScale(scale);

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&NXTColorDisplay::incomingMessage, this, _1));
  vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
}

NXTColorDisplay::~NXTColorDisplay()
{
  unsubscribe();
  clear();
  delete cylinder_;
}

void NXTColorDisplay::clear()
{

  messages_received_ = 0;
  setStatus(rviz::status_levels::Warn, "Topic", "No messages received");
}

void NXTColorDisplay::setTopic( const std::string& topic )
{
  unsubscribe();

  topic_ = topic;

  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void NXTColorDisplay::setAlpha( float alpha )
{
  alpha_ = alpha;

  propertyChanged(alpha_property_);

  processMessage(current_message_);
  causeRender();
}

void NXTColorDisplay::setDisplayLength( float displayLength )
{
  displayLength_ = displayLength;

  propertyChanged(display_property_);

  processMessage(current_message_);
  causeRender();
}


void NXTColorDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  sub_.subscribe(update_nh_, topic_, 10);
}

void NXTColorDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void NXTColorDisplay::onEnable()
{
  scene_node_->setVisible( true );
  subscribe();
}

void NXTColorDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible( false );
}

void NXTColorDisplay::fixedFrameChanged()
{
  clear();

  tf_filter_.setTargetFrame( fixed_frame_ );
}

void NXTColorDisplay::update(float wall_dt, float ros_dt)
{
}


void NXTColorDisplay::processMessage(const nxt_msgs::Color::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }

  ++messages_received_;

  {
    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus(rviz::status_levels::Ok, "Topic", ss.str());
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  geometry_msgs::Pose pose;
  pose.position.z = -0.0033;
  pose.position.y = 0;
  pose.position.x = 0.0185 + displayLength_/2;
  pose.orientation.x = 0.707;
  pose.orientation.z = -0.707;
  if (!vis_manager_->getFrameManager()->transform(msg->header.frame_id,msg->header.stamp,pose, position, orientation, true))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.c_str() );
  }

  cylinder_->setPosition(position);
  cylinder_->setOrientation(orientation);
  Ogre::Vector3 scale( 0.0155, 0.0155, displayLength_);
  rviz::scaleRobotToOgre( scale );
  cylinder_->setScale(scale);
  cylinder_->setColor(msg->r, msg->g, msg->b, alpha_);

}

void NXTColorDisplay::incomingMessage(const nxt_msgs::Color::ConstPtr& msg)
{
  processMessage(msg);
}

void NXTColorDisplay::reset()
{
  Display::reset();
  clear();
}

void NXTColorDisplay::createProperties()
{
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &NXTColorDisplay::getTopic, this ),
                                                                                     boost::bind( &NXTColorDisplay::setTopic, this, _1 ), parent_category_, this );
  setPropertyHelpText(topic_property_, "nxt_msgs::Color topic to subscribe to.");
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ros::message_traits::datatype<nxt_msgs::Color>());


  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Alpha", property_prefix_, boost::bind( &NXTColorDisplay::getAlpha, this ),
                                                                            boost::bind( &NXTColorDisplay::setAlpha, this, _1 ), parent_category_, this );

  display_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Display Length", property_prefix_, boost::bind( &NXTColorDisplay::getDisplayLength, this ),
                                                                            boost::bind( &NXTColorDisplay::setDisplayLength, this, _1 ), parent_category_, this );

  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the circle.");
}

const char* NXTColorDisplay::getDescription()
{
  return "Displays data from a nxt_msgs::Color message as a cirle of color.";
}
} // namespace nxt_rviz_plugin    
