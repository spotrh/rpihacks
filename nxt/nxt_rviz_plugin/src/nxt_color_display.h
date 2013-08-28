#ifndef NXT_COLOR_DISPLAY_H
#define NXT_COLOR_DISPLAY_H

#include "rviz/display.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"

#include <nxt_msgs/Color.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <boost/shared_ptr.hpp>

namespace ogre_tools
{
class Shape;
}


namespace Ogre
{
class SceneNode;
}

namespace nxt_rviz_plugin
{

/**
 * \class NXTColorDisplay
 * \brief Displays a nxt_msgs::Color message
 */
class NXTColorDisplay : public rviz::Display
{
public:
  NXTColorDisplay( const std::string& name, rviz::VisualizationManager* manager );
  virtual ~NXTColorDisplay();

  void setTopic( const std::string& topic );
  const std::string& getTopic() { return topic_; }

  void setAlpha( float alpha );
  float getAlpha() { return alpha_; }

  void setDisplayLength( float displayLength );
  float getDisplayLength() { return displayLength_; }

  // Overrides from Display
  virtual void targetFrameChanged() {}
  virtual void fixedFrameChanged();
  virtual void createProperties();
  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

  static const char* getTypeStatic() { return "Color"; }
  virtual const char* getType() const { return getTypeStatic(); }
  static const char* getDescription();

protected:
  void subscribe();
  void unsubscribe();
  void clear();
  void incomingMessage(const nxt_msgs::Color::ConstPtr& msg);
  void processMessage(const nxt_msgs::Color::ConstPtr& msg);

  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  std::string topic_;
  float alpha_;
  float displayLength_;

  uint32_t messages_received_;

  Ogre::SceneNode* scene_node_;
  ogre_tools::Shape* cylinder_;      ///< Handles actually drawing the cone

  message_filters::Subscriber<nxt_msgs::Color> sub_;
  tf::MessageFilter<nxt_msgs::Color> tf_filter_;
  nxt_msgs::Color::ConstPtr current_message_;

  rviz::ROSTopicStringPropertyWPtr topic_property_;
  rviz::FloatPropertyWPtr alpha_property_;
  rviz::FloatPropertyWPtr display_property_;
};

} // namespace nxt_rviz_plugin

#endif /* NXT_COLOR_DISPLAY_H */

