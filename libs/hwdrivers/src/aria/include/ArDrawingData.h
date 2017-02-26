/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARDRAWINGDATA_H
#define ARDRAWINGDATA_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"

/// A class for holding color information for ArDrawingData
class ArColor
{
public:
  /// Constructor (colors use full range of 0-255)
  ArColor(unsigned char red, unsigned char green, unsigned char blue)
    { myRed = red; myGreen = green; myBlue = blue; }
  /// Constructor
  ArColor() { myRed = 255, myGreen = 255, myBlue = 255; }

  /// Constructs a color from the given RGB value
  ArColor(ArTypes::Byte4 rgbValue)
  {
    myRed   = (rgbValue & 0xFF0000) >> 16;
    myGreen = (rgbValue & 0x00FF00) >> 8;
    myBlue  = (rgbValue & 0x0000FF);
  }

  /// Destructor
  virtual ~ArColor() {}
  /// Gets the red value (uses full range of 0-255)
  unsigned char getRed(void) { return myRed; }
  /// Gets the green value (uses full range of 0-255)
  unsigned char getGreen(void) { return myGreen; }
  /// Gets the blue value (uses full range of 0-255)
  unsigned char getBlue(void) { return myBlue; }
  /// Gets the color in a byte 4 for putting into a buffer
  ArTypes::Byte4 colorToByte4(void)
    { return ((myRed << 16) | (myGreen << 8) | myBlue); }
protected:
  unsigned char myRed;
  unsigned char myGreen;
  unsigned char myBlue;

};


/** Describes general properties of a figure to be drawn on screen.
 *  (The actual location/geometry data of the figure is stored elsewhere and may
 *  change frequently)
 *
 * The following shapes are currently recognized:
 * <ul>
 *     <li>"polyDots" - a set of small ellipses centered on each point.  Each
 *                      ellipse is "size" mm in diameter.  (Example: laser) </li>
 *	   <li>"polyArrows" - a set of small arrows that terminate on each point and
 *	                    point towards the robot.
 *                      Each arrow is "size" mm in length. (Example: sonar) </li>
 *     <li>"polyLine" - a line through each of the points.  The line is "size"
 *                      pixels wide.  (Example: path) </li>
 *     <li>"polyPoints" - a set of one pixel points.  (Example: localization)</li>
 *     <li>"polySegments" - a set of line segments.  For an array of n points,
 *                      n/2 segments are drawn.  Each segment is drawn from
 *                      array[i] to array[i+1] (with i starting at 0).  The
 *                      segments are "size" pixels wide.  </li>
 * </ul>
 *
 * The layer is an arbitrary int identifier that must be greater than 30 and
 * less than 100.  (Layers below 30 are considered part of the map data, i.e.
 * not associated with a particular robot.)  The robot is drawn on layer 50.
 * Any items that are to be drawn on top of the robot should have a layer
 * number greater than 50.  By default, range devices begin on layer 70.
 *
 * The drawing data also contains a "visibility" attribute that specifies
 * whether the data is to be displayed by default, and whether the user is allowed
 * to change the display.  The following visibilities are currently supported:
 * <ul>
 *  <li> "AlwaysOn"   - item is always displayed </li>
 *  <li> "DefaultOn"  - item is visible by default, but the user may hide it</li>
 *  <li> "DefaultOff" - item is hidden by default but the user may show it </li>
 *  <li> "AlwaysOff"  - item is always hidden</li>
 * </ul>
 * The default visibility is "DefaultOn".
 *
**/
class ArDrawingData
{
public:

  enum {
    DEFAULT_REFRESH_TIME = 200 ///< Default number of ms between data refresh requests
  };


  /// Constructor
  /**
     @param shape the name of the shape to draw  (see above / MobileEyes docs for meaning)
     @param primaryColor the main color (meaning depends on shape)
     @param size the size (meaning varies depends on shape)
     @param layer the layer to draw on (see above / MobileEyes docs for meaning)
     @param defaultRefreshTime how often we want to draw it in ms
     @param visibility a string that indicates whether the data is visible
            whether the user is allowed to change the visibility (see above / MobileEyes
            docs for valid values).
   **/
  /*AREXPORT*/ArDrawingData(const char *shape,
			                  ArColor primaryColor,
			                  int size,
			                  int layer,
			                  unsigned int defaultRefreshTime = DEFAULT_REFRESH_TIME,
                        const char *visibility = "DefaultOn") :
    myShape(shape),
    myPrimaryColor(primaryColor),
    mySize(size),
    myLayer(layer),
    myDefaultRefreshTime(defaultRefreshTime),
    mySecondaryColor(ArColor(0,0,0)),
    myVisibility(visibility)
  {
  }

  /// Constructor
  /**
     @param shape the name of the shape to draw  (see above / MobileEyes docs for meaning)
     @param primaryColor the main color (meaning depends on shape)
     @param size the size (meaning varies depends on shape)
     @param layer the layer to draw on (see above / MobileEyes docs for meaning)
     @param defaultRefreshTime how often we want to draw it in ms
     @param secondaryColor the secondary color (meaning depends on shape)
     @param visibility a string that indicates whether the data is visible
            whether the user is allowed to change the visibility (see above / MobileEyes
            docs for valid values).
   **/
  /*AREXPORT*/ArDrawingData(const char *shape,
			 ArColor primaryColor,
			 int size,
			 int layer,
			 unsigned int defaultRefreshTime,
			 ArColor secondaryColor,
       const char *visibility = "DefaultOn")
    {
      myShape = shape;
      myPrimaryColor = primaryColor;
      mySize = size;
      myLayer = layer;
      myDefaultRefreshTime = defaultRefreshTime;
      mySecondaryColor = secondaryColor;
      myVisibility = visibility;
    }
  /// Destructor
  /*AREXPORT*/ virtual ~ArDrawingData() {}
  /// Returns the shape of data to draw
  const char * getShape(void) { return myShape.c_str(); }
  /// Gets the primary color (meaning depending on shape)
  ArColor getPrimaryColor(void) { return myPrimaryColor; }
  /// Gets the size (meaning depends on shape, but its in mm)
  int getSize(void) { return mySize; }
  /// Gets the layer to draw at (see MobileEyes docs for what layer means)
  int getLayer(void) { return myLayer; }
  /// Gets how often this data should be drawn (0 == only when explicitly sent)
  unsigned int getDefaultRefreshTime(void) { return myDefaultRefreshTime; }
  /// Gets the secondary color (meaning depends on shape)
  ArColor getSecondaryColor(void) { return mySecondaryColor; }
  /// Gets the visibility of the drawing data
  const char *getVisibility(void) { return myVisibility.c_str(); }

  /// Sets the shape of data to draw
  void setShape(const char *shape) { myShape = shape; }
  /// Sets the primary color (meaning depends on shape)
  void setPrimaryColor(ArColor color) { myPrimaryColor = color; }
  /// Sets the size (meaning depends on shape, but its in mm)
  void setSize(int size) { mySize = size; }
  /// Sets the layer (see MobileEyes for docs on what layer means)
  void setLayer(int layer) { myLayer = layer; }
  /// Gets how often this data should be drawn (0 == only when explicitly sent)
  void setDefaultRefreshTime(unsigned int refreshTime)
    { myDefaultRefreshTime = refreshTime; }
  /// Sets the secondary color (meaning depends on shape)
  void setSecondaryColor(ArColor color) { mySecondaryColor = color; }
  /// Sets the visibility of the drawing data
  void setVisibility(const char *visibility) { myVisibility = visibility; }
protected:
  std::string myShape;
   ArColor myPrimaryColor;
  int mySize;
  int myLayer;
  unsigned int myDefaultRefreshTime;
  ArColor mySecondaryColor;
  std::string myVisibility;
};

#endif
