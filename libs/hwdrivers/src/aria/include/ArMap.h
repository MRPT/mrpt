/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef ARMAP_H
#define ARMAP_H

#include "ariaTypedefs.h"
#include "ariaUtil.h"
#include "ArFunctor.h"
#include "ArArgumentBuilder.h"
#include "ArMutex.h"

#include <vector>

class ArFileParser;

class ArMapObject;

/// This is a class for maps made with ScanStudio and Mapper3.
/**
 * ArMap encapsulates the data contained in a .map file.  It provides
 * methods to read and write a .map file, and to obtain and modify
 * the contents of the map. In addition, the application may install
 * callbacks on the map to be notified when its contents are modified.
 *
 * Aria maps contain the following types of data:
 *
 * <ul>
 *  <li> Header information which includes the boundaries of the map, various
 *    data counts, and the resolution (in mm) at which the map was made. </li>
 *  <li> Optional metadata that can be used to define custom object types.
 *    (This is explained in greater detail below.)  </li>
 *  <li> Map objects, such as goals and forbidden lines. </li>
 *  <li> Map data: lines and points (in mm). </li>
 * </ul>
 *
 * See below for the exact format of the actual .map file.
 *
 * @section MapThreading Thread Issuses
 *
 * The ArMap class is not inherently thread-safe.  It has lock() and
 * unlock() methods, which the application must call before
 * and after calls to get and set the data (e.g. getMapObjects(),
 * getPoints()...  setMapObjects(), setPoints()...).
 *
 * If you are going to use setMapObjects(), setPoints(), setLines(), or
 * setMapInfo(), then you should lock() the map beforehand, call the methods,
 * then call mapChanged() to invoke the callbacks, and then finally
 * unlock() the map when done.  Note that mapChanged() will only invoke
 * the callbacks if the data has actually changed.
 *
 * The readFile() and writeFile() methods do automatically lock
 * the map while they read and write.
 *
 * @section MapObjects Map Objects
 *
 * Certain types of objects are predefined for ArMaps.  These include
 * Goal, GoalWithHeading, Dock, ForbiddenLine, ForbiddenArea, and RobotHome.
 *
 *  - Goal and GoalWithHeading are basically named ArPoses, the difference being
 *    that the "th" (heading) value is only valid for GoalWithHeading.
 *  - Dock is an ArPose.
 *  - ForbiddenLine is a boundary line, and ForbiddenArea is a rectangular
 *    "sector". The extents of these objects are given as a pair of
 *    poses, a "from" point and a "to" point.
 *  - RobotHome may be either an ArPose or a rectangular sector.
 *
 *
 * A rectangle object may have an associated angle of rotation as well,
 * stored in the object pose theta value (ArMapObject::getPose().getTh()).
 * The actual global coordinates of the rectangle must be calculated
 * using this angle and its "from-to" values. You can get a list of the 4
 * ArLineSegment objects taht compose the rectangle's edges using
 * ArMapObject::getFromToSegments(). If you want to do your own calculations,
 * see ArMapObject::ArMapObject().
 *
 *
 * @section MapCustomObjects Defining Custom Map Objects
 *
 * It is possible to define additional types of objects in a map file using
 * the "MapInfo" metadata section.  For example, if you wished to program
 * some special behavior that would only occur upon reaching certain goals,
 * you could define a new goal type as follows:
 * <pre>
 *    MapInfo: GoalType Name=SpecialGoal "Label=Special" "Desc=Doing special stuff" Heading=Required Shape=VBars "Color0=0xff0000"
 * </pre>
 * The new goal type will appear be available in Mapper3 and MobilePlanner in a drop-down menu.
 * Instances in the map will also be displayed by MobileEyes.
 *
 *
 * Please read the following information carefully if you plan to use this
 * feature.
 *
 * Each MapInfo line is of the format:
 * <pre>
 *       MapInfo: \<Keyword\> ([ParamName=ParmValue] )*
 * </pre>
 *
 * (<code>([ParamName=ParamValue] )*</code> is a space separated list of
 * "<code>key=value</code> codes.)
 *
 * </p><p>
 * The following Keywords are supported:
 *  - GoalType : defines a goal subtypes
 *  - DockType : defines a dock subtypes
 *  - LocationType : defines another kind of point in the map
 *  - BoundaryType : defines a line object in the map
 *  - SectorType : defines a rectangular area of some kind in the map (which may be rotated)
 * .
 *
 * The available parameters depend on the Keyword.  Unless otherwise specified,
 * parameters are optional.
 *
 * If a parameter value contains a space, then both the parameter name and
 * value must be enclosed within quotes.  For example:
 * <pre>
 *      "Label=Normal Goal"
 * </pre>
 * Neither the name nor the value can contain the special characters #, ;,
 * %, or ".
 *
 * The following ParamNames are valid for all keywords:
 *  - Name=\<String\> : Text name of the type that is being defined.  (Required.)
 *  - Label=\<String\> : Label that is displayed for the type in popup menus, etc.
 *  - Desc=\<String\> : Description of the type that is displayed in tool tips, etc.
 *  - Vis=[AlwaysOn|DefaultOn|DefaultOff|AlwaysOff] : Specifies the
 *    visibility of the associated item type. The default is DefaultOn.
 *    (This parameter is currently primarily supported for IsData=1 items only.
 *    See BoundaryType.  In particular, if DefaultOff is specified for a
 *    non-data-item, there currently is no way in MobilePlanner or MobileEyes to
 *    subsequently show it.)
 * .
 *
 * For GoalType, DockType, and LocationType, the following ParamNames are
 * also supported:
 *  - Shape=[Plain|Cross|HBars|Triangle|T|U|VBars] : Shape of the icon used
 *    to draw the pose. (The default is Plain.)
 *      - Plain: The default shape, typically a filled square
 *      - Cross: A cross shape
 *      - HBars: A square shape containing horizontal bars or stripes
 *      - Triangle: A Triangle
 *      - T: A "T" shape
 *      - U: A "U" shape
 *      - VBars: A square shape containing vertical bars or stripes
 *  - Size=\<Integer\> : Width/height of the displayed icon in mm.
 *  - Color\<n\>=\<Hexadecimal Color Value\> (n=0,1,2) : Colors with which to draw the icon.
 *    (In general, Color0 is the primary icon color, Color1 is the heading
 *    color, and Color2 is the background/outline color.) The value
 *    is a hexadecimal number starting with <code>0x</code>, and followed
 *    by two digits for the red component, two digits for green, and two
 *    digits for blue. For example, <code>0xff00ff</code>.
 *  .
 *
 * In addition, the following ParamName is supported only for GoalTypes:
 *  - Heading=[Required|Optional|Never] : Whether a heading is required
 *    to be given with this goal, is optional, or is irrelevant.
 *  .
 *
 * For BoundaryType, the following ParamNames are also supported:
 *  - NameRequired=[0|1] : Whether the item must be named
 *  - Color0=\<RGB String\> (n=0,1,2) : Color with which to draw the line.
 *  - IsData=[0|1] : Set to 1 to indicate that the item is inherently
 *    part of the map data.  The default is 0 to indicate user-created
 *    items.
 * .
 *
 * For SectorType, the following ParamNames are also supported:
 *  - NameRequired=[0|1] : Whether the item must be named
 *  - Shape=[Plain|Arrow] : Shape of the icon used to draw the rectangle.
 *    (The default is Plain.)
 *  - Color\<n\>=\<RGB String\> (n=0,1) : Colors with which to draw the
 *     rectangle. (In general, Color0 is the primary rectangle color,
 *     Color1 is the shape/icon color.)
 *  .
 *
 * <i>Important Note</i>: if a map defines special GoalType or DockType items,
 * then it must define <b>all</b> possible goal or dock types, including the
 * default "Goal", "GoalWithHeading", and "Dock" types if you want those
 * types to remain available.
 *
 *
 * @section MapFileFormatSpec Map File Format Specification

The robot map file is in ASCII text, and may be viewed or edited in any
text editor.
Map file names conventionally end in the suffix ".map".
A formal description of the map syntax
follows in <a href="http://www.rfc-editor.org/rfc/rfc4234.txt">augmented
Backus-Naur Format (ABNF)</a>.

All blank lines should be ignored. As an exception to ABNF, literal
strings <em>are</em> case-sensitive.

<p>
A map is the line "2D-Map" followed by the metadata section, followed by
some number of data sections:
</p>
<pre>
ARMAP           = ("2D-Map" NEWLINE) (MetadataSection) (*DataSection)
</pre>

<p>
The MetadataSection section provides information about the map data, adds objects (Cairns)
and also provides storage of application-specific/custom information.
</p>
<pre>
MetadataSection      = *MetadataLine
MetadataLine         = MDKey ":" *(WS MDVal) NEWLINE
MDKey                = StringID
MDVal                = Integer / Float / StringID / KeyValPair
</pre>

<p>
Most metadata lines fall into one of two categories: a simple list of numeric tokens,
or a StringID followed by a list of either numeric tokens or a set of KeyValuePair
tokens.
</p>


<p>
Sensor data sections contain data that was recorded with sensors (e.g. a Laser Rangefinder
for the "DATA" section) and which represent more or less permanent, solid objects
detectable by a robot's range sensors.  (This data can be used for localization
and path planning, for instance.)  The DATA section is a collection of points
detected by a high-resolution sensor like the LRF. LINES abstracts the world into
a set of flat surfaces.
</p>
<pre>
DataSection          = ("DATA" NEWLINE *PointLine) / ("LINES" NEWLINE *LineLine)
PointLine            = XPos WS YPos NEWLINE
LineLine             = XPos WS YPos WS XPos WS YPos NEWLINE
</pre>

<p>
"Cairn" is a common instance of MDKey. A "Cairn" metadata entry looks like this:
</p>
<pre>
MetaDataLine         =/ Cairn  NEWLINE
Cairn                = "Cairn:" WS CairnType WS XPos WS YPos WS Theta WS InternalName WS IconName WS Label [WS TypeSpecificData]
CairnType            = StringID
XPos                 = Integer
YPos                 = Integer
Theta                = Integer
InternalName         = QuotedString
IconName             = QuotedString
Label                = QuotedString
TypeSpecificData     = *(WS MDKey)
</pre>

<p>
"MapInfo" is another common instance of MDKey. A "MapInfo" entry can describe custom
map object types for your application beyond the usual Cairn types (see above).
</p>
<pre>
MetaDataLine         =/ MapInfo NEWLINE
MapInfo              = "MapInfo:" WS MapInfoClass WS *(KeyValuePair)
MapInfoClass         = StringID
</pre>

<p>Data types:</p>
<pre>
KeyValPair           = (StringID "=" MDVal) /  QUOTE ALNUM "=" Text QUOTE
Integer              = ["-"] *1(DIGIT)
Float                = ["-"] *1(DIGIT | ".")
StringID             = *1 ALNUM     ; One or more alphanumeric characters
QuotedText           = QUOTE Text QUOTE
Text                 = *(ALNUM / WS / PUNCTUATION)
DIGIT                = ("0"-"9")
ALPHA                = ("a"-"z" / "A"-"Z")
ALNUM                = ALPHA / DIGIT
WS                   = *(" ")      ; Any number of ASCII space characters (incl. 0)
QUOTE                = %d34        ; ASCII double quote mark (")
NEWLINE              = %d10        ; ASCII newline (\n)
PUNCTUATION          = %d32-%d47 / %d58-%d64 / %d92-%d96 / %d123-%d126
ANY                  = %d32-%d126  ; Any ASCII text
</pre>



<p>
Notes:
</p>

<ol>

<li>Common IDs for <i>MDKey</i> are:
  <dl>
    <dt><code>MinPos</code></dt>
    <dt><code>MaxPos</code></dt>
    <dt><code>NumPoints</code></dt>
      <dd>Number of entries in the DATA section.</dd>
    <dt><code>LineMinPos</code></dt>
    <dt><code>LineMaxPos</code></dt>
    <dt><code>NumLines</code></dt>
      <dd>Number of entries in the LINES section.</dd>
    <dt><code>Resolution</code></dt>
    <dt><code>Cairn</code></dt>
      <dd>Defines a special object in the map with semantic meaning.</dd>
    <dt><code>MapInfo</code></dt>
      <dd>Describes custom, cairn types</dd>
  </dl>
  New values may be added in the future, or used only by some applications.
</li>

<li>Common <i>CairnType</i> values are:
  <dl>
    <dt><code>Goal</code></dt>
      <dd>A named goal. <i>Theta</i> should be ignored.  The name of the goal is provided in <i>Label</i>.</dd>
    <dt><code>GoalWithHeading</code></dt>
      <dd>A named goal. <i>Theta</i> indicates a final heading. The name of the goal is provided in <i>Label</i>.</dd>
    <dt><code>RobotHome</code></dt>
      <dd>A possible starting position of a robot.</dd>
    <dt><code>Dock</code></dt>
      <dd>A position and heading at which a docking maneuver may be initiated</dd>
    <dt><code>ForbiddenLine</code></dt>
      <dd>Specifies a line that any automatic navigation procedure should avoid crossing.
      This Cairn type has the following <i>TypeSpecificData</i>, which defines the endpoints
      of the line:
<pre>
TypeSpecificData     =/ ForbiddenLineData
ForbiddenLineData    =  XPos WS YPos WS XPos WS YPos
</pre>
      The normal Cairn pose is not used for <code>ForbiddenLine</code>.
      </dd>
    <dt><code>ForbiddenArea</code></dt>
      <dd>Specifies a rectangular area that any automatic navigation procedure should avoid entering.
      This Cairn type has the following <i>TypeSpecificData</i>, which defines the upper-left and
      lower-right opposing corners of the rectangle:
<pre>
TypeSpecificData     =/ ForbiddenAreaData
ForbiddenAreaData    =  XPos WS YPos WS XPos WS YPos
</pre>
      The normal Cairn pose for <code>ForbiddenArea</code> defines an offset of
      the geometric center of the area, plus a rotation of the  area.
      (Typically, <i>XPos</i> and <i>YPos</i> will always be <code>0</code> for <code>ForbiddenArea</code>, but <i>Theta</i> may be
       used to provide the rotation of the rectangular area).
      </dd>
   </dl>
</li>
<li>The <i>InternalName</i> and <i>IconName</i> tokens in <i>CairnData</i> are not currently used. Typically, <i>InternalName</i> is simply an empty quoted string ("") and <i>IconName</i>
is the placeholder value <code>"ICON"</code>.  You should preserve them when reading and writing
map files though, as they may be used in the future.
</li>
<li>It is be better to calculate maximum, minimum, and number of
points or lines based on the data in the map, if possible, rather than
relying on the metadata header.
</li>
<li>What the heck is a "cairn"?   The word is from the Scottish Gaelic, Old Irish
and Welsh "carn" and Middle English "carne".  A cairn is a pile of stones,
placed in the landscape as a memorial, navigation aid, or other marker. So we
use it to indicate a semantically meaningful object placed at some point by the human mapmaker
(rather than something detectable by the robot).
</li>
<li>
  Currently used <i>MapInfoClass</i> keywords include:
  <dl>
    <dt><code>GoalType</code></dt> <dd>define a custom goal subtype</dd>
    <dt><code>DockType</code></dt> <dd>define a custom dock subtype</dd>
    <dt><code>LocationType</code></dt> <dd>define a custom other generic poses on the map</dd>
    <dt><code>BoundaryType</code></dt> <dd>define a custom line on the map</dd>
    <dt><code>SectorType</code></dt> <dd>defines a custom rectangular area (which may be rotated)</dd>
  </dl>
  The following ParamNames are valid for all <i>MapInfoClass</i> keywords:
  <dl>
    <dt><code>Name=</code><i>Text</i></dt> <dd>Name of the type that is being defined.
    <dt><code>Label=</code><i>Text</i></dt> <dd>Label that is displayed for the type in a GUI, etc.</dd>
    <dt><code>Desc=</code><i>Text</i></dt> <dd>Longer description of the type that is displayed in a GUI, etc.</dd>
  </dl>
For more information about the use of <code>MapInfo</code> metadata, see the discussion above.
</li>
</ol>

**/

class ArMap
{
  public:

  /// List of the standard Info categories defined for the map
  enum InfoType {
      // If any InfoType is added, then the ourInfoNames array must also be updated...
	  //MAP_INFO,
    META_INFO,
	  TASK_INFO,
	  ROUTE_INFO,
    SCHED_TASK_INFO,
    SCHED_INFO,
	  LAST_INFO = SCHED_INFO ///< Last value in the enumeration
  };

  enum {
	  INFO_COUNT = LAST_INFO + 1 ///< Number of standard Info categories
  };


  /// Constructor
  AREXPORT ArMap(const char *baseDirectory = "./",
		 bool addToGlobalConfig = true,
		 const char *configSection = "Files",
		 const char *configParam = "Map",
		 const char *configDesc =
		 "Map of the environment we'll use to navigate",
		 bool ignoreEmptyFileName = true);
  /// Destructor
  AREXPORT virtual ~ArMap(void);

  /// Reads a map
  AREXPORT bool readFile(const char *fileName,
			 char *errorBuffer = NULL, size_t errorBufferLen = 0);

  /// Write out a map file
  AREXPORT bool writeFile(const char *fileName, bool internalCall = false);

  /// Adds a callback called when the map is changed
  AREXPORT void addMapChangedCB(ArFunctor *functor,
				ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback called when the map is changed
  AREXPORT void remMapChangedCB(ArFunctor *functor);

  /// Gets the base directory
  AREXPORT const char *getBaseDirectory(void) const;
  /// Gets the fileName that was loaded
  AREXPORT const char *getFileName(void) const;
  /// Sets whether we ignore empty file names or fail if we encounter one
  void setIgnoreEmptyFileName(bool ignore) { myIgnoreEmptyFileName = ignore; }
  /// Gets whether we ignore empty file names or fail if we encounter one
  bool getIgnoreEmptyFileName(void) { return myIgnoreEmptyFileName; }
  /// Sets the base directory
  AREXPORT void setBaseDirectory(const char *baseDirectory);
  /// Gets the map objects
  std::list<ArMapObject *> *getMapObjects(void) { return &myMapObjects; }

  /// Gets the first map object of given name and type if it exists
  AREXPORT ArMapObject *findFirstMapObject(const char *name, const char *type);


  /// Gets the map info strings
  std::list<ArArgumentBuilder *> *getMapInfo(void) { return &myMapInfo; }

  /// Gets the strings for the specified Info category.
  /**
   * @param infoType the int ID of the Info category; must be >= 0 and less than
   * numInfos
   * @return std::list<ArArgumentBuilder *> * a pointer to the Info list; NULL
   * if infoType was invalid
  **/
  AREXPORT std::list<ArArgumentBuilder *> *getInfo(int infoType);

  /// Gets the map points
  std::vector<ArPose> *getPoints(void) { return &myPoints; }
  /// Gets the lines
  std::vector<ArLineSegment> *getLines(void) { return &myLines; }
  /// Gets the lower left point of the points
  ArPose getMinPose(void) { return myMin; }
  /// Gets the upper right point of the points
  ArPose getMaxPose(void) { return myMax; }
  /// Gets the number of points
  int getNumPoints(void) { return myNumPoints; }
  /// Gets the lower left point of the lines
  ArPose getLineMinPose(void) { return myLineMin; }
  /// Gets the upper right point of the lines
  ArPose getLineMaxPose(void) { return myLineMax; }
  /// Gets the number of lines
  int getNumLines(void) { return myNumLines; }
  /// Gets the resolution (-1 if none specified)
  int getResolution(void) { return myResolution; }
  /// Gets a map object of given name and type if it exists
  AREXPORT ArMapObject *findMapObject(const char *name,
				      const char *type = NULL);
  /// Gets the last time the map objects were changed
  ArTime getMapObjectsChanged(void) { return myMapObjectsChanged; }
  /// Gets the last time the points were changed
  ArTime getPointsChanged(void) { return myPointsChanged; }
  /// Gets the last time the lines were changed
  ArTime getLinesChanged(void) { return myLinesChanged; }

  /// Gets the last time the map info was changed
  ArTime getMapInfoChanged(void) { return myMapInfoChanged; }

  /// Returns the last time that the specified Info category was changed.
  /**
   * @param infoType the int ID of the Info category; must be >= 0 and < numInfos
   * @return ArTime the time that the Info category was last changed; returns
   * default time (1/1/1970) if infoType is invalid
  **/
  AREXPORT ArTime getInfoChanged(int infoType);

  /// Sets the map objects (copies those passed in)
  AREXPORT void setMapObjects(const std::list<ArMapObject *> *mapObjects);
  /// Sets the points (copies those passed in)
  AREXPORT void setPoints(const std::vector<ArPose> *points);
  /// Sets the lines (copies those passed in)
  AREXPORT void setLines(const std::vector<ArLineSegment> *lines);

  /// Sets the map info (copies those passed in)
  AREXPORT void setMapInfo(const std::list<ArArgumentBuilder *> *mapInfo);

  /// Sets the contents of the specified Info category.
  /**
   * @param infoType the int ID of the Info category to be set; must be >= 0
   * and < numInfos;
   * @param infoList the std::list<ArArgumentBuilder *> * that defines the
   * Info category's contents; NULL to clear the Info
   * @return bool set to true if the contents were successfully set; false,
   * if infoType was invalid
  **/
  AREXPORT bool setInfo(int infoType,
						const std::list<ArArgumentBuilder *> *infoList);

  AREXPORT void setResolution(int resolution);

  /***
  /// Sets the route info (copies those passed in)
  AREXPORT void setRouteInfo(const std::list<ArArgumentBuilder *> *routeInfo);
  ****/

  /// Function that will call the map changed CBs if needed
  AREXPORT void mapChanged(void);

  /// Writes all of the map to a functor instead of a to a file
  AREXPORT void writeToFunctor(ArFunctor1<const char *> *functor,
			       const char *endOfLineChars);


  /// Parses a map line
  AREXPORT bool parseLine(char *line);
  /// Says that the parsing by lines is done and to use the parsed data
  AREXPORT void parsingComplete(void);

  /// Lock the map instance
  /*AREXPORT*/ int lock() {return(myMutex.lock());}
  /// Try to lock the map instance without blocking
  /*AREXPORT*/ int tryLock() {return(myMutex.tryLock());}
  /// Unlock the map instance
  /*AREXPORT*/ int unlock() {return(myMutex.unlock());}



  // When loading a map, returns whether all header, objects, and lines have completed loading.
  /**
   * This value returns true once the DATA tag has been reached.
   * The rest of the map contains data points.
  **/
  /*AREXPORT*/ bool isLoadingDataStarted() {return myLoadingDataStarted;}

  // When loading a map, returns whether all header and objects have completed loading.
  /**
   * This value returns true once the LINES tag has been reached.
   * The rest of the map contains data points.
  **/
  /*AREXPORT*/bool isLoadingLinesAndDataStarted()
               {return myLoadingLinesAndDataStarted;}

  /// Writes the map header information and objects to a text-based functor.
  /**
   * This method writes everything up to and including the DATA tag
   * to the given functor.  The data points are not written.
  **/
  AREXPORT void writeObjectsToFunctor
					(ArFunctor1<const char *> *functor,
			         const char *endOfLineChars);

  /// Writes the map data points to a functor.
  /**
   * A pointer to the entire data point vector is passed directly to the
   * functor in order to improve performance.  The functor should not
   * modify the vector's contents.
  **/
  AREXPORT void writePointsToFunctor
			(ArFunctor2<int, std::vector<ArPose> *> *functor);
  /// Writes the map line segments to a functor.
  /**
   * A pointer to the entire line segment vector is passed directly to the
   * modify the vector's contents.
  **/
  AREXPORT void writeLinesToFunctor
		(ArFunctor2<int, std::vector<ArLineSegment> *> *functor);

  /// Reads a data point from the given line, and adds it to the loading points.
  AREXPORT bool readDataPoint( char *line);
  /// Reads a line segment from the given line, and adds it to the loading lines.
  AREXPORT bool readLineSegment( char *line);
  /// Adds the specified data point to the loading points.
  AREXPORT void loadDataPoint(double x, double y);
  /// Adds the specified line segment to the loading lines.
  AREXPORT void loadLineSegment(double x1, double y1, double x2, double y2);
  /// Sets the level we log our map changed callback at
  /*AREXPORT*/ void setMapChangedLogLevel(ArLog::LogLevel level)
    {  myMapChangedLogLevel = level; }
  /// Gets the level we log our map changed callback at
  /*AREXPORT*/ArLog::LogLevel getMapChangedLogLevel(void)
    {  return myMapChangedLogLevel; }
  /// Adds a callback called before the map changed callbacks are called
  AREXPORT void addPreMapChangedCB(ArFunctor *functor,
                                  ArListPos::Pos position = ArListPos::LAST);
  /// Removes a callback called before the map changed callbacks are called
  AREXPORT void remPreMapChangedCB(ArFunctor *functor);

protected:


  // Function for processing the config file
  bool processFile(char *errorBuffer, size_t errorBufferLen);
  // Function to read the 2D-Map
  bool handle2DMap(ArArgumentBuilder *arg);
  // Function to read the minimum pos
  bool handleMinPos(ArArgumentBuilder *arg);
  // Function to read the maximum pos
  bool handleMaxPos(ArArgumentBuilder *arg);
  // Function to read the number of points
  bool handleNumPoints(ArArgumentBuilder *arg);
  // Function to read the line minimum pos
  bool handleLineMinPos(ArArgumentBuilder *arg);
  // Function to read the line maximum pos
  bool handleLineMaxPos(ArArgumentBuilder *arg);
  // Function to read the number of lines
  bool handleNumLines(ArArgumentBuilder *arg);
  // Function to handle the resolution
  bool handleResolution(ArArgumentBuilder *arg);
  // Function to handle the cairns
  bool handleMapObject(ArArgumentBuilder *arg);

  bool handleInfo(ArArgumentBuilder *arg, int info);

  // Function to handle the information about the map
  bool handleMapInfo(ArArgumentBuilder *arg);

  // Function to catch the LINES line signifying data
  bool handleLines(ArArgumentBuilder *arg);
  // Function to catch the DATA line signifying data
  bool handleData(ArArgumentBuilder *arg);
  // Function to snag the map points (mainly for the getMap over the network)
  bool handlePoint(ArArgumentBuilder *arg);
  // Function to snag the line segments (mainly for the getMap over the network)
  bool handleLine(ArArgumentBuilder *arg);

  /// Returns the name of the specified Info category (i.e. the "xInfo:" tag at the beginning of the line)
  /**
   * If subclasses define additional Info categories, then they must override this
   * method.
   * @param infoType the int ID of the Info category
   * @return const char * the name of the specified Info category; or NULL if not
   * found
  **/
  virtual const char *getInfoName(int infoType);

  /// Array of names for the standard Info categories; see InfoType
  static const char *ourInfoNames[INFO_COUNT];

  enum {
    MAX_MAP_NAME_LENGTH = 512
  };

  // lock for our data
  ArMutex myMutex;
  // resets the parser functions and variables (true if good, false otherwise)
  AREXPORT bool reset(void);
  std::string myBaseDirectory;
  std::string myFileName;
  struct stat myReadFileStat;
  ArFileParser *myLoadingParser;

  /// Number of different Info categories available in this map.
  int myNumInfos;

  // vars for if we got some important info, the other important info
  // is taken care of by the adding of callbacks
  bool myLoadingGot2DMap;
  bool myLoadingGotMaxPos;
  bool myLoadingGotMinPos;
  bool myLoadingGotLineMaxPos;
  bool myLoadingGotLineMinPos;
  ArPose myLoadingMaxFromFile;
  ArPose myLoadingMinFromFile;
  int myLoadingPointsRead;
  ArPose myLoadingLineMaxFromFile;
  ArPose myLoadingLineMinFromFile;
  int myLoadingLinesRead;

  std::string myConfigParam;
  bool myIgnoreEmptyFileName;
  // data from the file
  std::list<ArMapObject *> myLoadingMapObjects;
  std::vector<ArPose> myLoadingPoints;
  std::vector<ArLineSegment> myLoadingLines;

  std::list<ArArgumentBuilder *> *myLoadingInfoArray;
  std::list<ArArgumentBuilder *> myLoadingMapInfo;
  int myLoadingNumPoints;
  int myLoadingNumLines;
  int myLoadingResolution;
  ArPose myLoadingMax;
  ArPose myLoadingMin;
  ArPose myLoadingLineMax;
  ArPose myLoadingLineMin;

  // our good data in memory (could be the same thing)
  std::list<ArMapObject *> myMapObjects;
  ArTime myMapObjectsChanged;
  std::vector<ArPose> myPoints;
  ArTime myPointsChanged;

  std::vector<ArLineSegment> myLines;
  ArTime myLinesChanged;

  std::list<ArArgumentBuilder *> *myInfoArray;
  ArTime *myInfoChangedArray;

  std::list<ArArgumentBuilder *> myMapInfo;
  ArTime myMapInfoChanged;


  int myNumPoints;
  int myNumLines;
  int myResolution;
  ArPose myMax;
  ArPose myMin;
  ArPose myLineMax;
  ArPose myLineMin;


  std::list<ArFunctor *> myMapChangedCBList;
  std::list<ArFunctor *> myPreMapChangedCBList;
  ArTime myMapChangedMapObjects;
  ArTime myMapChangedPoints;

  ArTime *myMapChangedInfoArray;
  ArTime myMapChangedMapInfo;


  // things for our config
  bool myConfigProcessedBefore;
  char myConfigMapName[MAX_MAP_NAME_LENGTH];

  bool myLoadingDataStarted;
  bool myLoadingLinesAndDataStarted;
  ArLog::LogLevel myMapChangedLogLevel;
  // callbacks
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> my2DMapCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myMinPosCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myMaxPosCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myNumPointsCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myLineMinPosCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myLineMaxPosCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myNumLinesCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myResolutionCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myMapObjectCB;


  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myMapInfoCB;
  ArRetFunctor2C<bool, ArMap, ArArgumentBuilder *, int> **myInfoCBArray;

  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myDataCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myLinesCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myPointCB;
  ArRetFunctor1C<bool, ArMap, ArArgumentBuilder *> myLineCB;
  ArRetFunctor2C<bool, ArMap, char *, size_t> myProcessFileCB;
};

/// This is a class for objects within an ArMap
class ArMapObject
{
public:
  /// Constructor
  /*AREXPORT*/ ArMapObject(const char *type, ArPose pose, const char *fileName,
		       const char *iconName, const char *name,
		       bool hasFromTo, ArPose fromPose, ArPose toPose)
    {
      if (type != NULL) myType = type;
      if (name != NULL) myName = name;
      myPose = pose;
      if (iconName != NULL) myIconName = iconName;
      if (fileName != NULL) myFileName = fileName;
      myHasFromTo = hasFromTo; myFromPose = fromPose;  myToPose = toPose;
      if (myHasFromTo)
      {
       double angle = myPose.getTh();
       double sa = ArMath::sin(angle);
       double ca = ArMath::cos(angle);
       double fx = fromPose.getX();
       double fy = fromPose.getY();
       double tx = toPose.getX();
       double ty = toPose.getY();
       ArPose P0((fx*ca - fy*sa), (fx*sa + fy*ca));
       ArPose P1((tx*ca - fy*sa), (tx*sa + fy*ca));
       ArPose P2((tx*ca - ty*sa), (tx*sa + ty*ca));
       ArPose P3((fx*ca - ty*sa), (fx*sa + ty*ca));
       myFromToSegments.push_back(ArLineSegment(P0, P1));
       myFromToSegments.push_back(ArLineSegment(P1, P2));
       myFromToSegments.push_back(ArLineSegment(P2, P3));
       myFromToSegments.push_back(ArLineSegment(P3, P0));
      }
    }
  /// Copy constructor
  /*AREXPORT*/ArMapObject(const ArMapObject &mapObject)
    {
      myType = mapObject.myType; myName = mapObject.myName;
      myPose = mapObject.myPose; myIconName = mapObject.myIconName;
      myFileName = mapObject.myFileName;  myHasFromTo = mapObject.myHasFromTo;
      myFromPose = mapObject.myFromPose; myToPose = mapObject.myToPose;
      myFromToSegments = mapObject.myFromToSegments;
    }

  /// Destructor
  /*AREXPORT*/ virtual ~ArMapObject() {}
  /// Gets the type of the object
  const char *getType(void) const { return myType.c_str(); }
  /// Gets the pose of the object
  ArPose getPose(void) const { return myPose; }
  /// Gets the fileName of the object (probably never used for maps)
  const char *getFileName(void) const { return myFileName.c_str(); }
  /// Gets the icon string of the object
  const char *getIconName(void) const { return myIconName.c_str(); }
  /// Gets the name of the object (if any)
  const char *getName(void) const { return myName.c_str(); }
  /// Gets the addition args of the object
  bool hasFromTo(void) const { return myHasFromTo; }
  /// Gets the from pose (could be for line or box, depending)
  ArPose getFromPose(void) const { return myFromPose; }
  /// Gets the to pose (could be for line or box, depending)
  ArPose getToPose(void) const { return myToPose; }
  void log(void) {
    if (myHasFromTo)
      ArLog::log(ArLog::Terse,
		 "Cairn: %s %g %g %g \"%s\" %s \"%s\" %d %d %d %d",
		 getType(), myPose.getX(), myPose.getY(), myPose.getTh(),
		 getFileName(), getIconName(), getName(), myFromPose.getX(),
		 myFromPose.getY(), myToPose.getX(), myToPose.getY());
    else
      ArLog::log(ArLog::Terse, "Cairn: %s %g %g %g \"%s\" %s \"%s\"",
		 getType(), myPose.getX(), myPose.getY(), myPose.getTh(),
		 getFileName(), getIconName(), getName());
  }
  /// Gets a list of fromTo line segments that have been rotated
  /**
     Note that this function doesn't know if it makes sense for this
     map object to have the line segments or not (it makes sense on a
     ForbiddenArea but not a ForbiddenLine)...  This is just here so
     that everyone doesn't have to do the same calculation.  Note that
     this might be a little more CPU/Memory intensive transfering
     these around, so you may want to keep a copy of them if you're
     using them a lot (but make sure you clear the copy if the map
     changes).  It may not make much difference on a modern processor
     though (its set up this way for safety).
  **/
  std::list<ArLineSegment> getFromToSegments(void)
    {
      return myFromToSegments;
    }

  /*
  Don't use these, silently changing these values may screw up
  things that assume they don't change.
  void setPose(const ArPose& newpose) { myPose = newpose; }
  void setFromPose(const ArPose& newpose) { myFromPose = newpose; }
  void setToPose(const ArPose& newpose) { myToPose = newpose; }
  */
protected:
  std::string myType;
  std::string myName;
  ArPose myPose;
  std::string myFileName;
  std::string myIconName;
  bool myHasFromTo;
  ArPose myFromPose;
  ArPose myToPose;
  std::list<ArLineSegment> myFromToSegments;
};


#endif // ARMAP_H
