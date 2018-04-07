#ifndef CVD_V4LCONTROL_H
#define CVD_V4LCONTROL_H

#include <string>
#include <map>
#include <vector>

#include <linux/videodev2.h>

#include <cvd/exceptions.h>

namespace CVD {

namespace Exceptions
{
    /// @ingroup gException
    namespace V4LControl
    {
    /// @ingroup gException
    struct All: public CVD::Exceptions::All
    {
    };
    /// Error opening the device
    /// @ingroup gException
    struct DeviceOpen: public All {DeviceOpen(std::string dev); ///< Construct from the device name
    };

    /// Unsupported parameter
    /// @ingroup gException
    struct ParameterNotSupported: public All {
        ParameterNotSupported(std::string);  ///< Construct from parameter name
        ParameterNotSupported(unsigned int); ///< Construct from parameter id
    };

    /// Error querying value
    /// @ingroup gException
    struct GetValue: public All {GetValue(std::string); ///< Construct from parameter name
    };

    /// Error setting value
    /// @ingroup gException
    struct SetValue: public All {SetValue(std::string); ///< Construct from parameter name
    };

    /// Error querying device parameters
    /// @ingroup gException
    struct QueryParameters: public All {QueryParameters(std::string); ///< Construct from message
    };

    }
}


/**
exposes the V4L2 API to set parameters on a capture device. It can be used
in parallel to a v4lbuffer object to control and query all parameters supported.
Several abstraction levels are supported. On the highest there are individual
member functions for various fixed parameters. The next level supports querying and
setting/getting values for all parameters supported by the driver. Finally,
low-level access using V4L2 structs is supported as well.

@ingroup gVideo
*/
class V4LControl {
public:
    V4LControl( int fd, bool report = true );
    V4LControl( const std::string & name, bool report = true );

    /// @name HighLevel
    /// High level interface to set specific parameters without much hassle.
    /// These might fail if the parameter is not supported by the device and do
    /// not throw exceptions. Moreover ranges of parameter values are abstracted
    /// to lie within the intervall [0,1].
    /// If you want to have more control, use the lower level interface described in
    /// @ref Generic .
    //@{
    void exposure( int );
    int  exposure(void);

    void autoexposure(bool);
    bool autoexposure();

    void gain( double );
    double gain(void);

    void autogain(bool);
    bool autogain(void);

    void brightness(double);
    double brightness(void);

    void contrast(double);
    double contrast(void);

    void saturation(double);
    double saturation(void);
    //@}

    /// @name Generic
    /// generic high level interface abstracting the v4l2 structs
    //@{
    unsigned int getId( const std::string & name ) const;
    std::string getName( unsigned int id ) const;
    std::vector<unsigned int> supportedParameters(void) const;
    std::vector<std::string> supportedParameterNames(void) const;

    inline bool isSupported( const std::string & name ) const {
        return (controlNames.find(name) != controlNames.end());
    }
    inline void set( const std::string & name, int value ){
        set(getId(name), value);
    }
    inline int get( const std::string & name ){
        return get(getId(name));
    }
    inline int type( const std::string & name ){
        return type(getId(name));
    }
    inline std::map<unsigned int, std::string> menuValues( const std::string & name ){
        return menuValues(getId(name));
    }
    inline int defaultValue( const std::string & name ){
        return defaultValue(getId(name));
    }
    inline int min( const std::string & name ){
        return min(getId(name));
    }
    inline int max( const std::string & name ){
        return max(getId(name));
    }
    inline int step( const std::string & name ){
        return step(getId(name));
    }

    inline bool isSupported( unsigned int id ) const {
        return (controlData.find(id) != controlData.end());
    }
    void set( unsigned int id, int value);
    int get( unsigned int id );
    int type( unsigned int id );
    std::map<unsigned int, std::string> menuValues( unsigned int id ) const ;
    int defaultValue( unsigned int id ) const;
    int min( unsigned int id ) const;
    int max( unsigned int id ) const;
    int step( unsigned int id ) const;
    //@}

    /// @name LowLevel
    /// low level interface using v4l2 structs
    //@{
    int getQueryStruct( v4l2_queryctrl & query ) const;
    void getMenuStruct( unsigned int id, std::vector<v4l2_querymenu> & menu ) const;
    int setControlStruct( v4l2_control & value );
    int getControlStruct( v4l2_control & value ) const;
    //@}

    /// return file descriptor for the opened device
    inline int getFile(void) const { return device; }

    /// return file name of the opened device
    inline const std::string & getDevice(void) const { return deviceName; }

    /// set error reporting
    inline void setReportErrors( bool report ) { reportErrors = report; }
    inline bool getReportErrors(void) const { return reportErrors; }

protected:
    int device;
    std::string deviceName;
    struct v4l2_control control;
    bool reportErrors;

    void queryControls(void);

    std::map<std::string, unsigned int> controlNames;
    std::map<unsigned int, v4l2_queryctrl> controlData;
    std::map<unsigned int, std::vector<v4l2_querymenu> >  menuData;
};

} // namespace CVD

#endif
