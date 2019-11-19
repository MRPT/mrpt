#include <cvd/Linux/v4lcontrol.h>

#include <iostream>
#include <sstream>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

using namespace std;

template <typename T>
static inline double toUnit( T val, T min, T max){
    double interval = max - min;
    return (val - min)/interval;
}

template <typename T>
static inline T fromUnit( double val, T min, T max){
    double interval = max - min;
    return static_cast<T>(val * interval - min);
}

namespace CVD {

Exceptions::V4LControl::DeviceOpen::DeviceOpen(string device)
{
    what = "V4LControl: failed to open \""+device+ "\": " + strerror(errno);
}

Exceptions::V4LControl::ParameterNotSupported::ParameterNotSupported(string parameter)
{
    what = "V4LControl: parameter \"" + parameter + "\" is not supported.";
}

Exceptions::V4LControl::ParameterNotSupported::ParameterNotSupported(unsigned int id)
{
    ostringstream os;
    os << "V4LControl: parameter " << id << " is not supported.";
    what = os.str();
}

Exceptions::V4LControl::GetValue::GetValue(string parameter)
{
    what = "V4LControl: query value \""+parameter+ "\" failed: " + strerror(errno);
}

Exceptions::V4LControl::SetValue::SetValue(string parameter)
{
    what = "V4LControl: setting value \""+parameter+ "\" failed: " + strerror(errno);
}

Exceptions::V4LControl::QueryParameters::QueryParameters(string msg)
{
    what = "V4LControl: Querying parameters failed: \""+msg+ "\": " + strerror(errno);
}

V4LControl::V4LControl( int fd, bool report ) : device(fd), deviceName(""), reportErrors(report) {
    memset(&control, 0, sizeof(control));
    queryControls();
}

V4LControl::V4LControl( const std::string & name, bool report  ) : device(0), deviceName(name), reportErrors(report) {
    if( -1 == (device = open(deviceName.c_str(), O_RDWR | O_NONBLOCK))){
        throw Exceptions::V4LControl::DeviceOpen(deviceName);
    }
    memset(&control, 0, sizeof(control));
    queryControls();
}

void V4LControl::exposure( int value ){
    if(!isSupported(V4L2_CID_EXPOSURE))
        return;
    set(V4L2_CID_EXPOSURE, value);
}

int V4LControl::exposure(void){
    if(!isSupported(V4L2_CID_EXPOSURE))
        return 0;
    return get(V4L2_CID_EXPOSURE);
}

void V4LControl::autoexposure(bool value){
    if(!isSupported("Auto Exposure"))
        return;
    set("Auto Exposure", value);
}

bool V4LControl::autoexposure(){
    if(!isSupported("Auto Exposure"))
        return false;
    return get("Auto Exposure");
}

void V4LControl::gain( double value ){
    if(!isSupported(V4L2_CID_GAIN))
        return;
    set(V4L2_CID_GAIN, fromUnit(value, min(V4L2_CID_GAIN), max(V4L2_CID_GAIN)));
}

double V4LControl::gain(void){
    if(!isSupported(V4L2_CID_GAIN))
        return 0;
    return toUnit(get(V4L2_CID_GAIN), min(V4L2_CID_GAIN), max(V4L2_CID_GAIN));
}

void V4LControl::autogain(bool value){
    if(!isSupported(V4L2_CID_AUTOGAIN))
        return;
    set(V4L2_CID_AUTOGAIN, value);
}

bool V4LControl::autogain(void){
    if(!isSupported(V4L2_CID_AUTOGAIN))
        return false;
    return get(V4L2_CID_AUTOGAIN);
}

void V4LControl::brightness(double value){
    if(!isSupported(V4L2_CID_BRIGHTNESS))
        return;
    set(V4L2_CID_BRIGHTNESS, fromUnit(value, min(V4L2_CID_BRIGHTNESS), max(V4L2_CID_BRIGHTNESS)));
}

double V4LControl::brightness(void){
    if(!isSupported(V4L2_CID_BRIGHTNESS))
        return 0;
    return toUnit(get(V4L2_CID_BRIGHTNESS), min(V4L2_CID_BRIGHTNESS), max(V4L2_CID_BRIGHTNESS));
}

void V4LControl::contrast(double value){
    if(!isSupported(V4L2_CID_CONTRAST))
        return;
    set(V4L2_CID_CONTRAST, fromUnit(value, min(V4L2_CID_CONTRAST), max(V4L2_CID_CONTRAST)));
}

double V4LControl::contrast(void){
    if(!isSupported(V4L2_CID_CONTRAST))
        return 0;
    return toUnit(get(V4L2_CID_CONTRAST), min(V4L2_CID_CONTRAST), max(V4L2_CID_CONTRAST));
}

void V4LControl::saturation(double value){
    if(!isSupported(V4L2_CID_SATURATION))
        return;
    set(V4L2_CID_SATURATION, fromUnit(value, min(V4L2_CID_SATURATION), max(V4L2_CID_SATURATION)));
}

double V4LControl::saturation(void){
    if(!isSupported(V4L2_CID_SATURATION))
        return 0;
    return toUnit(get(V4L2_CID_SATURATION), min(V4L2_CID_SATURATION), max(V4L2_CID_SATURATION));
}

void V4LControl::queryControls(void){
    controlData.clear();
    controlNames.clear();
    menuData.clear();

    struct v4l2_queryctrl queryctrl;
    memset(&queryctrl, 0, sizeof(queryctrl));
    for (unsigned int i = V4L2_CID_BASE;  i < V4L2_CID_LASTP1; i++) {
        queryctrl.id = i;
        if (0 == getQueryStruct(queryctrl)) {
            if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
                continue;
            controlData[queryctrl.id] = queryctrl;
            controlNames[(const char *)queryctrl.name] = queryctrl.id;
            if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
                vector<v4l2_querymenu> menus;
                getMenuStruct( queryctrl.id, menus);
                menuData[queryctrl.id].swap(menus);
            }
        } else {
            if (errno != EINVAL) {
                throw Exceptions::V4LControl::QueryParameters("standard controls");
            }
        }
    }
    for (queryctrl.id = V4L2_CID_PRIVATE_BASE;;queryctrl.id++) {
        if (0 == getQueryStruct(queryctrl)) {
            if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
                continue;
            controlData[queryctrl.id] = queryctrl;
            controlNames[(const char *)queryctrl.name] = queryctrl.id;
            if (queryctrl.type == V4L2_CTRL_TYPE_MENU) {
                vector<v4l2_querymenu> menus;
                getMenuStruct( queryctrl.id, menus);
                menuData[queryctrl.id].swap(menus);
            }
        } else {
            break;
        }
    }
}

unsigned int V4LControl::getId( const std::string & name ) const {
    map<string, unsigned int>::const_iterator id = controlNames.find(name);
    if(id == controlNames.end())
        return V4L2_CID_LASTP1;
    return id->second;
}

string V4LControl::getName( unsigned int id ) const {
    map<unsigned int, v4l2_queryctrl>::const_iterator control = controlData.find(id);
    if(control == controlData.end())
        return "";
    return (const char *)control->second.name;
}

vector<unsigned int> V4LControl::supportedParameters(void) const {
    vector<unsigned int> result(controlNames.size());
    vector<unsigned int>::iterator id = result.begin();
    for(map<string, unsigned int>::const_iterator param = controlNames.begin(); param != controlNames.end(); ++param, ++id)
        *id = param->second;
    return result;
}

vector<string> V4LControl::supportedParameterNames(void) const {
    vector<string> result(controlNames.size());
    vector<string>::iterator id = result.begin();
    for(map<string, unsigned int>::const_iterator param = controlNames.begin(); param != controlNames.end(); ++param, ++id)
        *id = param->first;
    return result;
}

void V4LControl::set( unsigned int id, int value){
    control.id = id;
    if(!isSupported(control.id))
        throw Exceptions::V4LControl::ParameterNotSupported(id);
    control.value = value;
    if(-1 == setControlStruct( control ))
        if( errno != EBUSY )
            throw Exceptions::V4LControl::SetValue(getName(id));
}

int V4LControl::get( unsigned int id ){
    control.id = id;
    if(!isSupported(control.id))
        throw Exceptions::V4LControl::ParameterNotSupported(id);
    if(-1 == getControlStruct( control ))
        if( errno != EBUSY )
            throw Exceptions::V4LControl::GetValue(getName(id));
    return control.value;
}

int V4LControl::type( unsigned int id ){
    return controlData[id].type;
}

map<unsigned int, string> V4LControl::menuValues( unsigned int id ) const {
    if(!isSupported(id))
        throw Exceptions::V4LControl::ParameterNotSupported(id);
    map<unsigned int, string> result;
    const vector<v4l2_querymenu> & menu = menuData.find(id)->second;
    for(vector<v4l2_querymenu>::const_iterator m = menu.begin(); m != menu.end(); m++)
        result[m->index] = (const char *)m->name;
    return result;
}

int V4LControl::defaultValue( unsigned int id ) const {
    if(!isSupported(id))
        throw Exceptions::V4LControl::ParameterNotSupported(id);
    return controlData.find(id)->second.default_value;
}

int V4LControl::min( unsigned int id ) const {
    if(!isSupported(id))
        throw Exceptions::V4LControl::ParameterNotSupported(id);
    return controlData.find(id)->second.minimum;
}

int V4LControl::max( unsigned int id ) const {
    if(!isSupported(id))
        throw Exceptions::V4LControl::ParameterNotSupported(id);
    return controlData.find(id)->second.maximum;
}

int V4LControl::step( unsigned int id ) const{
    if(!isSupported(id))
        throw Exceptions::V4LControl::ParameterNotSupported(id);
    return controlData.find(id)->second.step;
}

int V4LControl::getQueryStruct( v4l2_queryctrl & query ) const {
    return ioctl (device, VIDIOC_QUERYCTRL, &query);
}

void V4LControl::getMenuStruct( unsigned int id, vector<v4l2_querymenu> & menu ) const {
    if(!isSupported(id))
        throw Exceptions::V4LControl::ParameterNotSupported(id);
    menu.clear();
    struct v4l2_querymenu querymenu;
    memset (&querymenu, 0, sizeof (querymenu));
    querymenu.id = id;
    int last;
    const struct v4l2_queryctrl & data = controlData.find(id)->second;

    for (querymenu.index = data.minimum; (int)querymenu.index <= data.maximum; querymenu.index++) {
        if (0 != (last = ioctl (device, VIDIOC_QUERYMENU, &querymenu)))
            throw Exceptions::V4LControl::QueryParameters("reading menu item");
        menu.push_back(querymenu);
    }
}

int V4LControl::setControlStruct( v4l2_control & control ){
    return ioctl(device, VIDIOC_S_CTRL, &control);
}

int V4LControl::getControlStruct( v4l2_control & control ) const {
    return ioctl(device, VIDIOC_G_CTRL, &control);
}

}
