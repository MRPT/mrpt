import mrpt

"""A more advanced example for mrpt usage with Python"""

class CellStates(object): # fake enum
    EMPTY = 1
    UNKNOWN = 0
    OCCUPIED = 2


class MapLogger(object):
    UNKNOWN_THRESHOLD = 0.7
    
    def __init__(self):
        print 'setofiniti'
        configs = mrpt.TSetOfMetricMapInitializers()
        print 'metricinit'
        conf = mrpt.TMetricMapInitializer()
        print 'metricini.gridmap'
        conf.set_COccupancyGridMap2D(-3, 3, -3, 3, 0.02)
        print 'set.push(metricinit)'
        configs.push_back(conf)
        self.map_builder = mrpt.CMetricMapBuilderRBPF(0, 0, configs)
        self.log = mrpt.CRawlog()
        self.readings = []
    
    def get_size(self):
        sizex = self.get_map().getSizeX()
        sizey = self.get_map().getSizeY()
        return sizex, sizey
    
    def submit_reading(self, reading):
        self.readings.append(reading)
    
    def get_clearance(self, pose, distance, width):
        """ Returns a pair: a value from CellStates and distance. If first item
        is EMPTY, then whole area is clear and distance is None
        """
        m = self.get_map()
        unknown = m.nearest_obstacle(pose, distance, width, self.UNKNOWN_THRESHOLD)
        if unknown > distance:
            return CellStates.EMPTY, None
        occupied = m.nearest_obstacle(pose, distance, width, 0.2)
        if occupied > distance:
            return CellStates.UNKNOWN, unknown
        return CellStates.OCCUPIED, occupied
    
    def update_position(self, pose_change):
        sf = mrpt.CSensoryFrame()
#        print 'no of readings:', len(self.readings)
        for observation in self.readings:
            sf.insert(observation)
        self.readings = []
  #      observation = mrpt.CObservationRange(0.1, 4, 0.3)
  #      observation.insert(0, position3D(0, 0, 90), 5)
   #     sf.insert(observation)
        self.log.addObservations(sf)

        robot_increment = pose_change
        action = mrpt.CActionRobotMovement2D()
        action.computeFromOdometry(robot_increment)
        acoll = mrpt.CActionCollection()
        acoll.insert(action)
        self.log.addAction(action)
        self.map_builder.processActionObservation(acoll, sf)
        
    def get_position(self):
        return self.map_builder.getCurrentPoseEstimation()
    
    def get_map(self):
        mmap = self.map_builder.getCurrentlyBuiltMetricMap()
        return mrpt.ptr2map(mmap.m_gridMaps[0])
    
    def get_str(self):
        """Returns a string representing the grayscale image in logodd format"""
        return self.get_map().as_string()
    
    def __del__(self):
        print 'SAVING'
        self.log.saveToRawLogFile('zadanie.rawlog')
        self.map_builder.saveCurrentEstimationToImage('final')


def get_observation(obs_range):
    observation = mrpt.CObservationRange(min_range, max_range, aperture)
    sensorPose = mrpt.CPose3D(0, 0, 0, 3.1415) # sensor pose must be 3D
    observation.insert(0, sensorPose, obs_range)
    return observation

if __name__ == '__main__':
    the_map = MapLogger()
    min_range = 0.1
    max_range = 2
    aperture = 0.3
    
    the_map.submit_reading(get_observation(1.5))
    
    pose_change = mrpt.CPose2D(0.2, 0.4, 0) + mrpt.CPose2D(1, 0, 3.14 / 2) # composed poses
    the_map.update_position(pose_change) # robot pose must be 2D

    # two ways to save the map
    the_map.map_builder.saveCurrentEstimationToImage('buildermap')
    
    occupancy_map = the_map.get_map()
    occupancy_map.saveAsBitmapFile('occgrid_map.png')
    print 'byebye!'
