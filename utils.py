from pyproj import transform, Proj, Transformer
import openpylivox as opl


def unify_measures(gnss):
    gnss['coords'] = WGS84_to_Grid(*gnss['coords'])
    gnss['velocity'] /= 3.6
    return gnss

# Coordinate transforms 
def WGS84_to_Grid(lon, lat):
    wgs84 = Proj(projparams = 'epsg:4326')
    OutputGrid = Proj(projparams = 'epsg:3857')
    x, y = transform(wgs84, OutputGrid, lat, lon)
    return [x, y]

def Grid_to_WGS84(x, y):
    wgs84 = Proj(projparams = 'epsg:4326')
    InputGrid = Proj(projparams = 'epsg:3857')
    lat, lon = transform(InputGrid, wgs84, x, y)
    return [lon, lat]

def Grid_to_WGS84_iter(points):
    transformer = Transformer.from_crs(3857, 4326)
    return transformer.itransform(points)



# Time HH:MM:SS/F representation
def hhmmss(synced_time):
    ls = list(map(lambda x: x.strftime('%H:%M:%S.%f'), synced_time))
    print("GNSS time: ", ls[0])
    print("IMU time: ", ls[1], '\n')
    return synced_time

# Livox convert func
def convertBin2CSV(filename):
    
    filename = 'lidar_1658737644300159744.log'
    opl.convertBin2CSV(filename)

# Basic dict for GNSS-IMU data
def pos_dict():
    pose = {"GNSS": {"timestamp": None, 
                     "coords": None,    # lon, lat (deg.f)
                     "heading": None,   # azimuth (degrees from North)
                     "velocity": None,  # km/h
                     "accuracy": None}, # m
            "IMU":  {"gyro_x": None,    # rad/s
                     "gyro_y": None,    # rad/s
                     "gyro_z": None,    # rad/s
                     "acc_x": None,     # g 
                     "acc_y": None,     # g 
                     "acc_z": None,     # g
                     "time": None}}         
    return pose