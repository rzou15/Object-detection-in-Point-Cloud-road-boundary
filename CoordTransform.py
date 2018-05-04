import numpy as np
import pandas as pd
import csv

PI = np.pi
a = 6378.137
b = 6356.752


def lla2ECEF(lat, lon, alt):
    """
    Given a point's Lat/Lon/Alt Coordinate, convert it to Earth-Centered-Earth-Fixed Coordinate.
    """
    sin = lambda x: np.sin(x)
    cos = lambda x: np.cos(x)
    sqrt = lambda x: np.sqrt(x)

    lat *= PI/180
    lon *=  PI/180

    f = (a - b)/a
    e = sqrt(f*(2-f))
    N = a*1000 / sqrt(1 - e**2 * (sin(lat))**2)

    x = (alt + N) * cos(lon) * cos(lat)
    y = (alt + N) * cos(lon) * sin(lat)
    z = (alt + (1-e**2)*N) * sin(lon)
    return x, y, z



def ecef2ENU(x, y, z, lat, lon, Xo, Yo, Zo):
    """
    Given a point's and an Origin's ECEF Coordinate, return the point's East-North-Up Coordinate under the Origin.
    """
    sin = lambda x: np.sin(x)
    cos = lambda x: np.cos(x)
    
    lat *= PI/180
    lon *= PI/180
    
    rot = [[-sin(lon), cos(lon), 0], 
           [-cos(lon)*sin(lat), -sin(lat)*sin(lon), cos(lat)],
           [cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)]]   
    
    p_ENU = np.matmul(rot, [x-Xo, y-Yo, z-Zo])
    
    return p_ENU[0], p_ENU[1], p_ENU[2]



def enu2CamCoord(x, y, z, qs, qx, qy, qz):
    """
    Given a point ENU Coordinate and a camera's quaternion, return the point's Camera Coordinate.
    """
    rot = [[qs**2 + qx**2 - qy**2 - qz**2, 2*qx*qy - 2*qs*qz, 2*qx*qz + 2*qs*qy], 
           [2*qx*qy + 2*qs*qz, qs**2 - qx**2 + qy**2 - qz**2, 2*qy*qz - 2*qs*qx],
           [2*qx*qz - 2*qs*qy, 2*qy*qz + 2*qs*qx, qs**2 - qx**2 - qy**2 + qz**2]]
    
    #p_ENU = np.matmul(p, [[1,0,0], [0,1,0], [0,0,-1]])
    p_CAM = np.matmul(rot, [x, y, -z])
    
    return p_CAM[0], p_CAM[1], p_CAM[2]



def main():

    cam_lat= 45.90414414
    cam_lon = 11.02845385
    cam_alt = 227.5819
    qs = 0.362114
    qx = 0.374050
    qy = 0.592222
    qz = 0.615007
    cam_Xo, cam_Yo, cam_Zo = lla2ECEF(cam_lat, cam_lon, cam_alt)

    df = pd.read_table("./final_project_data/final_project_point_cloud.fuse", header=None, sep=" ")
    df = df.values
    with open("point_cloud_camera_coord.csv", "w") as output:
        writer = csv.writer(output, lineterminator='\n')
        for i in range(0, df.shape[0]):
            lat = df[i][0]
            lon = df[i][1]
            alt = df[i][2]
            x_ECEF, y_ECEF, z_ECEF = lla2ECEF(lat, lon, alt)
            x_ENU, y_ENU, z_ENU = ecef2ENU(x_ECEF, y_ECEF, z_ECEF, lat, lon, cam_Xo, cam_Yo, cam_Zo)
            x_CAM, y_CAM, z_CAM = enu2CamCoord(x_ENU, y_ENU, z_ENU, qs, qx, qy, qz)

            writer.writerow([x_CAM, y_CAM, z_CAM, df[i][3]])



if __name__ == '__main__':
    main()

