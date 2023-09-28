import os,sys,math
from logzero import logger
import pdal,json
from pyproj import CRS
import open3d as o3d
import numpy as np
import laspy

def round_up_to_nearest_100(num):
    return math.ceil(num / 100) * 100

def getPC_INFO(input_file):
    info = {}
    pcd = o3d.io.read_point_cloud(input_file)
    points = np.asarray(pcd.points)

    info["min_x"] = round_up_to_nearest_100(np.min(points[:,0]))
    info["min_y"] = round_up_to_nearest_100(np.min(points[:,1]))
    info["min_z"] = round_up_to_nearest_100(np.min(points[:,2]))

    return info

def ply2las(input_file,output_file,epsg=None):
    """
        PLY files are normally using RGB unsigned char values from 0-255
        LAS RGB requires 0-65535 values 
    """
    info = {}
    pcd = o3d.io.read_point_cloud(input_file)
    points = np.asarray(pcd.points)

    # open3D uses 0.0->1.0 floating points values, LAS requires unsigned 16 bit colors
    rgb = np.asarray(pcd.colors) * 65535
    rgb = rgb.astype(np.uint16)

    info["min_x"] = np.min(points[:,0])
    info["min_y"] = np.min(points[:,1])
    info["min_z"] = np.min(points[:,2])
    info["max_x"] = np.max(points[:,0])
    info["max_y"] = np.max(points[:,1])
    info["max_z"] = np.max(points[:,2])

    intensities = (points[:,2]-info["min_z"])/(info["max_z"]-info["min_z"])*65535
    intensities = intensities.astype(np.uint16)

    header = laspy.LasHeader(version="1.2", point_format=2) # LAS point format 2 supports color

    header.add_crs(CRS.from_epsg(epsg))
    header.generating_software = "ProvEye"
    header.point_count = np.max(points.shape)
    header.scales = [0.0001, 0.0001, 0.0001]
    header.offsets = [round_up_to_nearest_100(info["min_x"]), round_up_to_nearest_100(info["min_y"]), round_up_to_nearest_100(info["min_z"])]
    header.mins = [info["min_x"], info["min_y"], info["min_z"]]
    header.maxs = [info["max_x"], info["max_y"], info["max_z"]]

    lasfile = laspy.LasData(header)

    lasfile.x = points[:,0]
    lasfile.y = points[:,1]
    lasfile.z = points[:,2]

    lasfile.red = rgb[:,0]
    lasfile.green = rgb[:,1]
    lasfile.blue = rgb[:,2]

    lasfile.intensity = intensities

    lasfile.write(output_file)
    logger.debug("Saving las format")

def ply2las_pdal(input_file,tmp_file,epsg=None):
    dict_cmd = dict()
    dict_cmd["pipeline"] = [
        {
            "type":"readers.ply",
            "filename":input_file,
        },
        {
            "type":"writers.las",
            "filename":tmp_file,
            "a_srs":"EPSG:{}".format(epsg),
            "extra_dims":"red=uint16,green=uint16,blue=uint16",
            "scale_x":"auto", 
            "scale_y":"auto", 
            "scale_z":"auto", 
            "offset_x":"auto", 
            "offset_y":"auto", 
            "offset_z":"auto"
        },
    ]

    json_cmd = json.dumps(dict_cmd)

    logger.debug(json_cmd)

    pipeline = pdal.Pipeline(json_cmd)
    count = pipeline.execute()
    arrays = pipeline.arrays
    metadata = pipeline.metadata
    log = pipeline.log

    #logger.info("{}".format(metadata))
    #logger.info("{}".format(log))
    logger.debug("Conversion from PLY to LAS Complete")

def convertPLY2COPC(input_file,output_file,epsg=None):
    '''
        Using PDAL Convert PC to COPC
    '''

    #info = getPC_INFO(input_file)

    tmp_file = "/tmp/t_cloud.las"

    ply2las(input_file,tmp_file,epsg)
    #ply2las_pdal(input_file,tmp_file,epsg)
    convertLAS2COPC(tmp_file,output_file,epsg)

def convertLAS2COPC(input_file,output_file,epsg=None):
    '''
        Using PDAL Convert PC to COPC
    '''

    dict_cmd = dict()
    dict_cmd["pipeline"] = [
        {
            "type":"readers.las",
            "filename":input_file,
        },
        {
            "type":"writers.copc",
            "filename":output_file,
            "forward": "all",
            "a_srs":"EPSG:{}".format(epsg)
        },
    ]

    json_cmd = json.dumps(dict_cmd)

    logger.debug(json_cmd)

    pipeline = pdal.Pipeline(json_cmd)
    count = pipeline.execute()
    arrays = pipeline.arrays
    metadata = pipeline.metadata
    log = pipeline.log

    #logger.debug("{}".format(metadata))
    #logger.debug("{}".format(log))
    logger.info("Saving COPC File: {}".format(output_file))

if __name__=='__main__':
    logger.info("Converting {} -> {} EPSG:{}".format(sys.argv[1],sys.argv[2],sys.argv[3]))
    if(".ply" in sys.argv[1]):
        convertPLY2COPC(sys.argv[1],sys.argv[2],sys.argv[3])
    if(".las" in sys.argv[1]):
        convertLAS2COPC(sys.argv[1],sys.argv[2],sys.argv[3])