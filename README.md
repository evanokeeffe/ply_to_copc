File takes RGB PLY from visualSFM/openMVG/odm/Metashape/Pix4D and turns into a COPC viewable file.<br>
Uses laspy to convert the uchar PLY RGB values and convert them to LAS unsigned 16 bit int.<br>

Requires:<br>
logzero,pdalpyproj,open3d,numpy,laspy<br>